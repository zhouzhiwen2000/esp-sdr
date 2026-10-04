/* Function CoreBoard-1: native modem diagnostic lanes -> circular PARLIO DMA
 * -> stable SRAM staging -> UDP or PSRAM-buffered TCP IQ8 frames.
 * Optional UDP path uses GDMA staging and borrowed MAC DMA payloads.
 */
#include "ethernet_rx.h"
#ifdef CONFIG_ESP_SDR_S31_DIRECT_TX
#include "emac_direct.h"
#endif
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "driver/parlio_rx.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_private/esp_gpio_reserve.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_cpu.h"
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
#include "dma_copy.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "modem/modem_widgets_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/hp_system_reg.h"
#include "soc/parl_io_reg.h"
#include "soc/hp_sys_clkrst_reg.h"
#include "soc/soc.h"

#define CONTROL_PORT 9875
#define TCP_PORT 9876
#define TCP_BLOCKS 2048u
#define NODE_BYTES 4032u
#define RING_NODES 8u
#define EVENTS 128u
#define PAYLOAD_BYTES 1344u
static const int pins[16]={35,36,37,38,39,40,42,43,44,45,46,47,48,49,0,1};

/* Little-endian wire protocol, also defined by host/s31_receiver/protocol.py. */
typedef struct __attribute__((packed)) {
    char magic[4];
    uint8_t version, flags;
    uint16_t header_bytes;
    uint32_t session, sequence;
    uint64_t first_sample;
    uint32_t rate_hz, frequency_mhz;
    uint16_t samples, format;
    uint64_t dma_dropped_samples;
} iq_header_t;
_Static_assert(sizeof(iq_header_t)==44,"IQ network header");
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
_Static_assert(NODE_BYTES==S31_DMA_SLOTS*S31_DMA_PAYLOAD_BYTES,"DMA frame layout");
_Static_assert(PAYLOAD_BYTES==S31_DMA_PAYLOAD_BYTES,"DMA packet size");
_Static_assert(S31_DMA_HEADROOM>=42+sizeof(iq_header_t),"DMA header space");
#endif

static esp_eth_handle_t eth;
static esp_netif_t *netif;
static int control=-1, listener=-1, tcp_client=-1;
static struct sockaddr_in tcp_peer;
static bool tcp_mode, capture_done, sender_done;
static uint32_t queued_head, queued_tail, queue_max;
typedef struct {
    iq_header_t header;
    uint8_t data[NODE_BYTES];
} queued_block_t;
static queued_block_t *queued;
static esp_err_t init_error=ESP_ERR_INVALID_STATE;
static unsigned init_step;
static uint8_t source_mac[6], peer_mac[6];
static struct sockaddr_in reply_peer, stream_peer;
static bool replying, active, requested, link_up;
static unsigned link_mbps, rate_hz, center_mhz, session_id;
static uint32_t heartbeat_ms;
static TaskHandle_t worker;
static uint32_t event_head;
static uint64_t produced_samples;
static struct { const void *data; uint32_t bytes; uint64_t first; } events[EVENTS];
typedef struct {
    uint64_t sent_bytes, dma_drops, buffer_drops;
    uint32_t packets, tx_errors, max_backlog, elapsed_ms;
    uint32_t parlio_overflow, capture_clock;
    uint64_t copy_cycles, submit_cycles, wait_cycles;
    uint64_t cpu_stage_bytes, dma_stage_bytes;
    int error;
} stream_stats_t;
static stream_stats_t stats;
static portMUX_TYPE stats_lock=portMUX_INITIALIZER_UNLOCKED;

bool s31_net_active(void) { return __atomic_load_n(&active,__ATOMIC_ACQUIRE); }
bool s31_net_reply(const void *data,size_t size) {
    if(!replying) return false;
    (void)sendto(control,data,size,0,(struct sockaddr *)&reply_peer,sizeof(reply_peer));
    return true;
}

static void ethernet_event(void *arg,esp_event_base_t base,int32_t id,void *data) {
    (void)arg;(void)base;(void)data;
    bool up=id==ETHERNET_EVENT_CONNECTED;
    if(up) {
        eth_speed_t speed=ETH_SPEED_10M;
        if(esp_eth_ioctl(eth,ETH_CMD_G_SPEED,&speed)==ESP_OK)
            __atomic_store_n(&link_mbps,speed==ETH_SPEED_1000M?1000:speed==ETH_SPEED_100M?100:10,__ATOMIC_RELEASE);
    }
    if(up || id==ETHERNET_EVENT_DISCONNECTED || id==ETHERNET_EVENT_STOP) {
        __atomic_store_n(&link_up,up,__ATOMIC_RELEASE);
        if(!up) __atomic_store_n(&requested,false,__ATOMIC_RELEASE);
    }
}

#define INIT_TRY(call) do { init_step=__LINE__;init_error=(call);if(init_error!=ESP_OK)return init_error; } while(0)
esp_err_t s31_net_init(void) {
    INIT_TRY(esp_netif_init());
    esp_netif_config_t nc=ESP_NETIF_DEFAULT_ETH();
    netif=esp_netif_new(&nc);
    if(!netif) return init_error=ESP_ERR_NO_MEM;
    eth_mac_config_t mc=ETH_MAC_DEFAULT_CONFIG();
    mc.flags|=ETH_MAC_FLAG_PIN_TO_CORE;
    mc.rx_task_prio=21; /* ACK reception must preempt the TCP producer. */
    eth_esp32_emac_config_t ec=ETH_ESP32_EMAC_DEFAULT_CONFIG();
    ec.smi_gpio.mdc_num=5;ec.smi_gpio.mdio_num=6;
    init_step=__LINE__;
    esp_eth_mac_t *mac=esp_eth_mac_new_esp32(&ec,&mc);
    eth_phy_config_t pc=ETH_PHY_DEFAULT_CONFIG();pc.phy_addr=-1;pc.reset_gpio_num=7;
    esp_eth_phy_t *phy=esp_eth_phy_new_generic(&pc);
    if(!mac || !phy) return init_error=ESP_ERR_NO_MEM;
    esp_eth_config_t cfg=ETH_DEFAULT_CONFIG(mac,phy);
    INIT_TRY(esp_eth_driver_install(&cfg,&eth));
    INIT_TRY(esp_eth_ioctl(eth,ETH_CMD_G_MAC_ADDR,source_mac));
    bool auto_nego=true;
    INIT_TRY(esp_eth_ioctl(eth,ETH_CMD_S_AUTONEGO,&auto_nego));
    /* YT8531 RGMII delays from the board reference configuration. */
    uint32_t value=0xa001;
    esp_eth_phy_reg_rw_data_t reg={.reg_addr=0x1e,.reg_value_p=&value};
    INIT_TRY(esp_eth_ioctl(eth,ETH_CMD_WRITE_PHY_REG,&reg));
    reg.reg_addr=0x1f;INIT_TRY(esp_eth_ioctl(eth,ETH_CMD_READ_PHY_REG,&reg));
    value|=BIT(8);INIT_TRY(esp_eth_ioctl(eth,ETH_CMD_WRITE_PHY_REG,&reg));
    value=0xa003;reg.reg_addr=0x1e;INIT_TRY(esp_eth_ioctl(eth,ETH_CMD_WRITE_PHY_REG,&reg));
    reg.reg_addr=0x1f;INIT_TRY(esp_eth_ioctl(eth,ETH_CMD_READ_PHY_REG,&reg));
    value=(value&~255u)|0xdd;INIT_TRY(esp_eth_ioctl(eth,ETH_CMD_WRITE_PHY_REG,&reg));
    INIT_TRY(esp_netif_attach(netif,esp_eth_new_netif_glue(eth)));
    (void)esp_netif_dhcpc_stop(netif);
    esp_netif_ip_info_t ip={0};
    INIT_TRY(esp_netif_str_to_ip4("169.254.9.36",&ip.ip));
    INIT_TRY(esp_netif_str_to_ip4("255.255.0.0",&ip.netmask));
    INIT_TRY(esp_netif_set_ip_info(netif,&ip));
    INIT_TRY(esp_event_handler_register(ETH_EVENT,ESP_EVENT_ANY_ID,ethernet_event,NULL));
    control=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if(control<0) return init_error=ESP_FAIL;
    struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(CONTROL_PORT),.sin_addr.s_addr=INADDR_ANY};
    if(bind(control,(struct sockaddr *)&address,sizeof(address))<0) {
        close(control);control=-1;return init_error=ESP_FAIL;
    }
    listener=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(listener<0)return init_error=ESP_FAIL;
    address.sin_port=htons(TCP_PORT);
    if(bind(listener,(struct sockaddr *)&address,sizeof(address))<0 || listen(listener,1)<0)
        return init_error=ESP_FAIL;
    int nonblocking=1;ioctl(listener,FIONBIO,&nonblocking);
    INIT_TRY(esp_eth_start(eth));
    return init_error=ESP_OK;
}
#undef INIT_TRY

static bool resolve_mac(struct in_addr peer) {
    ip4_addr_t ip;ip4_addr_set_u32(&ip,peer.s_addr);
    struct eth_addr *address=NULL;const ip4_addr_t *cached=NULL;
    LOCK_TCPIP_CORE();
    int found=etharp_find_addr(esp_netif_get_netif_impl(netif),&ip,&address,&cached);
    bool ok=found>=0 && address;
    if(ok) memcpy(peer_mac,address->addr,6);
    UNLOCK_TCPIP_CORE();
    return ok;
}

static void put16(uint8_t *p,unsigned v) { p[0]=v>>8;p[1]=v; }
static uint16_t ip_checksum(const uint8_t *p) {
    unsigned sum=0;for(unsigned j=0;j<20;j+=2)sum+=(p[j]<<8)|p[j+1];
    while(sum>>16)sum=(sum&65535)+(sum>>16);
    return ~sum;
}
static void init_iq_header(uint8_t *header) {
    memset(header,0,42+sizeof(iq_header_t));
    memcpy(header,peer_mac,6);memcpy(header+6,source_mac,6);put16(header+12,0x800);
    uint8_t *ip=header+14,*udp=header+34;
    ip[0]=0x45;
    ip[6]=0x40;ip[8]=64;ip[9]=IPPROTO_UDP;
    uint32_t source=inet_addr("169.254.9.36");
    memcpy(ip+12,&source,4);memcpy(ip+16,&stream_peer.sin_addr.s_addr,4);
    put16(udp,CONTROL_PORT);memcpy(udp+2,&stream_peer.sin_port,2);
    iq_header_t *h=(void *)(header+42);
    memcpy(h->magic,"S31Q",4);h->version=2;h->flags=0;h->header_bytes=sizeof(*h);
    h->session=session_id;h->rate_hz=rate_hz;h->frequency_mhz=center_mhz;h->format=1;
}
static esp_err_t IRAM_ATTR send_iq(uint8_t *header,const void *data,unsigned length,uint64_t first,uint32_t sequence,uint64_t drops,stream_stats_t *st) {
    uint8_t *ip=header+14,*udp=header+34;
    put16(ip+2,28+sizeof(iq_header_t)+length);put16(ip+4,sequence);
    put16(ip+10,0);put16(ip+10,ip_checksum(ip));
    put16(udp+4,8+sizeof(iq_header_t)+length);
    iq_header_t *h=(void *)(header+42);
    h->sequence=sequence;h->first_sample=first;h->samples=length/2;
    h->dma_dropped_samples=drops;
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
    (void)data;
    esp_eth_buf_desc_t frame={.buf=header,.len=42+sizeof(iq_header_t)+length};
    uint32_t mark=esp_cpu_get_cycle_count();
    esp_err_t result=esp_eth_transmit_ctrl_bufs(eth,(void *)&s31_emac_frame_token,&frame,1);
#else
    esp_eth_buf_desc_t bufs[2]={{.buf=header,.len=42+sizeof(iq_header_t)},
                              {.buf=(uint8_t *)data,.len=length}};
    uint32_t mark=esp_cpu_get_cycle_count();
#ifdef CONFIG_ESP_SDR_S31_DIRECT_TX
    esp_err_t result=esp_eth_transmit_ctrl_bufs(eth,(void *)&s31_emac_direct_token,bufs,2);
#else
    esp_err_t result=esp_eth_transmit_ctrl_bufs(eth,NULL,bufs,2);
#endif
#endif
    st->submit_cycles+=(uint32_t)(esp_cpu_get_cycle_count()-mark);
    return result;
}

/* TCP is decoupled from acquisition by an ~8 MiB PSRAM queue. Network
 * retransmission or host scheduling must not stop the PARLIO consumer. */
static bool tcp_send_all(const uint8_t *data,unsigned size) {
    int64_t blocked=esp_timer_get_time();
    while(size) {
        int n=send(tcp_client,data,size,0);
        if(n>0) {data+=n;size-=n;blocked=esp_timer_get_time();continue;}
        if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK) &&
           esp_timer_get_time()-blocked<1000000)continue;
        return false;
    }
    return true;
}
static void tcp_sender(void *arg) {
    (void)arg;
    uint8_t frame[sizeof(iq_header_t)+NODE_BYTES] __attribute__((aligned(8)));
    for(;;) {
        unsigned tail=__atomic_load_n(&queued_tail,__ATOMIC_RELAXED);
        unsigned head=__atomic_load_n(&queued_head,__ATOMIC_ACQUIRE);
        if(tail==head) {
            if(__atomic_load_n(&capture_done,__ATOMIC_ACQUIRE))break;
            vTaskDelay(1);continue;
        }
        queued_block_t *block=&queued[tail%TCP_BLOCKS];
        unsigned length=block->header.samples*2;
        memcpy(frame,block,sizeof(iq_header_t)+length);
        __atomic_store_n(&queued_tail,tail+1,__ATOMIC_RELEASE);
        if(!tcp_send_all(frame,sizeof(iq_header_t)+length)) {
            portENTER_CRITICAL(&stats_lock);stats.tx_errors++;portEXIT_CRITICAL(&stats_lock);
            __atomic_store_n(&requested,false,__ATOMIC_RELEASE);break;
        }
        portENTER_CRITICAL(&stats_lock);stats.sent_bytes+=length;stats.packets++;portEXIT_CRITICAL(&stats_lock);
    }
    shutdown(tcp_client,SHUT_WR);
    close(tcp_client);tcp_client=-1;
    __atomic_store_n(&sender_done,true,__ATOMIC_RELEASE);
    vTaskDelete(NULL);
}
static void IRAM_ATTR publish_stats(stream_stats_t st) {
    portENTER_CRITICAL(&stats_lock);
    if(tcp_mode) {st.sent_bytes=stats.sent_bytes;st.packets=stats.packets;st.tx_errors=stats.tx_errors;}
    stats=st;
    portEXIT_CRITICAL(&stats_lock);
}

static bool IRAM_ATTR dma_event(parlio_rx_unit_handle_t unit,const parlio_rx_event_data_t *event,void *arg) {
    (void)unit;(void)arg;
    unsigned head=__atomic_load_n(&event_head,__ATOMIC_RELAXED);
    events[head%EVENTS].data=event->data;events[head%EVENTS].bytes=event->recv_bytes;
    events[head%EVENTS].first=produced_samples;produced_samples+=event->recv_bytes/2;
    __atomic_store_n(&event_head,head+1,__ATOMIC_RELEASE);
    BaseType_t wake=pdFALSE;vTaskNotifyGiveFromISR(worker,&wake);return wake==pdTRUE;
}

static void stream_task(void *arg) {
    (void)arg;
    worker=xTaskGetCurrentTaskHandle();
    sender_done=true; /* Cleanup must also work before sender creation. */
    esp_err_t error=ESP_OK;
    uint64_t pinmask=0;for(unsigned j=0;j<16;j++)pinmask|=1ull<<pins[j];
    uint8_t *ring=NULL,*stage=NULL,*stage_pool=NULL;
    unsigned stage_bytes=NODE_BYTES,stage_count=tcp_mode?1:2;
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
    if(!tcp_mode) {stage_bytes=S31_DMA_STAGE_BYTES;stage_count=3;}
#endif
    uint8_t header[42+sizeof(iq_header_t)] __attribute__((aligned(8)));
    init_iq_header(header);
    unsigned stage_index=0;
    parlio_rx_unit_handle_t unit=NULL;
    parlio_rx_delimiter_handle_t delimiter=NULL;
    bool enabled=false,pins_set=false;
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
    s31_dma_copy_t copier={0};
    uint8_t *prefetched_stage=NULL;
#endif
    uint32_t old_enable=REG_READ(HP_SYSTEM_MODEM_DIAG_EN_REG);
    uint32_t old_fix=REG_READ(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG);
    uint32_t old_exchange=REG_READ(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG);
    stream_stats_t st={0};
    int64_t start=esp_timer_get_time();
#define TRY(call) do {error=(call);if(error!=ESP_OK)goto done;}while(0)
    if(esp_gpio_is_reserved(pinmask)) {error=ESP_ERR_INVALID_STATE;goto done;}
    ring=heap_caps_aligned_alloc(64,NODE_BYTES*RING_NODES,MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);
    stage_pool=heap_caps_aligned_alloc(64,stage_bytes*stage_count,MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);
    stage=stage_pool;
    if(!ring || !stage) {error=ESP_ERR_NO_MEM;goto done;}
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
    if(!tcp_mode) {
        TRY(s31_dma_copy_init(&copier));
        /* Check full and short transfers before enabling live acquisition. */
        const unsigned sizes[]={128,1344,1346,2688,3870,NODE_BYTES};
        for(unsigned j=0;j<24;j++) {
            for(unsigned i=0;i<NODE_BYTES;i++)ring[i]=(uint8_t)(i*37u+(i>>8)+j*17u);
            memset(stage,0xa5,stage_bytes);
            unsigned size=sizes[j%(sizeof(sizes)/sizeof(sizes[0]))];
            TRY(s31_dma_copy(&copier,stage,ring,size));
            for(unsigned i=0;i<S31_DMA_SLOTS;i++) {
                unsigned n=size>i*PAYLOAD_BYTES?size-i*PAYLOAD_BYTES:0;
                if(n>PAYLOAD_BYTES)n=PAYLOAD_BYTES;
                uint8_t *slot=stage+i*S31_DMA_STRIDE;
                if(n && memcmp(slot+S31_DMA_HEADROOM,ring+i*PAYLOAD_BYTES,n)) {error=ESP_FAIL;goto done;}
                for(unsigned k=0;k<S31_DMA_STRIDE;k++)if((k<S31_DMA_HEADROOM || k>=S31_DMA_HEADROOM+n) && slot[k]!=0xa5) {
                    error=ESP_FAIL;goto done;
                }
            }
        }
        for(unsigned i=0;i<stage_count*S31_DMA_SLOTS;i++)
            init_iq_header(stage_pool+i*S31_DMA_STRIDE+S31_DMA_HEADROOM-sizeof(header));
    }
#endif
    if(tcp_mode) {
        queued=heap_caps_malloc(sizeof(*queued)*TCP_BLOCKS,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
        if(!queued) {error=ESP_ERR_NO_MEM;goto done;}
        queued_head=queued_tail=queue_max=0;capture_done=false;sender_done=false;
        if(xTaskCreatePinnedToCore(tcp_sender,"s31_tcp_tx",8192,NULL,17,NULL,0)!=pdPASS) {
            sender_done=true;error=ESP_ERR_NO_MEM;goto done;
        }
    }
    REG_SET_BIT(HP_SYSTEM_CLK_EN_REG,HP_SYSTEM_REG_CLK_EN);
    REG_SET_BIT(HP_SYSTEM_PROBEA_CTRL_REG,HP_SYSTEM_REG_PROBE_GLOBAL_EN);
    REG_SET_BIT(MODEM_WIDGETS_CLK_CONF_REG,MODEM_WIDGETS_CLK_EN);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG,0x32f2);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG,
              MODEM_WIDGETS_DIAG_BUS_FIX_LOW_EN|(22u<<MODEM_WIDGETS_DIAG_BUS_FIX_SEL_LOW_S));
    REG_WRITE(HP_SYSTEM_MODEM_DIAG_EN_REG,UINT32_MAX);
    gpio_config_t gpio={.pin_bit_mask=pinmask,.mode=GPIO_MODE_INPUT_OUTPUT,
                       .pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE};
    TRY(gpio_config(&gpio));pins_set=true;
    for(unsigned bit=0;bit<16;bit++) {
        unsigned source;
        if(bit<8)source=bit+2<8?HP_PROBE_TOP_OUT0_IDX+bit+2:HP_PROBE_TOP_OUT8_IDX+bit-6;
        else if(bit<12)source=HP_PROBE_TOP_OUT8_IDX+bit-4;
        else source=LCD_DATA_OUT_PAD_OUT0_IDX+bit-12;
        esp_rom_gpio_connect_out_signal(pins[bit],source,false,false);
    }
    parlio_rx_unit_config_t cfg={.trans_queue_depth=1,.max_recv_size=NODE_BYTES*RING_NODES,
        .dma_burst_size=64,.data_width=16,.clk_src=rate_hz<1000000?PARLIO_CLK_SRC_XTAL:PARLIO_CLK_SRC_DEFAULT,
        .exp_clk_freq_hz=rate_hz,.clk_in_gpio_num=GPIO_NUM_NC,.clk_out_gpio_num=GPIO_NUM_NC,.valid_gpio_num=GPIO_NUM_NC};
    for(unsigned bit=0;bit<16;bit++)cfg.data_gpio_nums[bit]=pins[bit^8];
    TRY(parlio_new_rx_unit(&cfg,&unit));
    st.capture_clock=REG_READ(HP_SYS_CLKRST_PARLIO_RX_CTRL0_REG);
    parlio_rx_soft_delimiter_config_t dc={.sample_edge=PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order=PARLIO_BIT_PACK_ORDER_LSB,.eof_data_len=NODE_BYTES};
    TRY(parlio_new_rx_soft_delimiter(&dc,&delimiter));
    parlio_rx_event_callbacks_t cb={.on_partial_receive=dma_event};
    event_head=0;produced_samples=0;
    TRY(parlio_rx_unit_register_event_callbacks(unit,&cb,NULL));
    TRY(parlio_rx_unit_enable(unit,true));enabled=true;
    TRY(parlio_rx_soft_delimiter_start_stop(unit,delimiter,true));
    parlio_receive_config_t receive={.delimiter=delimiter,.flags.partial_rx_en=true};
    REG_WRITE(PARL_IO_INT_CLR_REG,PARL_IO_RX_FIFO_WOVF_INT_CLR);
    TRY(parlio_rx_unit_receive(unit,ring,NODE_BYTES*RING_NODES,&receive));
    unsigned tail=0,sequence=0;uint64_t consumed=0;
    uint32_t last_stats_ms=(uint32_t)(start/1000);
    while(__atomic_load_n(&requested,__ATOMIC_ACQUIRE)) {
        uint32_t now_ms=(uint32_t)(esp_timer_get_time()/1000);
        if(now_ms-__atomic_load_n(&heartbeat_ms,__ATOMIC_ACQUIRE)>5000) break;
        bool have_stage=false;
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
        if(prefetched_stage) {
            uint32_t mark=esp_cpu_get_cycle_count();
            TRY(s31_dma_copy_wait(&copier));
            st.copy_cycles+=(uint32_t)(esp_cpu_get_cycle_count()-mark);
            st.dma_stage_bytes+=copier.length;
            stage=prefetched_stage;prefetched_stage=NULL;have_stage=true;
        }
#endif
        unsigned head=__atomic_load_n(&event_head,__ATOMIC_ACQUIRE);
        unsigned backlog=head-tail;
        if(backlog>st.max_backlog)st.max_backlog=backlog;
        if(backlog>=RING_NODES-1) {
            uint64_t end=events[(head-1)%EVENTS].first+events[(head-1)%EVENTS].bytes/2;
            st.dma_drops+=end-consumed;consumed=end;tail=head;continue;
        }
        if(!backlog) {(void)ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(10));continue;}
        const void *data=events[tail%EVENTS].data;
        unsigned length=events[tail%EVENTS].bytes;
        uint64_t first=events[tail%EVENTS].first;
        if(length>NODE_BYTES || (length&1)) {error=ESP_ERR_INVALID_SIZE;break;}
        if(!have_stage) {
          stage=stage_pool+stage_bytes*(stage_index++%stage_count);
#ifdef CONFIG_ESP_SDR_S31_DIRECT_TX
        if(!tcp_mode) {
            esp_eth_buf_desc_t fence={.buf=stage,.len=stage_bytes};
            uint32_t wait_mark=esp_cpu_get_cycle_count();
            ESP_ERROR_CHECK(esp_eth_transmit_ctrl_bufs(eth,(void *)&s31_emac_drain_token,&fence,1));
            st.wait_cycles+=(uint32_t)(esp_cpu_get_cycle_count()-wait_mark);
        }
#endif
        uint32_t copy_mark=esp_cpu_get_cycle_count();
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
        if(!tcp_mode) {TRY(s31_dma_copy(&copier,stage,data,length));st.dma_stage_bytes+=length;}
        else
#endif
        {memcpy(stage,data,length);st.cpu_stage_bytes+=length;}
        st.copy_cycles+=(uint32_t)(esp_cpu_get_cycle_count()-copy_mark);
        }
        head=__atomic_load_n(&event_head,__ATOMIC_ACQUIRE);
        if(head-tail>=RING_NODES-1)continue; /* Discard through the next loop's overrun branch. */
        tail++;consumed=first+length/2;
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
        /* Three stages allow the previous block's MAC transmission to overlap
         * the current block's headers and the next block's DMA scatter.
         * Staging buffers are reused only after their MAC loans end. */
        if(!tcp_mode && head>tail && head-tail<RING_NODES-1) {
            unsigned next_length=events[tail%EVENTS].bytes;
            if(!next_length || next_length>NODE_BYTES || (next_length&1)) {
                error=ESP_ERR_INVALID_SIZE;break;
            }
            uint8_t *next_stage=stage_pool+stage_bytes*(stage_index++%stage_count);
            esp_eth_buf_desc_t fence={.buf=next_stage,.len=stage_bytes};
            uint32_t mark=esp_cpu_get_cycle_count();
            ESP_ERROR_CHECK(esp_eth_transmit_ctrl_bufs(eth,(void *)&s31_emac_drain_token,&fence,1));
            st.wait_cycles+=(uint32_t)(esp_cpu_get_cycle_count()-mark);
            mark=esp_cpu_get_cycle_count();
            TRY(s31_dma_copy_start(&copier,next_stage,events[tail%EVENTS].data,next_length));
            st.copy_cycles+=(uint32_t)(esp_cpu_get_cycle_count()-mark);
            prefetched_stage=next_stage;
        }
#endif
        if(tcp_mode) {
            unsigned head=__atomic_load_n(&queued_head,__ATOMIC_RELAXED);
            unsigned tail=__atomic_load_n(&queued_tail,__ATOMIC_ACQUIRE);
            if(head-tail>=TCP_BLOCKS) {st.buffer_drops+=length/2;}
            else {
                queued_block_t *block=&queued[head%TCP_BLOCKS];
                iq_header_t h={.magic={'S','3','1','Q'},.version=2,.flags=0,
                    .header_bytes=sizeof(h),.session=session_id,.sequence=head,
                    .first_sample=first,.rate_hz=rate_hz,.frequency_mhz=center_mhz,
                    .samples=length/2,.format=1,.dma_dropped_samples=st.dma_drops};
                memcpy(&block->header,&h,sizeof(h));
                memcpy(block->data,stage,length);
                __atomic_store_n(&queued_head,head+1,__ATOMIC_RELEASE);
                if(head-tail+1>queue_max)__atomic_store_n(&queue_max,head-tail+1,__ATOMIC_RELEASE);
            }
        } else for(unsigned offset=0;offset<length;offset+=PAYLOAD_BYTES) {
            unsigned size=length-offset;if(size>PAYLOAD_BYTES)size=PAYLOAD_BYTES;
            uint8_t *packet_header=header;const uint8_t *payload=stage+offset;
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
            payload=stage+(offset/PAYLOAD_BYTES)*S31_DMA_STRIDE+S31_DMA_HEADROOM;
            packet_header=(uint8_t *)payload-sizeof(header);
#endif
            esp_err_t sent=send_iq(packet_header,payload,size,first+offset/2,sequence++,st.dma_drops,&st);
            if(sent==ESP_OK) {st.sent_bytes+=size;st.packets++;}else st.tx_errors++;
        }
        st.elapsed_ms=now_ms-(uint32_t)(start/1000);
        if(now_ms-last_stats_ms>=10) {
            st.parlio_overflow|=!!(REG_READ(PARL_IO_INT_RAW_REG)&PARL_IO_RX_FIFO_WOVF_INT_RAW);
            publish_stats(st);last_stats_ms=now_ms;
        }
    }
done:
    if(enabled)st.parlio_overflow|=!!(REG_READ(PARL_IO_INT_RAW_REG)&PARL_IO_RX_FIFO_WOVF_INT_RAW);
#ifdef CONFIG_ESP_SDR_S31_DIRECT_TX
    if(!tcp_mode && stage_pool) {
        esp_eth_buf_desc_t fence={.buf=stage_pool,.len=stage_bytes*stage_count};
        ESP_ERROR_CHECK(esp_eth_transmit_ctrl_bufs(eth,(void *)&s31_emac_drain_token,&fence,1));
    }
#endif
#ifdef CONFIG_ESP_SDR_S31_DMA_COPY
    if(copier.busy)ESP_ERROR_CHECK(s31_dma_copy_wait(&copier));
    s31_dma_copy_deinit(&copier);
#endif
    if(enabled) {
        (void)parlio_rx_soft_delimiter_start_stop(unit,delimiter,false);
        (void)parlio_rx_unit_disable(unit);
    }
    __atomic_store_n(&capture_done,true,__ATOMIC_RELEASE);
    if(tcp_mode) {
        while(!__atomic_load_n(&sender_done,__ATOMIC_ACQUIRE))vTaskDelay(1);
        if(tcp_client>=0) {close(tcp_client);tcp_client=-1;}
        heap_caps_free(queued);queued=NULL;
    }
    if(delimiter)(void)parlio_del_rx_delimiter(delimiter);
    if(unit)(void)parlio_del_rx_unit(unit);
    if(pins_set)for(unsigned j=0;j<16;j++)(void)gpio_reset_pin(pins[j]);
    REG_WRITE(HP_SYSTEM_MODEM_DIAG_EN_REG,old_enable);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG,old_fix);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG,old_exchange);
    heap_caps_free(ring);heap_caps_free(stage_pool);
    st.error=error;st.elapsed_ms=(esp_timer_get_time()-start)/1000;
    publish_stats(st);
    __atomic_store_n(&active,false,__ATOMIC_RELEASE);
    vTaskDelete(NULL);
#undef TRY
}

bool s31_net_command(const char *line,unsigned frequency_mhz,s31_net_reply_fn reply) {
    if(!strcmp(line,"NET?")) {
        stream_stats_t st;portENTER_CRITICAL(&stats_lock);st=stats;portEXIT_CRITICAL(&stats_lock);
        reply("NET {\"protocol_version\":2,\"payload_checksum\":\"none\",\"ip\":\"169.254.9.36\",\"link\":%s,\"mbps\":%u,\"active\":%s,\"rate\":%u,\"session\":%u,\"bytes\":%"PRIu64",\"packets\":%u,\"dma_drops\":%"PRIu64",\"tx_errors\":%u,\"backlog\":%u,\"parlio_overflow\":%u,\"capture_clock\":%u,\"elapsed_ms\":%u,\"error\":%d,\"init_error\":%d,\"heap\":%u,\"tcp\":%s,\"queue_max\":%u,\"buffer_drops\":%"PRIu64",\"internal\":%u,\"largest\":%u,\"init_step\":%u,\"copy_cycles\":%"PRIu64",\"submit_cycles\":%"PRIu64",\"wait_cycles\":%"PRIu64",\"cpu_stage_bytes\":%"PRIu64",\"dma_stage_bytes\":%"PRIu64",\"direct_tx\":%s}\n",
            __atomic_load_n(&link_up,__ATOMIC_ACQUIRE)?"true":"false",__atomic_load_n(&link_mbps,__ATOMIC_ACQUIRE),
            s31_net_active()?"true":"false",rate_hz,session_id,st.sent_bytes,st.packets,st.dma_drops,
            st.tx_errors,st.max_backlog,st.parlio_overflow,st.capture_clock,st.elapsed_ms,st.error,init_error,(unsigned)esp_get_free_heap_size(),tcp_mode?"true":"false",__atomic_load_n(&queue_max,__ATOMIC_ACQUIRE),st.buffer_drops,
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
            (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA),init_step,st.copy_cycles,st.submit_cycles,st.wait_cycles,st.cpu_stage_bytes,st.dma_stage_bytes,
#ifdef CONFIG_ESP_SDR_S31_DIRECT_TX
            tcp_mode ? "false" : "true"
#else
            "false"
#endif
        );
        return true;
    }
    if(!strcmp(line,"NETSTOP")) {
        __atomic_store_n(&requested,false,__ATOMIC_RELEASE);
        int64_t until=esp_timer_get_time()+2000000;
        while(s31_net_active() && esp_timer_get_time()<until)vTaskDelay(1);
        reply(s31_net_active()?"ERR stop_timeout\n":"OK\n");return true;
    }
    unsigned rate,session;char extra;
    if(sscanf(line,"NETPING %u %c",&session,&extra)==1) {
        if(session==session_id)__atomic_store_n(&heartbeat_ms,(uint32_t)(esp_timer_get_time()/1000),__ATOMIC_RELEASE);
        reply("PONG %u\n",session);return true;
    }
    bool use_tcp=sscanf(line,"NETSTARTTCP %u %u %c",&rate,&session,&extra)==2;
    if(use_tcp || sscanf(line,"NETSTART %u %u %c",&rate,&session,&extra)==2) {
        if(!replying) {reply("ERR udp_control_required\n");return true;}
        if(init_error!=ESP_OK || !__atomic_load_n(&link_up,__ATOMIC_ACQUIRE)) {reply("ERR ethernet_down\n");return true;}
        if(__atomic_load_n(&link_mbps,__ATOMIC_ACQUIRE)!=1000) {reply("ERR gigabit_required\n");return true;}
        if(s31_net_active()) {reply("ERR stream_active\n");return true;}
        if(rate!=250000 && rate!=1000000 && rate!=2000000 && rate!=4000000 && rate!=8000000 && rate!=16000000 && rate!=20000000 && rate!=32000000 && rate!=40000000 && rate!=53333333) {
            reply("ERR rate\n");return true;
        }
        if(use_tcp && (tcp_client<0 || tcp_peer.sin_addr.s_addr!=reply_peer.sin_addr.s_addr)) {
            reply("ERR tcp_connect_required\n");return true;
        }
        if(!use_tcp && !resolve_mac(reply_peer.sin_addr)) {reply("ERR arp_pending\n");return true;}
        stream_peer=reply_peer;rate_hz=rate;center_mhz=frequency_mhz;session_id=session;
        tcp_mode=use_tcp;sender_done=true;queue_max=0;
        portENTER_CRITICAL(&stats_lock);memset(&stats,0,sizeof(stats));portEXIT_CRITICAL(&stats_lock);
        __atomic_store_n(&heartbeat_ms,(uint32_t)(esp_timer_get_time()/1000),__ATOMIC_RELEASE);
        __atomic_store_n(&requested,true,__ATOMIC_RELEASE);__atomic_store_n(&active,true,__ATOMIC_RELEASE);
        if(xTaskCreatePinnedToCore(stream_task,"s31_eth_rx",4096,NULL,22,&worker,1)!=pdPASS) {
            __atomic_store_n(&active,false,__ATOMIC_RELEASE);reply("ERR task_memory\n");return true;
        }
        reply("OK\n");return true;
    }
    return false;
}

void s31_net_poll(void (*command)(const char *),bool serial_available) {
    if(control<0)return;
    if(listener>=0) {
        struct sockaddr_in peer;socklen_t size=sizeof(peer);
        int incoming=accept(listener,(struct sockaddr *)&peer,&size);
        if(incoming>=0) {
            if(s31_net_active())close(incoming);
            else {
                if(tcp_client>=0)close(tcp_client);
                tcp_client=incoming;tcp_peer=peer;
                struct timeval timeout={.tv_sec=0,.tv_usec=20000};
                setsockopt(tcp_client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
            }
        }
    }
    char line[128];socklen_t length=sizeof(reply_peer);
    int n=recvfrom(control,line,sizeof(line)-1,MSG_DONTWAIT,(struct sockaddr *)&reply_peer,&length);
    if(n<=0)return;
    line[n]=0;char *end=strpbrk(line,"\r\n");if(end)*end=0;
    replying=true;
    bool query=!strcmp(line,"NET?") || !strcmp(line,"INFO") || !strcmp(line,"CAPS");
    bool safe=query || !strncmp(line,"NET",3) || !strncmp(line,"FREQ ",5) ||
              !strncmp(line,"GAIN",4) || !strcmp(line,"LIMITS?");
    bool owner=reply_peer.sin_addr.s_addr==stream_peer.sin_addr.s_addr && reply_peer.sin_port==stream_peer.sin_port;
    if(!safe) {const char *text="ERR udp_command\n";s31_net_reply(text,strlen(text));}
    else if((!serial_available || (s31_net_active() && !owner)) && !query) {
        const char *text="ERR busy\n";s31_net_reply(text,strlen(text));
    } else command(line);
    replying=false;
}
