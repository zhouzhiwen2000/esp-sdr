/* Opt-in, bounded hardware experiments; not part of the public burst protocol.
 * Included after S31 RX initialization helpers. Never export this image.
 * Requires the stock S31 320 MHz profile and a CoreBoard-1 with bare headers.
 * DMA <0=AXI,1=AHB> <0=SRAM copy,1=TCM read,2=TCM write,3=live TCM read>
 * PARLIO <250000|1000000|4000000|16000000> <100..10000 ms> <0=local,1=serial>
 * PARLIO uses one circular receive transaction; no stop/rearm between nodes.
 * Counters test DMA consumption/transport, not RF phase or spectral fidelity.
 */
#if CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 320
#error "S31 DMA diagnostic deadlines require the 320 MHz profile"
#endif
#include "esp_heap_caps.h"
#include "esp_private/gdma.h"
#include "esp_private/gdma_link.h"
#include "hal/axi_dma_ll.h"
#include "hal/ahb_dma_ll.h"
#include "soc/axi_dma_struct.h"
#include "soc/ahb_dma_struct.h"
#include "driver/gpio.h"
#include "driver/parlio_rx.h"
#include "esp_rom_gpio.h"
#include "esp_private/esp_gpio_reserve.h"
#include "soc/gpio_sig_map.h"
#include "modem/modem_widgets_reg.h"

#define PROBE_BYTES 16384u
static gdma_channel_handle_t probe_tx[2], probe_rx[2];
static gdma_link_list_handle_t probe_tx_list[2], probe_rx_list[2];
static int probe_channel[2];
static uint32_t *probe_src, *probe_dst;

static void probe_dma_init(unsigned backend) {
    if (probe_tx[backend]) return;
    if (!probe_src) {
        probe_src=heap_caps_aligned_alloc(64, PROBE_BYTES, MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
        probe_dst=heap_caps_aligned_alloc(64, PROBE_BYTES, MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
        assert(probe_src && probe_dst);
    }
    gdma_channel_alloc_config_t allocation={0};
    ESP_ERROR_CHECK(backend ? gdma_new_ahb_channel(&allocation,&probe_tx[backend],&probe_rx[backend])
                           : gdma_new_axi_channel(&allocation,&probe_tx[backend],&probe_rx[backend]));
    gdma_strategy_config_t strategy={.owner_check=true,.auto_update_desc=true,.eof_till_data_popped=true};
    gdma_transfer_config_t transfer={.max_data_burst_size=16,.access_ext_mem=false};
    ESP_ERROR_CHECK(gdma_apply_strategy(probe_tx[backend],&strategy));
    ESP_ERROR_CHECK(gdma_apply_strategy(probe_rx[backend],&strategy));
    ESP_ERROR_CHECK(gdma_config_transfer(probe_tx[backend],&transfer));
    ESP_ERROR_CHECK(gdma_config_transfer(probe_rx[backend],&transfer));
    uint32_t mask=0;
    ESP_ERROR_CHECK(gdma_get_free_m2m_trig_id_mask(probe_tx[backend],&mask));
    assert(mask);
    gdma_trigger_t trigger=GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_M2M,0);
    trigger.instance_id=__builtin_ctz(mask);
    ESP_ERROR_CHECK(gdma_connect(probe_tx[backend],trigger));
    ESP_ERROR_CHECK(gdma_connect(probe_rx[backend],trigger));
    ESP_ERROR_CHECK(gdma_get_channel_id(probe_rx[backend],&probe_channel[backend]));
    gdma_link_list_config_t links={.num_items=8,.item_alignment=8,.flags.check_owner=true};
    ESP_ERROR_CHECK(gdma_new_link_list(&links,&probe_tx_list[backend]));
    ESP_ERROR_CHECK(gdma_new_link_list(&links,&probe_rx_list[backend]));
}

/* All DMA setup is complete before handing TCM ownership to the modem.
 * No allocator, driver API, flash code or TCM loads within the open gate.
 * The 1 ms deadline closes the gate even if DMA cannot access live TCM.
 */
static unsigned IRAM_ATTR probe_dma_execute(unsigned backend, bool live, unsigned *status_open,
                                           unsigned *elapsed_open, unsigned *writer) {
    int ch=probe_channel[backend];
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&dump_mux);
    if (live) {
        REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,0xff000000u);
        __asm__ volatile("fence rw, rw" ::: "memory");
        REG_WRITE(DUMP_MODE,(REG_READ(DUMP_MODE)&~0x01fe0000u)|(9u<<21)|(3u<<17));
        REG_WRITE(DUMP_CTRL,(reset_ctrl&~0x803fffffu)|CAPACITY|BIT(17)|BIT(18));
        REG_WRITE(DUMP_CTRL,(reset_ctrl&~0x803fffffu)|CAPACITY|BIT(17)|BIT(31));
        REG_SET_BIT(DUMP_CTRL,BIT(19));
        REG_CLR_BIT(DUMP_CTRL,BIT(19));
        unsigned wait=esp_cpu_get_cycle_count();
        while ((REG_READ(DUMP_MODE)&0x1ffffu)<PROBE_BYTES/4+256 &&
               esp_cpu_get_cycle_count()-wait<640000u) {}
    }
    unsigned start=esp_cpu_get_cycle_count();
    if (backend) ahb_dma_ll_tx_start(&AHB_DMA,ch);
    else axi_dma_ll_tx_start(&AXI_DMA,ch);
    unsigned raw;
    do {
        raw=backend ? ahb_dma_ll_rx_get_interrupt_status(&AHB_DMA,ch,true)
                    : axi_dma_ll_rx_get_interrupt_status(&AXI_DMA,ch,true);
    } while (!(raw&(GDMA_LL_EVENT_RX_SUC_EOF|GDMA_LL_EVENT_RX_DESC_ERROR|GDMA_LL_EVENT_RX_DESC_EMPTY)) &&
             esp_cpu_get_cycle_count()-start<320000u);
    *elapsed_open=esp_cpu_get_cycle_count()-start;
    *status_open=raw;
    *writer=REG_READ(DUMP_MODE)&0x1ffffu;
    if (live) {
        REG_CLR_BIT(DUMP_CTRL,BIT(31));
        REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,0);
        __asm__ volatile("fence rw, rw" ::: "memory");
    }
    unsigned closed=esp_cpu_get_cycle_count();
    while (!(raw&(GDMA_LL_EVENT_RX_SUC_EOF|GDMA_LL_EVENT_RX_DESC_ERROR|GDMA_LL_EVENT_RX_DESC_EMPTY)) &&
           esp_cpu_get_cycle_count()-closed<320000u) {
        raw=backend ? ahb_dma_ll_rx_get_interrupt_status(&AHB_DMA,ch,true)
                    : axi_dma_ll_rx_get_interrupt_status(&AXI_DMA,ch,true);
    }
    __asm__ volatile("fence rw, rw" ::: "memory");
    taskEXIT_CRITICAL(&dump_mux);
    esp_ipc_isr_release_other_cpu();
    return raw;
}

static void probe_dma(unsigned backend, unsigned direction) {
    probe_dma_init(backend);
    REG_CLR_BIT(DUMP_CTRL,BIT(31));
    REG_WRITE(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,0);
    void *src=direction==1 || direction==3 ? (void *)dump : probe_src;
    void *dst=direction==2 ? (void *)dump : probe_dst;
    for(unsigned j=0;j<PROBE_BYTES/4;j++) ((uint32_t *)src)[j]=0x73910000u^(j*0x10203u);
    memset(dst,0xa5,PROBE_BYTES);
    ESP_ERROR_CHECK(gdma_reset(probe_tx[backend]));
    ESP_ERROR_CHECK(gdma_reset(probe_rx[backend]));
    gdma_buffer_mount_config_t tx={.buffer=src,.length=PROBE_BYTES,.buffer_alignment=4,
        .flags={.mark_eof=true,.mark_final=GDMA_FINAL_LINK_TO_NULL,.bypass_buffer_addr_align_check=true}};
    gdma_buffer_mount_config_t rx={.buffer=dst,.length=PROBE_BYTES,.buffer_alignment=4,
        .flags={.mark_final=GDMA_FINAL_LINK_TO_NULL,.bypass_buffer_addr_align_check=true}};
    ESP_ERROR_CHECK(gdma_link_mount_buffers(probe_tx_list[backend],0,&tx,1,NULL));
    ESP_ERROR_CHECK(gdma_link_mount_buffers(probe_rx_list[backend],0,&rx,1,NULL));
    int ch=probe_channel[backend];
    if(backend) {
        ahb_dma_ll_rx_clear_interrupt_status(&AHB_DMA,ch,UINT32_MAX);
        ahb_dma_ll_tx_clear_interrupt_status(&AHB_DMA,ch,UINT32_MAX);
        ahb_dma_ll_tx_set_desc_addr(&AHB_DMA,ch,gdma_link_get_head_addr(probe_tx_list[backend]));
    } else {
        axi_dma_ll_rx_clear_interrupt_status(&AXI_DMA,ch,UINT32_MAX);
        axi_dma_ll_tx_clear_interrupt_status(&AXI_DMA,ch,UINT32_MAX);
        axi_dma_ll_tx_set_desc_addr(&AXI_DMA,ch,gdma_link_get_head_addr(probe_tx_list[backend]));
    }
    ESP_ERROR_CHECK(gdma_start(probe_rx[backend],gdma_link_get_head_addr(probe_rx_list[backend])));
    unsigned raw_open=0,cycles=0,writer=0;
    unsigned raw=probe_dma_execute(backend,direction>=3,&raw_open,&cycles,&writer);
    ESP_ERROR_CHECK(gdma_stop(probe_tx[backend]));
    ESP_ERROR_CHECK(gdma_stop(probe_rx[backend]));
    unsigned mismatch=0;
    for(unsigned j=0;j<PROBE_BYTES/4;j++) if(((uint32_t *)src)[j]!=((uint32_t *)dst)[j]) mismatch++;
    unsigned old_pattern=0,zero_words=0;
    for(unsigned j=0;j<PROBE_BYTES/4;j++) {
        old_pattern+=((uint32_t *)dst)[j]==(0x73910000u^(j*0x10203u));
        zero_words+=((uint32_t *)dst)[j]==0;
    }
    reply("DMA %u %u %u %08x %08x %u %u %u %08x %08x %08x %08x %u %u\n",
          backend,direction,PROBE_BYTES,raw_open,raw,cycles,writer,mismatch,
          esp_rom_crc32_le(0,src,PROBE_BYTES),esp_rom_crc32_le(0,dst,PROBE_BYTES),
          ((uint32_t *)src)[0],((uint32_t *)dst)[0],old_pattern,zero_words);
}

#define PROBE_NODE_BYTES 4032u
#define PROBE_RING_NODES 16u
#define PROBE_EVENTS 64u
/* CoreBoard-1 header-only pins: leave USB33/34, Ethernet4..19,
 * the radio/module pins20..32, and audio50..57 untouched. */
static const int probe_pins[16]={35,36,37,38,39,40,42,43,44,45,46,47,48,49,0,1};
static struct { const void *data; unsigned bytes; } probe_events[PROBE_EVENTS];
static unsigned probe_head;
static TaskHandle_t probe_task;

static bool IRAM_ATTR probe_parlio_event(parlio_rx_unit_handle_t unit,
                                        const parlio_rx_event_data_t *event, void *arg) {
    (void)unit; (void)arg;
    unsigned head=__atomic_load_n(&probe_head,__ATOMIC_RELAXED);
    probe_events[head%PROBE_EVENTS].data=event->data;
    probe_events[head%PROBE_EVENTS].bytes=event->recv_bytes;
    __atomic_store_n(&probe_head,head+1,__ATOMIC_RELEASE);
    BaseType_t wake=pdFALSE;
    vTaskNotifyGiveFromISR(probe_task,&wake);
    return wake==pdTRUE;
}

static void probe_parlio(unsigned rate, unsigned ms, bool output) {
    uint64_t mask=0;
    for(unsigned j=0;j<16;j++) mask|=1ull<<probe_pins[j];
    if(esp_gpio_is_reserved(mask)) { reply("ERR probe_pins_reserved\n"); return; }
    uint8_t *ring=heap_caps_aligned_alloc(64,PROBE_NODE_BYTES*PROBE_RING_NODES,MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);
    uint8_t *stage=heap_caps_aligned_alloc(64,PROBE_NODE_BYTES,MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);
    if(!ring || !stage) {
        heap_caps_free(ring); heap_caps_free(stage); reply("ERR probe_memory\n"); return;
    }
    memset(ring,0xa5,PROBE_NODE_BYTES*PROBE_RING_NODES);
    uint32_t old_enable=REG_READ(HP_SYSTEM_MODEM_DIAG_EN_REG);
    uint32_t old_fix=REG_READ(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG);
    uint32_t old_exchange=REG_READ(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG);
    REG_SET_BIT(HP_SYSTEM_CLK_EN_REG,HP_SYSTEM_REG_CLK_EN);
    REG_SET_BIT(HP_SYSTEM_PROBEA_CTRL_REG,HP_SYSTEM_REG_PROBE_GLOBAL_EN);
    REG_SET_BIT(MODEM_WIDGETS_CLK_CONF_REG,MODEM_WIDGETS_CLK_EN);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG,0x32f2);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG,
              MODEM_WIDGETS_DIAG_BUS_FIX_LOW_EN|(22u<<MODEM_WIDGETS_DIAG_BUS_FIX_SEL_LOW_S));
    REG_WRITE(HP_SYSTEM_MODEM_DIAG_EN_REG,UINT32_MAX);
    gpio_config_t pins={.pin_bit_mask=mask,.mode=GPIO_MODE_INPUT_OUTPUT,
                       .pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&pins));
    /* Native diagnostic Q[9:2], I[19:12], exchanged into I then Q bytes. */
    for(unsigned bit=0;bit<16;bit++) {
        unsigned source;
        if(bit<8) source=bit+2<8 ? HP_PROBE_TOP_OUT0_IDX+bit+2 : HP_PROBE_TOP_OUT8_IDX+bit-6;
        else if(bit<12) source=HP_PROBE_TOP_OUT8_IDX+bit-4;
        else source=LCD_DATA_OUT_PAD_OUT0_IDX+bit-12;
        esp_rom_gpio_connect_out_signal(probe_pins[bit],source,false,false);
    }
    parlio_rx_unit_config_t cfg={.trans_queue_depth=1,.max_recv_size=PROBE_NODE_BYTES*PROBE_RING_NODES,
        .dma_burst_size=64,.data_width=16,.clk_src=rate<1000000 ? PARLIO_CLK_SRC_XTAL : PARLIO_CLK_SRC_DEFAULT,
        .exp_clk_freq_hz=rate,.clk_in_gpio_num=GPIO_NUM_NC,.clk_out_gpio_num=GPIO_NUM_NC,.valid_gpio_num=GPIO_NUM_NC};
    for(unsigned bit=0;bit<16;bit++) cfg.data_gpio_nums[bit]=probe_pins[bit^8];
    parlio_rx_unit_handle_t unit=NULL;
    parlio_rx_delimiter_handle_t delimiter=NULL;
    ESP_ERROR_CHECK(parlio_new_rx_unit(&cfg,&unit));
    /* The driver enables inputs without changing the output matrix. Calling
     * gpio_set_direction here would reset every output to ordinary GPIO. */
    parlio_rx_soft_delimiter_config_t delim={.sample_edge=PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order=PARLIO_BIT_PACK_ORDER_LSB,.eof_data_len=PROBE_NODE_BYTES};
    ESP_ERROR_CHECK(parlio_new_rx_soft_delimiter(&delim,&delimiter));
    parlio_rx_event_callbacks_t callbacks={.on_partial_receive=probe_parlio_event};
    probe_head=0; probe_task=xTaskGetCurrentTaskHandle();
    (void)ulTaskNotifyTake(pdTRUE,0);
    ESP_ERROR_CHECK(parlio_rx_unit_register_event_callbacks(unit,&callbacks,NULL));
    ESP_ERROR_CHECK(parlio_rx_unit_enable(unit,true));
    ESP_ERROR_CHECK(parlio_rx_soft_delimiter_start_stop(unit,delimiter,true));
    parlio_receive_config_t receive={.delimiter=delimiter,.flags.partial_rx_en=true};
    unsigned tail=0,drops=0,bytes=0,max_backlog=0,crc=0;
    int64_t start=esp_timer_get_time();
    ESP_ERROR_CHECK(parlio_rx_unit_receive(unit,ring,PROBE_NODE_BYTES*PROBE_RING_NODES,&receive));
    while(esp_timer_get_time()-start<(int64_t)ms*1000) {
        unsigned head=__atomic_load_n(&probe_head,__ATOMIC_ACQUIRE);
        unsigned backlog=head-tail;
        if(backlog>max_backlog) max_backlog=backlog;
        if(backlog>=PROBE_RING_NODES-1) { drops+=backlog;tail=head;continue; }
        if(!backlog) { (void)ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(10));continue; }
        const void *data=probe_events[tail%PROBE_EVENTS].data;
        unsigned length=probe_events[tail%PROBE_EVENTS].bytes;
        if(length>PROBE_NODE_BYTES) { drops++;tail++;continue; }
        memcpy(stage,data,length);
        head=__atomic_load_n(&probe_head,__ATOMIC_ACQUIRE);
        if(head-tail>=PROBE_RING_NODES-1) { drops+=head-tail;tail=head;continue; }
        unsigned sequence=tail++;
        /* Local mode measures DMA + full-node copying, with a checksum of
         * each node's first 64 bytes. A software CRC over 32 MB/s itself
         * saturates the consumer. Uploaded data retains full CRC coverage. */
        crc=esp_rom_crc32_le(crc,stage,output ? length : (length<64 ? length : 64));
        if(output) {
            reply("PDATA %u %u %08x\n",sequence,length,esp_rom_crc32_le(0,stage,length));
            if(!burst_serial_send(stage,length)) break;
        }
        bytes+=length;
    }
    int64_t elapsed=esp_timer_get_time()-start;
    ESP_ERROR_CHECK(parlio_rx_soft_delimiter_start_stop(unit,delimiter,false));
    ESP_ERROR_CHECK(parlio_rx_unit_disable(unit));
    unsigned events=__atomic_load_n(&probe_head,__ATOMIC_ACQUIRE);
    reply("PEND %u %u %u %u %u %u %"PRIi64" %08x\n",rate,ms,events,bytes,drops,max_backlog,elapsed,crc);
    /* Return a stopped-ring sample for distribution analysis in local mode. */
    if(!output) {
        reply("PSAMPLE %u %08x\n",PROBE_NODE_BYTES,esp_rom_crc32_le(0,ring,PROBE_NODE_BYTES));
        burst_serial_send(ring,PROBE_NODE_BYTES);
    }
    ESP_ERROR_CHECK(parlio_del_rx_delimiter(delimiter));
    ESP_ERROR_CHECK(parlio_del_rx_unit(unit));
    for(unsigned bit=0;bit<16;bit++) ESP_ERROR_CHECK(gpio_reset_pin(probe_pins[bit]));
    REG_WRITE(HP_SYSTEM_MODEM_DIAG_EN_REG,old_enable);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG,old_fix);
    REG_WRITE(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG,old_exchange);
    heap_caps_free(ring); heap_caps_free(stage);
}

static bool s31_dma_probe_command(const char *line) {
    unsigned backend,direction; char extra;
    if(sscanf(line,"DMA %u %u %c",&backend,&direction,&extra)==2 && backend<2 && direction<4) {
        probe_dma(backend,direction); return true;
    }
    unsigned rate,ms,output;
    if(sscanf(line,"PARLIO %u %u %u %c",&rate,&ms,&output,&extra)==3 &&
       (rate==250000 || rate==1000000 || rate==4000000 || rate==16000000) && ms>=100 && ms<=10000 && output<=1) {
        probe_parlio(rate,ms,output); return true;
    }
    return false;
}
