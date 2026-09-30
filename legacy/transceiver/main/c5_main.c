/* ESP32-C5 USB Serial/JTAG burst transceiver. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_phy_cert_test.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_memory_layout.h"
#include "nvs_flash.h"
#include "soc/soc.h"
#include "riscv/rv_utils.h"
#include "hal/usb_serial_jtag_ll.h"

#if CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 240 || CONFIG_PM_ENABLE
#error "C5 paced TX requires a fixed 240 MHz CPU clock"
#endif

/* Ownership bit 1 covers 128 KiB. MAC_DUMP_ALLOC selects its upper 64 KiB.
 * Reserving only the upper half lets the modem corrupt task stacks/heap. */
SOC_RESERVE_MEMORY_REGION(0x40820000, 0x40840000, c5_rf_dump);
#define IQ_WORDS 16380u
#define IQ_BUFFER ((uint32_t *)0x40830000)
#define SRAM_OWNER_REG 0x60095004u
#define TXDC_REG 0x600a0c08u
extern void adctrig(uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);
extern void phy_stop_tx_tone(unsigned);
extern void phy_pbus_workmode(void);
extern void phy_pbus_xpd_rx_on(unsigned);
extern void phy_pbus_xpd_tx_off(void);
extern void phy_set_rxclk_en(unsigned);
extern void phy_chip_set_chan(unsigned,unsigned);
extern void phy_rx_filter_mode(unsigned);
static portMUX_TYPE tx_mux=portMUX_INITIALIZER_UNLOCKED;
static unsigned frequency_mhz=2412;
static bool rx_ready;
static int rx_filter=-1; /* -1 restores the PHY-calibrated automatic mode. */
static uint32_t tx_cycles,tx_late;
static int rx_analog_filter=-1;
extern unsigned phy_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void phy_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
/* RX-only analog capacitance. Preserve PHY calibration between snapshots,
 * before TX and across retunes; zero is the widest tested code, not bypass. */
static void rx_analog_apply(unsigned saved[2]) {
    for(unsigned j=0;j<2;j++) {
        saved[j]=phy_chip_i2c_readReg(0x67,1,6+j);
        if(rx_analog_filter>=0)phy_i2c_writeReg(0x67,1,6+j,(saved[j]&~63u)|(unsigned)rx_analog_filter);
    }
}
static void rx_analog_restore(const unsigned saved[2]) {
    if(rx_analog_filter>=0)for(unsigned j=0;j<2;j++)phy_i2c_writeReg(0x67,1,6+j,saved[j]);
}

/* Required by the stock RF test archive; no shell is exposed. */
int cmd_parse(char *cmd,char *name,int *argc,char **argv) {
    (void)cmd;(void)name;(void)argc;(void)argv;return -1;
}
/* Bulk transfers poll the 64-byte hardware FIFO, avoiding an ISR and RTOS
 * ring-buffer round trip for every packet. Control-idle polling still sleeps. */
static bool IRAM_ATTR __attribute__((noinline)) send_bytes(const void *data,size_t n) {
    const uint8_t *p=data;
    size_t original=n;
    int64_t deadline=esp_timer_get_time()+3000000;
    while(n) {
        if(esp_timer_get_time()>deadline)return false;
        if(!usb_serial_jtag_ll_txfifo_writable())continue;
        size_t count=n>64?64:n;
        int sent=usb_serial_jtag_ll_write_txfifo(p,count);
        usb_serial_jtag_ll_txfifo_flush();
        p+=sent;n-=sent;
    }
    if(original && original%64==0) {
        while(!usb_serial_jtag_ll_txfifo_writable())
            if(esp_timer_get_time()>deadline)return false;
        usb_serial_jtag_ll_txfifo_flush();
    }
    return true;
}
static void reply(const char *s) { (void)send_bytes(s,strlen(s)); }
static bool IRAM_ATTR __attribute__((noinline)) receive_bytes(void *data,size_t n) {
    uint8_t *p=data;
    int64_t deadline=esp_timer_get_time()+3000000;
    while(n && esp_timer_get_time()<deadline) {
        int k=usb_serial_jtag_ll_read_rxfifo(p,n>64?64:n);
        if(k>0){p+=k;n-=k;}
    }
    return n==0;
}
#include "burst_gain.h"

static void prepare_rx(void) {
    if(rx_ready)return;
    phy_chip_set_chan(frequency_mhz,0);
    phy_stop_tx_tone(1);
    phy_pbus_workmode();
    phy_pbus_xpd_tx_off();
    phy_pbus_xpd_rx_on(1);
    phy_set_rxclk_en(1);
    if(rx_filter>=0)phy_rx_filter_mode((unsigned)rx_filter);
    gain_apply();
    rx_ready=true;
}
#include "filter_probe.h"
#include "continuous_tone.h"

static size_t packed_size(unsigned n) { return (n*20u+7u)/8u; }
/* Two complete IQ10 samples occupy five bytes; an odd tail occupies three. */
static void pack_iq(unsigned n) {
    uint8_t *p=(uint8_t *)IQ_BUFFER;
    for(unsigned j=0;j<n;j+=2,p+=5) {
        uint32_t a=IQ_BUFFER[j]&0xfffffu;
        uint32_t b=j+1<n?IQ_BUFFER[j+1]&0xfffffu:0;
        p[0]=a;p[1]=a>>8;p[2]=(a>>16)|(b<<4);
        if(j+1<n){p[3]=b>>4;p[4]=b>>12;}
    }
}
static void unpack_iq(unsigned n) {
    const uint8_t *base=(uint8_t *)IQ_BUFFER;
    for(unsigned pair=(n+1)/2;pair-->0;) {
        const uint8_t *p=base+pair*5;
        uint32_t a=p[0]|((uint32_t)p[1]<<8)|((uint32_t)(p[2]&15)<<16);
        if(pair*2+1<n) {
            uint32_t b=(p[2]>>4)|((uint32_t)p[3]<<4)|((uint32_t)p[4]<<12);
            IQ_BUFFER[pair*2+1]=b;
        }
        IQ_BUFFER[pair*2]=a;
    }
}
/* IQ8 is signed two's complement I then Q. Retain each IQ10 field's
 * upper eight bits (arithmetic truncation); TX restores two zero low bits. */
static void pack_iq8(unsigned n) {
    uint8_t *p=(uint8_t *)IQ_BUFFER;
    for(unsigned j=0;j<n;j++) {
        uint32_t w=IQ_BUFFER[j];p[2*j]=(w>>2)&255;p[2*j+1]=(w>>12)&255;
    }
}
static void unpack_iq8(unsigned n) {
    const uint8_t *p=(const uint8_t *)IQ_BUFFER;
    for(unsigned j=n;j-->0;) {
        uint32_t i=p[2*j],q=p[2*j+1];IQ_BUFFER[j]=(i<<2)|(q<<12);
    }
}
static size_t wire_size(unsigned n,unsigned format) {
    return format==16?n*2:format==20?packed_size(n):n*4;
}
#ifdef SAMPLE_RATE_PROBE
/* Volatile, bounded dump-clock/source investigation; excluded from releases. */
static unsigned probe_source,probe_clock,probe_tx,probe_adc=4,probe_dac=2;
extern void phy_adc_rate_set(unsigned);
extern unsigned phy_chip_i2c_readReg(unsigned,unsigned,unsigned);
extern void phy_i2c_writeReg(unsigned,unsigned,unsigned,unsigned);
static bool probe_capture;
#endif
static bool capture(unsigned n,unsigned divider,unsigned format) {
    prepare_rx();
    
    for(unsigned j=0;j<n;j++)IQ_BUFFER[j]=0xa5a0055au;
    unsigned analog_saved[2];rx_analog_apply(analog_saved);
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
#ifdef SAMPLE_RATE_PROBE
    unsigned adc_saved=phy_chip_i2c_readReg(0x66,0,4);
    uint32_t adc_digital_saved=REG_READ(0x600a0448);
    if(probe_adc<2)phy_adc_rate_set(probe_adc);
#endif
    int64_t start=esp_timer_get_time();
    /* Vendor selector 0 maps to raw source 15 and pulses the software trigger.
     * In particular, do NOT set CTRL bit 17 as in the C61 continuous backend:
     * on this C5 it produced only a short, incomplete snapshot. */
#ifdef SAMPLE_RATE_PROBE
    if(probe_capture && probe_source>0) {
        /* Seed stock packing/clock setup, then bounded raw-source capture. */
        adctrig(255,0,0,probe_clock*2u,0,0,0,0,0);
        for(unsigned j=0;j<n;j++)IQ_BUFFER[j]=0xa5a0055au;
        uint32_t mode=REG_READ(0x600a9008);
        REG_WRITE(0x600a9008,(mode&~0x001e0000u)|((probe_source-1)<<17));
        REG_WRITE(SRAM_OWNER_REG,(owner&~0xf00u)|0x10200u);
        uint32_t ctrl=0x80000000u|n;
        REG_WRITE(0x600a9004,ctrl);
        REG_WRITE(0x600a9004,ctrl|(1u<<19));REG_WRITE(0x600a9004,ctrl);
        int64_t wait_start=esp_timer_get_time();
        while(!(REG_READ(0x600a9004)&(1u<<22)) && esp_timer_get_time()-wait_start<20000){}
        REG_WRITE(0x600a9004,0);REG_WRITE(0x600a9008,mode);
    } else adctrig(n-1,0,0,(probe_capture?probe_clock:divider)*2u,0,0,0,0,0);
#else
    adctrig(n-1,0,0,divider*2u,0,0,0,0,0);
#endif
    uint32_t elapsed=(uint32_t)(esp_timer_get_time()-start);
#ifdef SAMPLE_RATE_PROBE
    if(probe_adc<2){phy_i2c_writeReg(0x66,0,4,adc_saved);REG_WRITE(0x600a0448,adc_digital_saved);}
#endif
    REG_WRITE(SRAM_OWNER_REG,owner);
    rx_analog_restore(analog_saved);
    for(unsigned j=0;j<n;j++) {
        if(IQ_BUFFER[j]==0xa5a0055au){reply("ERR capture_timeout\n");return false;}
    }
    gain_feed(IQ_BUFFER,n);
    size_t bytes=wire_size(n,format);
    if(format==16)pack_iq8(n);else if(format==20)pack_iq(n);
    uint32_t crc=esp_rom_crc32_le(0,(const uint8_t *)IQ_BUFFER,bytes);
    char h[96];
    snprintf(h,sizeof(h),"DATA %u %08" PRIx32 " %" PRIu32 "\n",n,crc,elapsed);
    return send_bytes(h,strlen(h)) && send_bytes(IQ_BUFFER,bytes);
}
static void IRAM_ATTR __attribute__((noinline)) emit_iq(uint32_t *words,unsigned n,unsigned rate,unsigned repeats) {
    const uint32_t original=REG_READ(TXDC_REG);
    /* Preserve bit 25 selected by phy_iq_swap_set for the RF band. Forcing
     * the low-band value mirrors CPU TX at 5 GHz; bit 26 enables injection. */
    const uint32_t control=(original&~0xfffffu)|0x04000000u;
    const uint32_t step=240000000u/rate;
    for(unsigned j=0;j<n;j++)words[j]=control|(words[j]&0xfffffu);
    uint32_t late=0;
    taskENTER_CRITICAL(&tx_mux);
    unsigned irq_state=RV_CLEAR_CSR(mstatus,MSTATUS_MIE);
    /* Give the first sample a full interval for register/loop preparation. */
    uint32_t start=esp_cpu_get_cycle_count()+step,target=start;
    for(unsigned repeat=0;repeat<repeats;repeat++) {
        for(unsigned j=0;j<n;j++) {
            uint32_t value=words[j],now;
            do{now=esp_cpu_get_cycle_count();}while((int32_t)(now-target)<0);
            uint32_t lag=now-target;if(lag>late)late=lag;
            REG_WRITE(TXDC_REG,value);
            target+=step;
        }
    }
    while((int32_t)(esp_cpu_get_cycle_count()-target)<0){}
    tx_cycles=esp_cpu_get_cycle_count()-start;
    tx_late=late;
    REG_WRITE(TXDC_REG,original);
    RV_SET_CSR(mstatus,irq_state & MSTATUS_MIE);
    taskEXIT_CRITICAL(&tx_mux);
}
/* Private modem SRAM reader, identified by C5 hardware/B210 experiments.
 * 0x600a900c: size[13:0], half-rate[17], done[18], loop[19],
 * repeat count[27:20] (0 means infinite with loop set), enable[31].
 * This is modem-local DMA, independent of the general GDMA channels. */
static bool IRAM_ATTR __attribute__((noinline)) emit_dma(unsigned n,unsigned rate,unsigned repeats) {
    uint32_t owner=REG_READ(SRAM_OWNER_REG),adc=REG_READ(0x600a9004);
    uint32_t replay=REG_READ(0x600a900c),scale=REG_READ(0x600a0c04);
    /* The stock tone stop restores maximum DAC scaling, which clips native
     * replay. A conservative fixed scale preserves two-tone amplitude ratio. */
    REG_WRITE(0x600a0c04,(scale&0xff0000ffu)|0x000f0f00u);
    uint32_t ctrl=0x80000000u|n;
    if(rate==40000000)ctrl|=1u<<17;
    if(repeats>1)ctrl|=(1u<<19)|(repeats<<20);
    const uint32_t limit=(240000000u/rate)*n*repeats+24000u;
    taskENTER_CRITICAL(&tx_mux);
    unsigned irq=RV_CLEAR_CSR(mstatus,MSTATUS_MIE);
    REG_WRITE(0x600a9004,adc&~(1u<<31));
    REG_WRITE(SRAM_OWNER_REG,(owner&~0xf00u)|0x10200u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    uint32_t start=esp_cpu_get_cycle_count();
    REG_WRITE(0x600a900c,ctrl);
    bool done=false;
    do {
        done=(REG_READ(0x600a900c)&(1u<<18))!=0;
        tx_cycles=esp_cpu_get_cycle_count()-start;
    } while(!done && tx_cycles<limit);
    REG_WRITE(0x600a900c,ctrl&~(1u<<31));
    REG_WRITE(SRAM_OWNER_REG,owner);
    REG_WRITE(0x600a900c,replay&~(1u<<31));
    REG_WRITE(0x600a9004,adc&~(1u<<31));
    REG_WRITE(0x600a0c04,scale);
    RV_SET_CSR(mstatus,irq&MSTATUS_MIE);
    taskEXIT_CRITICAL(&tx_mux);
    tx_late=0; /* Not a CPU-paced stream; completion/timeout is checked above. */
    return done;
}
static bool transmit(unsigned n,unsigned rate,uint32_t crc,unsigned format,unsigned repeats) {
    if(rate>=40000000) {
        /* Initialize dump clocks and the reserved bank before receiving IQ.
         * adctrig writes the SRAM, so it must never run after payload upload. */
        prepare_rx();
        uint32_t owner=REG_READ(SRAM_OWNER_REG);
        adctrig(255,0,0,0,0,0,0,0,0);
        REG_WRITE(SRAM_OWNER_REG,owner);
        REG_WRITE(0x600a9004,REG_READ(0x600a9004)&~(1u<<31));
    }
    reply("READY\n");
    size_t bytes=wire_size(n,format);
    if(!receive_bytes(IQ_BUFFER,bytes)){reply("ERR upload_timeout\n");return false;}
    if(esp_rom_crc32_le(0,(uint8_t *)IQ_BUFFER,bytes)!=crc){reply("ERR crc\n");return false;}
    if(format==16)unpack_iq8(n);else if(format==20)unpack_iq(n);
    
    esp_phy_wifi_tx_tone(1,frequency_mhz,80);
    phy_stop_tx_tone(1);
    rx_ready=false;
    bool ok=true;
    if(rate>=40000000)ok=emit_dma(n,rate,repeats);
    else emit_iq(IQ_BUFFER,n,rate,repeats);
    esp_phy_wifi_tx_tone(0,frequency_mhz,80);
    if(!ok){reply("ERR dma_timeout\n");return false;}
    char h[96];
    snprintf(h,sizeof(h),"SENT %u %" PRIu32 " %" PRIu32 "\n",n*repeats,tx_cycles,tx_late);
    reply(h);
    return true;
}
#ifdef C5_REPLAY_PROBE
#include "c5_replay_probe.h"
#endif
#include "continuous_replay.h"

void app_main(void) {
    esp_log_level_set("*",ESP_LOG_NONE);
    esp_err_t e=nvs_flash_init();
    if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());e=nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    usb_serial_jtag_driver_config_t usb={.tx_buffer_size=8192,.rx_buffer_size=8192};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    esp_log_level_set("*",ESP_LOG_NONE);
    ESP_ERROR_CHECK(usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(1000)));
    ESP_ERROR_CHECK(usb_serial_jtag_driver_uninstall());
    reply("C5SDR READY\n");
    char line[128];size_t used=0;bool overflow=false;
    for(;;) {
        replay_service();cw_service();
        char ch;
        if(usb_serial_jtag_ll_read_rxfifo((uint8_t *)&ch,1)!=1){vTaskDelay(1);continue;}
        if(ch=='\r')continue;
        if(ch!='\n') {
            if(used<sizeof(line)-1)line[used++]=ch;else overflow=true;
            continue;
        }
        line[used]=0;used=0;
        if(overflow){overflow=false;reply("ERR command_length\n");continue;}
#ifdef FILTER_REGISTER_PROBE
        if(filter_probe_command(line))continue;
#endif
        if(replay_command(line))continue;
        if(cw_command(line))continue;
        if(gain_command(line))continue;
        unsigned n,rate,crc,repeats;char extra;uint64_t nonce;
        bool iq8=false;
        if(!strncmp(line,"CAP16 ",6)){memcpy(line,"CAP20",5);iq8=true;}
        if(!strncmp(line,"TX16 ",5)){memcpy(line,"TX20",4);iq8=true;}
        if(!strncmp(line,"LOOP16 ",7)){memcpy(line,"LOOP20",6);iq8=true;}
        if(sscanf(line,"SYNC %" SCNu64 " %c",&nonce,&extra)==1) {
            char answer[48];snprintf(answer,sizeof(answer),"SYNC %" PRIu64 "\n",nonce);reply(answer);
        }
#ifdef C5_REPLAY_PROBE
        else if(sscanf(line,"RPROBE %u %x %c",&n,&crc,&extra)==2 && n<1048576) replay_probe(n,crc);
#endif
#ifdef C5_TUNE_PROBE
        else if(sscanf(line,"FREQEX %u %c",&n,&extra)==1 && n>=100 && n<=7500) {
            frequency_mhz=n;rx_ready=false;prepare_rx();reply("OK\n");
        }
#endif
        else if(sscanf(line,"RXRUN %u %u %u %u %c",&n,&rate,&repeats,&crc,&extra)==4 &&
                n>=256 && n<=IQ_WORDS && rate<=5 && repeats>0 && repeats<=1000 && (crc==16 || crc==20)) {
            /* Format 16 = IQ8, 20 = IQ10 packed. Each frame
             * is a separate capture, with RF gaps during USB transfer. */
            bool ok=true;
            for(unsigned j=0;j<repeats && ok;j++){ok=capture(n,rate,crc);vTaskDelay(1);}
            if(ok)reply("END\n");
        }
        else if(sscanf(line,"TXRUN %u %u %u %u %c",&n,&rate,&repeats,&crc,&extra)==4 &&
                n>0 && n<=IQ_WORDS && repeats>0 && repeats<=1000 && (crc==16 || crc==20) &&
                (rate==250000||rate==500000||rate==1000000||rate==2000000||rate==3000000||
                 rate==4000000||rate==6000000||rate==40000000||rate==80000000) && n<=rate/10) {
            unsigned format=crc;bool ok=true;
            reply("RUN\n");
            for(unsigned j=0;j<repeats && ok;j++) {
                uint8_t h[4];
                if(!receive_bytes(h,4)){reply("ERR upload_timeout\n");ok=false;break;}
                uint32_t checksum=h[0]|((uint32_t)h[1]<<8)|((uint32_t)h[2]<<16)|((uint32_t)h[3]<<24);
                ok=transmit(n,rate,checksum,format,1);vTaskDelay(1);
            }
            if(ok)reply("END\n");
        }
#ifdef SAMPLE_RATE_PROBE
        else if(sscanf(line,"RXPROBE %u %u %c",&n,&rate,&extra)==2 && n<17 && rate<8) {
            probe_source=n;probe_clock=rate;probe_capture=true;capture(16380,0,20);probe_capture=false;
        }
        else if(sscanf(line,"ADCCLOCK %u %c",&n,&extra)==1 && (n<2 || n==4)) {probe_adc=n;reply("OK\n");}
        else if(sscanf(line,"DACCLOCK %u %c",&n,&extra)==1 && n<=2) {probe_dac=n;reply("OK\n");}
        else if(sscanf(line,"TXCLOCK %u %c",&n,&extra)==1 && n<8) {probe_tx=n;reply("OK\n");}
#endif
        else if(!strcmp(line,"CAPS")) {
            reply("CAPS REPLAY CW GAIN HWAGC IQ8 LPF LPF12 ALPF" "\n");
        }
        else if(!strcmp(line,"ALPF AUTO")){rx_analog_filter=-1;reply("OK\n");}
        else if(sscanf(line,"ALPF %u %c",&n,&extra)==1 && n<=63){rx_analog_filter=(int)n;reply("OK\n");}
        else if(!strcmp(line,"ALPF?")) {
            prepare_rx();char answer[64];snprintf(answer,sizeof(answer),"ALPF %d %u %u\n",rx_analog_filter,
                phy_chip_i2c_readReg(0x67,1,6),phy_chip_i2c_readReg(0x67,1,7));reply(answer);
        }
        else if(!strcmp(line,"LPF AUTO")){rx_filter=-1;rx_ready=false;prepare_rx();reply("OK\n");}
        else if(sscanf(line,"LPF %u %c",&n,&extra)==1 && (n==0 || n==4 || n==8 || n==12)) {
            rx_filter=(int)n;rx_ready=false;prepare_rx();reply("OK\n");
        }
        else if(!strcmp(line,"LPF?")) {
            char answer[64];snprintf(answer,sizeof(answer),"LPF %d %u\n",rx_filter,(unsigned)((REG_READ(0x600a0430)>>18)&15));reply(answer);
        }
        else if(!strcmp(line,"INFO")) reply("C5SDR 6 burst 16380\n");
        else if(sscanf(line,"FREQ %u %c",&n,&extra)==1 && ((n>=2100 && n<=2700)||(n>=4800 && n<=6000))) {
            frequency_mhz=n;rx_ready=false;prepare_rx();reply("OK\n");
        } else if(((!strncmp(line,"CAP ",4) && sscanf(line,"CAP %u %u %c",&n,&rate,&extra)==2) ||
                   (!strncmp(line,"CAP20 ",6) && sscanf(line,"CAP20 %u %u %c",&n,&rate,&extra)==2)) &&
                   n>=256 && n<=IQ_WORDS && rate<=5) capture(n,rate,!strncmp(line,"CAP20 ",6)? (iq8?16:20):0);
        else if(!strncmp(line,"LOOP20 ",7) &&
                sscanf(line,"LOOP20 %u %u %u %x %c",&n,&rate,&repeats,&crc,&extra)==4 &&
                n>0 && n<=IQ_WORDS && repeats>0 && repeats<=100000 &&
                (rate==250000||rate==500000||rate==1000000||rate==2000000||rate==3000000||
                 rate==4000000||rate==6000000||rate==40000000||rate==80000000) &&
                (uint64_t)n*repeats<=rate/10 && (rate<40000000 || repeats<=255))
            transmit(n,rate,crc,iq8?16:20,repeats);
        else if(((!strncmp(line,"TX ",3) && sscanf(line,"TX %u %u %x %c",&n,&rate,&crc,&extra)==3) ||
                 (!strncmp(line,"TX20 ",5) && sscanf(line,"TX20 %u %u %x %c",&n,&rate,&crc,&extra)==3)) && n>0 && n<=IQ_WORDS &&
                (rate==250000||rate==500000||rate==1000000||rate==2000000||rate==3000000||
                 rate==4000000||rate==6000000||rate==40000000||rate==80000000) &&
                 n<=rate/10) transmit(n,rate,crc,!strncmp(line,"TX20 ",5)?(iq8?16:20):0,1);
        else reply("ERR command\n");
    }
}
