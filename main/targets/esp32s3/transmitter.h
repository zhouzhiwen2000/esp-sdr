/* TX implementation restored from e605684. Included by the burst backend. */
#if CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 240 || CONFIG_PM_ENABLE
#error "Burst TX requires a fixed 240 MHz CPU clock"
#endif
static portMUX_TYPE tx_mux=portMUX_INITIALIZER_UNLOCKED;
static uint32_t tx_cycles,tx_late;
#define receive_bytes burst_serial_receive
#include "tx_unpack.h"
#ifdef SAMPLE_RATE_PROBE
static unsigned probe_tx,probe_dac=2;
#endif
#include "continuous_tone.h"
static bool IRAM_ATTR emit_dma(unsigned n,unsigned rate,unsigned repeats) {
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
    REG_WRITE(0x60033d5c,0);
    REG_WRITE(0x60033d64,0);
    uint32_t ctrl=0x80000000u|n;
    if(rate==80000000)ctrl|=1u<<15;
    if(repeats>1)ctrl|=(1u<<19)|(repeats<<20);
    taskENTER_CRITICAL(&tx_mux);
    REG_WRITE(SRAM_OWNER_REG,(owner&~15u)|4u);
    uint32_t start=esp_cpu_get_cycle_count();
    REG_WRITE(0x60033d64,ctrl);
    const uint32_t limit=(240000000u/rate)*n*repeats+240000u;
    bool done;
    do{done=(REG_READ(0x60033d64)&(1u<<18))!=0;tx_cycles=esp_cpu_get_cycle_count()-start;}while(!done&&tx_cycles<limit);
    REG_WRITE(0x60033d64,0);REG_WRITE(SRAM_OWNER_REG,owner);
    taskEXIT_CRITICAL(&tx_mux);tx_late=0;return done;
}
static bool transmit(unsigned n,unsigned rate,uint32_t crc,unsigned format,unsigned repeats) {
    reply("READY\n");
    size_t bytes=wire_size(n,format);
    if(!receive_bytes(IQ_BUFFER,bytes)){reply("ERR upload_timeout\n");return false;}
    if(esp_rom_crc32_le(0,(uint8_t *)IQ_BUFFER,bytes)!=crc){reply("ERR crc\n");return false;}
    if(format==16)unpack_iq8(n);else if(format==20)unpack_iq(n);

    esp_phy_wifi_tx_tone(1,1,80);
    s3_tune(frequency_mhz);
    phy_stop_tx_tone(1);
    rx_ready=false;
    bool ok=true;
    ok=emit_dma(n,rate,repeats);
    esp_phy_wifi_tx_tone(0,1,80);
    prepare_rx();
    if(!ok){reply("ERR dma_timeout\n");return false;}
    char h[96];
    snprintf(h,sizeof(h),"SENT %u %" PRIu32 " %" PRIu32 "\n",n*repeats,tx_cycles,tx_late);
    reply(h);
    return true;
}
#include "continuous_replay.h"
#include "tx_commands.h"
