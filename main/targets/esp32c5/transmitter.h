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
#define TXDC_REG 0x600a0c08u
#include "continuous_tone.h"
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
    prepare_rx();
    if(!ok){reply("ERR dma_timeout\n");return false;}
    char h[96];
    snprintf(h,sizeof(h),"SENT %u %" PRIu32 " %" PRIu32 "\n",n*repeats,tx_cycles,tx_late);
    reply(h);
    return true;
}
#include "continuous_replay.h"
#ifdef C5_REPLAY_PROBE
#include "c5_replay_probe.h"
#endif
#include "tx_commands.h"
