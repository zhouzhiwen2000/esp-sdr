#ifdef SAMPLE_RATE_PROBE
static uint32_t probe_replay_mode,probe_dac_digital;
static unsigned probe_dac_analog,probe_dac_filter;
static void probe_dac_apply(void) {
#if CONFIG_IDF_TARGET_ESP32C5
    probe_dac_digital=REG_READ(0x600a0448);probe_dac_analog=phy_chip_i2c_readReg(0x66,0,4);
    if(probe_dac<2){
        phy_i2c_writeReg(0x66,0,4,(probe_dac_analog&~16u)|(probe_dac<<4));
        REG_WRITE(0x600a0448,(probe_dac_digital&~12u)|(probe_dac?12u:0));
    }
#else
    probe_dac_digital=REG_READ(0x6000608c);probe_dac_analog=rom_chip_i2c_readReg(0x66,0,4);probe_dac_filter=rom_chip_i2c_readReg(0x67,0,2);
    if(probe_dac<2)rom_dac_rate_set(probe_dac);
#endif
}
static void probe_dac_restore(void) {
    if(probe_dac>=2)return;
#if CONFIG_IDF_TARGET_ESP32C5
    phy_i2c_writeReg(0x66,0,4,probe_dac_analog);REG_WRITE(0x600a0448,probe_dac_digital);
#else
    rom_chip_i2c_writeReg(0x66,0,4,probe_dac_analog);rom_chip_i2c_writeReg(0x67,0,2,probe_dac_filter);REG_WRITE(0x6000608c,probe_dac_digital);
#endif
}
#endif
/* Upload once, modem-local DMA repeats indefinitely. CPU remains available for
 * USB Stop/keepalive. Only the short register transitions mask interrupts. */
static bool replay_active;
static unsigned replay_samples,replay_rate;
static int64_t replay_deadline;
static uint32_t replay_owner;
#if CONFIG_IDF_TARGET_ESP32C5
static uint32_t replay_adc,replay_previous,replay_scale;
#endif
static void replay_stop(void) {
    if(!replay_active)return;
    taskENTER_CRITICAL(&tx_mux);
#if CONFIG_IDF_TARGET_ESP32C5
#ifdef SAMPLE_RATE_PROBE
    REG_WRITE(0x600a9008,probe_replay_mode);
#endif
    REG_WRITE(0x600a900c,0);
    REG_WRITE(SRAM_OWNER_REG,replay_owner);
    REG_WRITE(0x600a900c,replay_previous&~(1u<<31));
    REG_WRITE(0x600a9004,replay_adc&~(1u<<31));
    REG_WRITE(0x600a0c04,replay_scale);
#else
    REG_WRITE(0x60033d64,0);
    REG_WRITE(SRAM_OWNER_REG,replay_owner);
#endif
    taskEXIT_CRITICAL(&tx_mux);
#ifdef SAMPLE_RATE_PROBE
    probe_dac_restore();
#endif
#if CONFIG_IDF_TARGET_ESP32C5
    esp_phy_wifi_tx_tone(0,frequency_mhz,80);
#else
    esp_phy_wifi_tx_tone(0,1,80);
#endif
    replay_active=false;rx_ready=false;prepare_rx();
}
static void replay_service(void) {
    if(replay_active && esp_timer_get_time()>=replay_deadline)replay_stop();
}
static void replay_start(unsigned n,unsigned rate,uint32_t crc,unsigned format) {
    replay_stop();cw_stop();prepare_rx();
#if CONFIG_IDF_TARGET_ESP32C5
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
    adctrig(255,0,0,0,0,0,0,0,0);
    REG_WRITE(SRAM_OWNER_REG,owner);
    REG_WRITE(0x600a9004,REG_READ(0x600a9004)&~(1u<<31));
#endif
    reply("READY\n");
    size_t bytes=wire_size(n,format);
    if(!receive_bytes(IQ_BUFFER,bytes)){reply("ERR upload_timeout\n");return;}
    if(esp_rom_crc32_le(0,(uint8_t *)IQ_BUFFER,bytes)!=crc){reply("ERR crc\n");return;}
    if(format==16)unpack_iq8(n);else unpack_iq(n);
#if CONFIG_IDF_TARGET_ESP32C5
    esp_phy_wifi_tx_tone(1,frequency_mhz,80);
#else
    esp_phy_wifi_tx_tone(1,1,80);s3_tune(frequency_mhz);
#endif
    phy_stop_tx_tone(1);rx_ready=false;
#ifdef SAMPLE_RATE_PROBE
    probe_dac_apply();
#endif
    replay_owner=REG_READ(SRAM_OWNER_REG);
    /* Count zero + loop enable selects unlimited replay. */
    uint32_t ctrl=0x80080000u|n;
    taskENTER_CRITICAL(&tx_mux);
#if CONFIG_IDF_TARGET_ESP32C5
    replay_adc=REG_READ(0x600a9004);replay_previous=REG_READ(0x600a900c);replay_scale=REG_READ(0x600a0c04);
    REG_WRITE(0x600a0c04,(replay_scale&0xff0000ffu)|0x000f0f00u);
    REG_WRITE(0x600a9004,replay_adc&~(1u<<31));
    REG_WRITE(SRAM_OWNER_REG,(replay_owner&~0xf00u)|0x10200u);
    if(rate==40000000)ctrl|=1u<<17;
#ifdef SAMPLE_RATE_PROBE
    probe_replay_mode=REG_READ(0x600a9008);
    REG_WRITE(0x600a9008,(probe_replay_mode&~0x00e00000u)|(probe_tx<<21));
#endif
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    REG_WRITE(0x600a900c,ctrl);
#else
    REG_WRITE(0x60033d5c,0);REG_WRITE(0x60033d64,0);
    REG_WRITE(SRAM_OWNER_REG,(replay_owner&~15u)|4u);
    if(rate==80000000)ctrl|=1u<<15;
#ifdef SAMPLE_RATE_PROBE
    ctrl=(ctrl&~0x00038000u)|(probe_tx<<15);
#endif
    REG_WRITE(0x60033d64,ctrl);
#endif
    taskEXIT_CRITICAL(&tx_mux);
    replay_active=true;replay_samples=n;replay_rate=rate;
    replay_deadline=esp_timer_get_time()+5000000;
    reply("PLAYING\n");
}
static bool replay_command(const char *line) {
    if(!strcmp(line,"REPLAY STOP")){replay_stop();reply("OK\n");return true;}
    if(!strcmp(line,"REPLAY KEEP")) {
        if(!replay_active)reply("ERR replay_inactive\n");
        else {replay_deadline=esp_timer_get_time()+5000000;reply("OK\n");}
        return true;
    }
    if(!strcmp(line,"REPLAY?")) {
        char h[64];snprintf(h,sizeof(h),"REPLAY %s %u %u\n",replay_active?"ON":"OFF",replay_samples,replay_rate);reply(h);return true;
    }
    unsigned n,rate,crc;char extra;
    bool iq8=!strncmp(line,"REPLAY16 ",9);
    if((iq8?sscanf(line,"REPLAY16 %u %u %x %c",&n,&rate,&crc,&extra):
             sscanf(line,"REPLAY20 %u %u %x %c",&n,&rate,&crc,&extra))==3 &&
       n>0 && n<=IQ_WORDS && (rate==40000000||rate==80000000)) {
        replay_start(n,rate,crc,iq8?16:20);return true;
    }
    if(replay_active && strcmp(line,"INFO") && strcmp(line,"CAPS") &&
       strcmp(line,"GAIN?"))replay_stop();
    return false;
}
