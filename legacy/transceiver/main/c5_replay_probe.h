/* Experimental C5 private modem replay hypothesis. Not built into normal firmware.
 * +0x04/+0x08/+0x18 are established by stock C5 adctrig disassembly.
 * +0x0c and its field meanings are hypotheses from related C61/S31 blocks.
 * All writes are bounded to this block and the already-reserved SRAM bank.
 */
static void replay_probe(unsigned flags,unsigned crc) {
    const unsigned n=4096;
    reply("READY\n");
    if(!receive_bytes(IQ_BUFFER,n*4)){reply("ERR upload_timeout\n");return;}
    if(esp_rom_crc32_le(0,(uint8_t *)IQ_BUFFER,n*4)!=crc){reply("ERR crc\n");return;}
    uint32_t saved[7];
    for(unsigned i=0;i<7;i++)saved[i]=REG_READ(0x600a9000+i*4);
    uint32_t owner=REG_READ(SRAM_OWNER_REG);
    esp_phy_wifi_tx_tone(1,frequency_mhz,80);
    phy_stop_tx_tone(1);
    uint32_t dc=REG_READ(TXDC_REG),scale=REG_READ(0x600a0c04);
    unsigned gain=255u>>((flags>>17)&7);
    REG_WRITE(0x600a0c04,(scale&0xff0000ffu)|(gain<<8)|(gain<<16));
    if(flags & 1) REG_WRITE(TXDC_REG,dc&~0x06000000u);
    uint32_t ctrl=0x80000000u|n;
    if(flags&2)ctrl|=1u<<19;
    ctrl|=((flags>>2)&15)<<14;
    ctrl|=((flags>>6)&255)<<20;
    taskENTER_CRITICAL(&tx_mux);
    unsigned irq=RV_CLEAR_CSR(mstatus,MSTATUS_MIE);
    REG_WRITE(0x600a9004,saved[1]&~(1u<<31));
    /* Match only the stock C5 reserved-bank owner selection. */
    REG_WRITE(SRAM_OWNER_REG,(owner&~0xf00u)|0x10200u);
    REG_WRITE(0x600a9008,(saved[2]&~0x00e00000u)|(((flags>>14)&7)<<21));
    REG_WRITE(0x600a900c,ctrl);
    uint32_t first=REG_READ(0x600a900c),start=esp_cpu_get_cycle_count();
    uint32_t done_cycles=0;
    while(esp_cpu_get_cycle_count()-start<4800000u) {
        if(!done_cycles && (REG_READ(0x600a900c)&(1u<<18)))done_cycles=esp_cpu_get_cycle_count()-start;
    }
    uint32_t last=REG_READ(0x600a900c),mode=REG_READ(0x600a9008);
    REG_WRITE(0x600a900c,last&~(1u<<31));
    REG_WRITE(SRAM_OWNER_REG,owner);
    REG_WRITE(TXDC_REG,dc);
    REG_WRITE(0x600a0c04,scale);
    REG_WRITE(0x600a9008,saved[2]);
    REG_WRITE(0x600a900c,saved[3]);
    REG_WRITE(0x600a9004,saved[1]);
    RV_SET_CSR(mstatus,irq&MSTATUS_MIE);
    taskEXIT_CRITICAL(&tx_mux);
    esp_phy_wifi_tx_tone(0,frequency_mhz,80);rx_ready=false;
    uint32_t after=esp_rom_crc32_le(0,(uint8_t *)IQ_BUFFER,n*4);
    char h[160];snprintf(h,sizeof(h),"PROBED %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %" PRIu32 "\n",ctrl,first,last,mode,after,done_cycles);reply(h);
}
