/* Burst TX protocol shared by C5 and S3. Rates are samples/second. */
static bool tx_rate_valid(unsigned rate) {
#if CONFIG_IDF_TARGET_ESP32C5
    if(rate==250000 || rate==500000 || rate==1000000 || rate==2000000 ||
       rate==3000000 || rate==4000000 || rate==6000000)return true;
#endif
    return rate==40000000 || rate==80000000;
}

static bool tx_command(const char *line) {
    /* These handlers stop active RF before a different radio command or upload. */
    if(replay_command(line))return true;
    if(cw_command(line))return true;
    unsigned n,rate,repeats,crc;char extra;
#ifdef C5_REPLAY_PROBE
    if(sscanf(line,"RPROBE %u %x %c",&n,&crc,&extra)==2 && n<1048576) {
        replay_probe(n,crc);return true;
    }
#endif
#ifdef SAMPLE_RATE_PROBE
    if(sscanf(line,"DACCLOCK %u %c",&n,&extra)==1 && n<=2) {
        probe_dac=n;reply("OK\n");return true;
    }
    if(sscanf(line,"TXCLOCK %u %c",&n,&extra)==1 && n<8) {
        probe_tx=n;reply("OK\n");return true;
    }
#endif
    if(!strncmp(line,"TXRUN ",6) && sscanf(line,"TXRUN %u %u %u %u %c",&n,&rate,&repeats,&crc,&extra)==4 &&
       n>0 && n<=IQ_WORDS && repeats>0 && repeats<=1000 && (crc==16 || crc==20) &&
       tx_rate_valid(rate) && n<=rate/10) {
        unsigned format=crc;bool ok=true;
        reply("RUN\n");
        for(unsigned j=0;j<repeats && ok;j++) {
            uint8_t h[4];
            if(!receive_bytes(h,4)){reply("ERR upload_timeout\n");ok=false;break;}
            uint32_t checksum=h[0]|((uint32_t)h[1]<<8)|((uint32_t)h[2]<<16)|((uint32_t)h[3]<<24);
            ok=transmit(n,rate,checksum,format,1);vTaskDelay(1);
        }
        if(ok)reply("END\n");
        return true;
    }
    unsigned format=!strncmp(line,"TX16 ",5)||!strncmp(line,"LOOP16 ",7)?16:20;
    if(((!strncmp(line,"LOOP16 ",7) && sscanf(line,"LOOP16 %u %u %u %x %c",&n,&rate,&repeats,&crc,&extra)==4) ||
        (!strncmp(line,"LOOP20 ",7) && sscanf(line,"LOOP20 %u %u %u %x %c",&n,&rate,&repeats,&crc,&extra)==4)) &&
       n>0 && n<=IQ_WORDS && repeats>0 && repeats<=100000 && tx_rate_valid(rate) &&
       (uint64_t)n*repeats<=rate/10 && (rate<40000000 || repeats<=255)) {
        transmit(n,rate,crc,format,repeats);return true;
    }
    if(((!strncmp(line,"TX16 ",5) && sscanf(line,"TX16 %u %u %x %c",&n,&rate,&crc,&extra)==3) ||
        (!strncmp(line,"TX20 ",5) && sscanf(line,"TX20 %u %u %x %c",&n,&rate,&crc,&extra)==3) ||
        (!strncmp(line,"TX ",3) && sscanf(line,"TX %u %u %x %c",&n,&rate,&crc,&extra)==3)) &&
       n>0 && n<=IQ_WORDS && tx_rate_valid(rate) && n<=rate/10) {
        transmit(n,rate,crc,!strncmp(line,"TX ",3)?0:format,1);return true;
    }
    return false;
}
