/* Shared snapshot RX gain control. Codes are PHY gain indices, not dB.
 * The controller runs before packing and selects gain for the next snapshot.
 * Keep the conservative tested range 0..50 (C5) / 0..55 (S3); do not index unverified table slots. */
extern void force_rx_gain(unsigned,unsigned,unsigned);
#if CONFIG_IDF_TARGET_ESP32C5
#define BURST_GAIN_MAX 50u
#define BURST_GAIN_REG 0x600a702cu
#else
#define BURST_GAIN_MAX 55u
#define BURST_GAIN_REG 0x6001c02cu
#endif
typedef enum { GAIN_SOFTWARE, GAIN_MANUAL, GAIN_HARDWARE } burst_gain_mode_t;
static burst_gain_mode_t gain_mode=GAIN_SOFTWARE;
static unsigned gain_code=40,gain_low_count;
static void gain_apply(void) { force_rx_gain(gain_mode!=GAIN_HARDWARE,gain_code,0); }
static bool gain_command(const char *line) {
    unsigned code;char extra;
    if(!strcmp(line,"GAIN?")) {
        char h[64];snprintf(h,sizeof(h),"GAIN %s %d 0 %u %u\n",gain_mode==GAIN_HARDWARE?"HARDWARE":gain_mode==GAIN_SOFTWARE?"AUTO":"MANUAL",gain_mode==GAIN_HARDWARE?-1:(int)gain_code,BURST_GAIN_MAX,(unsigned)((REG_READ(BURST_GAIN_REG)>>23)&1));reply(h);return true;
    }
    if(!strcmp(line,"GAIN AUTO")) {gain_mode=GAIN_SOFTWARE;gain_low_count=0;}
    else if(!strcmp(line,"GAIN HARDWARE")) {gain_mode=GAIN_HARDWARE;gain_low_count=0;}
    else if(sscanf(line,"GAIN MANUAL %u %c",&code,&extra)==1 && code<=BURST_GAIN_MAX) {
        gain_mode=GAIN_MANUAL;gain_code=code;gain_low_count=0;
    } else return false;
    gain_apply();reply("OK\n");return true;
}
static void gain_feed(const uint32_t *words,unsigned n) {
    if(gain_mode!=GAIN_SOFTWARE)return;
    /* Peak includes DC so offset-induced clipping also reduces gain. */
    unsigned peak=0;
    for(unsigned j=0;j<n;j++) {
        int i=(int)(words[j]&1023),q=(int)((words[j]>>10)&1023);
        if(i>=512)i-=1024;
        if(q>=512)q-=1024;
        unsigned ai=i<0?-i:i,aq=q<0?-q:q;
        if(ai>peak)peak=ai;
        if(aq>peak)peak=aq;
    }
    unsigned old=gain_code;
    if(peak>=420) {gain_code=gain_code>3?gain_code-4:0;gain_low_count=0;}
    else if(peak<180) {if(++gain_low_count>=4){if(gain_code<BURST_GAIN_MAX)++gain_code;gain_low_count=0;}}
    else gain_low_count=0;
    if(old!=gain_code)gain_apply();
}
