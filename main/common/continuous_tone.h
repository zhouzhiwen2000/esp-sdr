/* Native CW with a renewable host lease. Keepalive never restarts the RF tone. */
static bool cw_active;
static int64_t cw_deadline;
static void cw_stop(void) {
    if(!cw_active)return;
#if CONFIG_IDF_TARGET_ESP32C5
    esp_phy_wifi_tx_tone(0,frequency_mhz,80);
#else
    esp_phy_wifi_tx_tone(0,1,80);
#endif
    cw_active=false;rx_ready=false;prepare_rx();
}
static void cw_service(void) {
    if(cw_active && esp_timer_get_time()>=cw_deadline)cw_stop();
}
static bool cw_command(const char *line) {
    if(!strcmp(line,"CW STOP")){cw_stop();reply("OK\n");return true;}
    if(!strcmp(line,"CW KEEP")) {
        if(!cw_active)reply("ERR cw_inactive\n");
        else {cw_deadline=esp_timer_get_time()+5000000;reply("OK\n");}
        return true;
    }
    if(!strcmp(line,"CW START")) {
        if(!cw_active) {
#if CONFIG_IDF_TARGET_ESP32C5
            esp_phy_wifi_tx_tone(1,frequency_mhz,80);
#else
            esp_phy_wifi_tx_tone(1,1,80);
            s3_tune(frequency_mhz);
#endif
            rx_ready=false;cw_active=true;
        }
        cw_deadline=esp_timer_get_time()+5000000;reply("OK\n");return true;
    }
    if(!strcmp(line,"CW?")){reply(cw_active?"CW ON\n":"CW OFF\n");return true;}
    /* A new connection, tuning, capture or other radio change ends CW first. */
    if(cw_active && strcmp(line,"INFO") && strcmp(line,"CAPS") &&
       strcmp(line,"GAIN?"))cw_stop();
    return false;
}
