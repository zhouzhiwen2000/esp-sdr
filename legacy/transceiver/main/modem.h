#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "app_config.h"

/* OTA-characterized on ESP32-S31 rev 0 with a PlutoSDR. The PLL transition is
 * near 2295 MHz at the low edge. Correct reception remains visible above
 * 2800 MHz, but sensitivity/coherence degrade toward the ~2920 MHz loss edge;
 * keep production controls inside the repeatably strong region. */
#define RF_FREQ_MIN_HZ 2300000000u
#define RF_FREQ_MAX_HZ 2800000000u

/* Expert gain words are staged in low-table slot 0; every forced entry is
 * mirrored into the second (high) gain table at the same offset. */
#define EXPERT_GAIN_SLOT 0u
#define EXPERT_GAIN_SLOT_SECOND_TABLE 80u

typedef struct {
    uint32_t rf_freq_hz;
    int32_t frequency_correction_ppb;
    uint32_t bw_mhz;
    uint32_t second_chan;
    uint32_t gain_mode;
    uint32_t rx_gain;
    uint32_t tx_gain;
    uint32_t expert_gain_word0;
    uint32_t expert_gain_word1;
    uint32_t expert_gain_word2;
    uint32_t loopback;
    uint32_t loopback_tx_gain;
    uint32_t loopback_rx_gain;
    uint32_t loopback_bb_gain;
    uint32_t tx_tone_enable;
    int32_t tx_tone0_step;
    uint32_t filter_bw_mhz;
    uint32_t rx_filter_override;
    uint32_t rx_filter_mode;
    uint32_t rx_filter_dcap;
    uint32_t dc_offset_automatic;
} modem_config_t;

void modem_prepare_direct_phy_access(void);
esp_err_t modem_init(const modem_config_t *config);
void modem_apply_rx_config(const modem_config_t *config);
void modem_apply_live_rx_config(const modem_config_t *config);
void modem_prepare_tx_replay(const modem_config_t *config);
void modem_rearm_tx_replay(void);
void modem_stop_tx_replay(void);
void modem_recover_rx_after_tx_replay(void);
void modem_force_rx_gain_slot(uint32_t slot);
void modem_select_rx_gain_slot_fast(uint32_t slot);
void modem_prepare_sdr_agc_gain_table(void);
void modem_dcoc_write_entry(uint32_t slot, uint32_t w0, uint32_t w1, uint32_t w2);
void modem_setup_rx_gain_table(void);
void modem_calibrate_dco(void);
void modem_enter_debug_mode(void);
void modem_enter_work_mode(void);
uint32_t modem_rf_freq_hz(void);
