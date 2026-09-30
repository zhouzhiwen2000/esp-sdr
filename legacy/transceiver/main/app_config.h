#pragma once

#include <stdint.h>

#include "sdkconfig.h"

#define IQ_CHUNK_SAMPLE_WORDS 1024u
#define IQ_CHUNKS_PER_BANK 16u
#define IQ_MAX_TRIGGER_INTERVAL_BANKS 1000000u
#define IQ_TRIGGER_CONFIG_WORDS 16u
/* Keep 128 ms of 2 MS/s complex output in PSRAM. Acquisition never stops, so
 * this queue deliberately absorbs long USB/lwIP scheduling excursions rather
 * than coupling the PARLIO producer to transport timing. */
#define IQ_STREAM_RING_CHUNKS 256u

#define RF_FREQ_DEFAULT_HZ 2412000000u
#define RF_CORRECTION_MAX_PPB 100000
#define RX_FILTER_NARROW_DCAP 60u
#define RX_FILTER_BW_OPEN 0u
#define RX_FILTER_BW_MIN_MHZ 13u
#define RX_FILTER_BW_MAX_MHZ 54u

/* Manual RX gain selects a calibrated low-table slot. The runtime table
 * capture determines the upper limit; the slot number is the reported gain
 * in 1 dB units. */
#define RX_GAIN_MIN_DB 0u
#define RX_GAIN_STEP_DB 1u

/* Experimental replay TX RF gain. This is the six-bit RFTX2 PBUS gain code,
 * not yet an absolute dBm calibration. Start conservatively: the vendor TX
 * calibration route uses code 23 and can trip the development board's
 * brownout detector at replay start on a marginal supply. */
#define TX_GAIN_MIN 0u
#define TX_GAIN_MAX 63u
#define TX_GAIN_DEFAULT 8u
/* One replay descriptor addresses at most 16,383 IQ10 words.  A host upload
 * may prebuffer 64 ordered descriptors in PSRAM for a finite TX batch. */
#define TX_REPLAY_SEGMENT_WORDS_MAX 16383u
#define TX_BATCH_SEGMENTS_MAX 64u
#define TX_BATCH_WORDS_MAX \
    (TX_REPLAY_SEGMENT_WORDS_MAX * TX_BATCH_SEGMENTS_MAX)

enum {
    GAIN_MODE_AUTO = 0u,
    GAIN_MODE_MANUAL = 1u,
    GAIN_MODE_EXPERT = 2u,
};

enum {
    IQ_TRIGGER_MODE_INTERVAL = 0u,
    IQ_TRIGGER_MODE_AGC = 1u,
    IQ_TRIGGER_MODE_POWER = 2u,
};

enum {
    SECOND_CHAN_NONE = 0u,
    SECOND_CHAN_ABOVE = 1u,
    SECOND_CHAN_BELOW = 2u,
};

typedef struct __attribute__((packed)) {
    uint32_t stream_wifi_packets;
} stream_config_t;

typedef struct __attribute__((packed)) {
    uint32_t rf_freq_hz;
    int32_t frequency_correction_ppb;
} radio_config_t;

typedef struct __attribute__((packed)) {
    uint32_t gain_mode;
    uint32_t rx_gain;
    uint32_t tx_gain;
    uint32_t expert_gain_word0;
    uint32_t expert_gain_word1;
    uint32_t expert_gain_word2;
} gain_config_t;

typedef struct __attribute__((packed)) {
    uint32_t bw_mhz;
    uint32_t second_chan;
} bandwidth_config_t;

typedef struct __attribute__((packed)) {
    uint32_t loopback;
    uint32_t loopback_tx_gain;
    uint32_t loopback_rx_gain;
    uint32_t loopback_bb_gain;
} loopback_config_t;

typedef struct __attribute__((packed)) {
    uint32_t tx_tone_enable;
    int32_t tx_tone0_step;
} tx_config_t;

typedef struct __attribute__((packed)) {
    uint32_t adc_decimation;
    uint32_t adc_source_sel;
} iq_engine_config_t;

typedef struct __attribute__((packed)) {
    uint32_t trigger_mode;
    uint32_t trigger_config[IQ_TRIGGER_CONFIG_WORDS];
} trigger_config_t;

typedef struct __attribute__((packed)) {
    /* SDR-style analog bandwidth: 0 = open/widest, otherwise MHz. */
    uint32_t filter_bw_mhz;
    /* Expert override bypasses filter_bw_mhz and exposes the raw controls. */
    uint32_t rx_filter_override;
    uint32_t rx_filter_mode;
    uint32_t rx_filter_dcap;
} rx_filter_config_t;

typedef struct __attribute__((packed)) {
    uint32_t wifi_dummy_tx_enable;
    uint32_t wifi_dummy_tx_interval_ms;
} wifi_tx_config_t;

typedef struct __attribute__((packed)) {
    /* Automatic DC correction request. Characterized dump paths use the
     * firmware analog servo; production PARLIO leaves the PHY calibration
     * untouched and lets Soapy apply safe host-side tracking. */
    uint32_t automatic;
} dc_offset_config_t;

typedef struct __attribute__((packed)) {
    stream_config_t stream;
    radio_config_t radio;
    gain_config_t gain;
    bandwidth_config_t bandwidth;
    loopback_config_t loopback;
    tx_config_t tx;
    iq_engine_config_t iq_engine;
    trigger_config_t trigger;
    rx_filter_config_t rx_filter;
    wifi_tx_config_t wifi_tx;
    dc_offset_config_t dc_offset;
} capture_config_t;
