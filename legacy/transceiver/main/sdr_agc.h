#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "modem.h"

/* Sample a modest, evenly-spaced subset of each chunk.  This is enough to
 * follow SDR envelope power while keeping the producer's hot path cheap. */
#define SDR_AGC_SAMPLES_PER_CHUNK 32u
#define SDR_AGC_SAMPLE_STRIDE 31u

typedef struct {
    uint32_t below_180_max;
    uint16_t band_180;
    uint16_t band_360;
    uint16_t band_400;
    uint16_t band_450;
    uint16_t band_500;
} sdr_agc_peak_hist_t;

static inline uint32_t sdr_agc_abs10(uint32_t value)
{
    int32_t signed_value = (int32_t)(value << 22) >> 22;
    return signed_value < 0 ? (uint32_t)-signed_value : (uint32_t)signed_value;
}

static inline void sdr_agc_peak_add_value(sdr_agc_peak_hist_t *hist,
                                          uint32_t value)
{
    if (value >= 500u) {
        ++hist->band_500;
    } else if (value >= 450u) {
        ++hist->band_450;
    } else if (value >= 400u) {
        ++hist->band_400;
    } else if (value >= 360u) {
        ++hist->band_360;
    } else if (value >= 180u) {
        ++hist->band_180;
    } else if (value > hist->below_180_max) {
        hist->below_180_max = value;
    }
}

static inline void sdr_agc_peak_add_word(sdr_agc_peak_hist_t *hist,
                                         uint32_t word)
{
    uint32_t i = sdr_agc_abs10(word);
    uint32_t q = sdr_agc_abs10(word >> 10);
    sdr_agc_peak_add_value(hist, i > q ? i : q);
}

static inline uint32_t sdr_agc_peak_value(const sdr_agc_peak_hist_t *hist)
{
    uint32_t count = hist->band_500;
    if (count >= 3u) return 500u;
    count += hist->band_450;
    if (count >= 3u) return 450u;
    count += hist->band_400;
    if (count >= 3u) return 400u;
    count += hist->band_360;
    if (count >= 3u) return 360u;
    count += hist->band_180;
    return count >= 3u ? 180u : hist->below_180_max;
}

/* Arm the software AGC for GAIN_MODE_AUTO, or disarm it for other modes. */
void sdr_agc_arm(const modem_config_t *config);
bool sdr_agc_active(void);

/* Feed a robust near-peak absolute I/Q component from one captured chunk. */
void sdr_agc_feed_peak(uint32_t peak);

/* Producer-core hook: applies a queued DCOC operating-point change. */
void sdr_agc_service_dcoc(void);

uint32_t sdr_agc_current_gain(void);
uint32_t sdr_agc_last_peak(void);
uint32_t sdr_agc_gain_changes(void);
