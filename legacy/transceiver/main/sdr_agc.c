/* Predictable SDR automatic gain control built on the calibrated forced-gain
 * table.  The packet PHY's native AGC is a poor fit for continuous IQ: its RF
 * saturation path can silently change analog gain while AGCRD3 still reports
 * a fixed slot.  This controller makes every gain decision explicit. */

#include "sdr_agc.h"

#include <stdatomic.h>

#include "dcoc.h"
#include "esp_timer.h"
#include "gaintable.h"

#define SDR_AGC_EVAL_INTERVAL_US 4000
#define SDR_AGC_HIGH_PEAK 360u
#define SDR_AGC_LOW_PEAK 180u
#define SDR_AGC_LOW_EVALS_BEFORE_RAISE 25u
#define SDR_AGC_RELEASE_HOLD_US 500000
#define SDR_AGC_NO_PENDING_GAIN UINT32_MAX

static struct {
    _Atomic bool active;
    _Atomic uint32_t current_gain;
    _Atomic uint32_t last_peak;
    _Atomic uint32_t gain_changes;
    uint32_t low_evals;
    int64_t last_eval_us;
    int64_t last_high_us;
    modem_config_t config;
    _Atomic uint32_t pending_dcoc_gain;
} s;

static void select_gain(uint32_t gain)
{
    modem_select_rx_gain_slot_fast(gain);
    s.current_gain = gain;
    ++s.gain_changes;

    /* Each table slot has its own analog DC correction codes. The consumer
     * chooses gain, but the producer owns the servo state; queue its re-arm. */
    atomic_store_explicit(&s.pending_dcoc_gain, gain, memory_order_release);
}

void sdr_agc_arm(const modem_config_t *config)
{
    s.active = false;
    s.last_peak = 0u;
    s.low_evals = 0u;
    s.last_eval_us = 0;
    s.last_high_us = 0;
    s.gain_changes = 0u;
    atomic_store_explicit(&s.pending_dcoc_gain, SDR_AGC_NO_PENDING_GAIN,
                          memory_order_relaxed);
    s.config = *config;
    if (config->gain_mode != GAIN_MODE_AUTO || config->loopback != 0u) {
        return;
    }

    uint32_t count = gaintable_entry_count();
    if (count == 0u) {
        return;
    }
    s.current_gain = count - 1u;
    select_gain(s.current_gain);
    s.active = true;
}

void sdr_agc_service_dcoc(void)
{
    uint32_t gain = atomic_exchange_explicit(
        &s.pending_dcoc_gain, SDR_AGC_NO_PENDING_GAIN, memory_order_acquire);
    if (gain == SDR_AGC_NO_PENDING_GAIN) {
        return;
    }
    modem_config_t dcoc_config = s.config;
    dcoc_config.gain_mode = GAIN_MODE_MANUAL;
    dcoc_config.rx_gain = gain;
    dcoc_arm(&dcoc_config);
}

bool sdr_agc_active(void)
{
    return s.active;
}

void sdr_agc_feed_peak(uint32_t peak)
{
    if (!s.active) {
        return;
    }
    s.last_peak = peak;
    int64_t now_us = esp_timer_get_time();
    if (peak >= SDR_AGC_HIGH_PEAK) {
        /* Sparse robust samples can occasionally miss a narrowband crest.
         * Hold the reduced gain after any overload so those misses cannot
         * make release pump upward while a strong carrier remains present. */
        s.last_high_us = now_us;
    }
    if (now_us - s.last_eval_us < SDR_AGC_EVAL_INTERVAL_US) {
        return;
    }
    s.last_eval_us = now_us;

    uint32_t next = s.current_gain;
    if (peak >= 500u) {
        next = next > 10u ? next - 10u : 0u;
    } else if (peak >= 450u) {
        next = next > 8u ? next - 8u : 0u;
    } else if (peak >= 400u) {
        next = next > 6u ? next - 6u : 0u;
    } else if (peak >= SDR_AGC_HIGH_PEAK) {
        next = next > 3u ? next - 3u : 0u;
    }

    if (peak >= SDR_AGC_HIGH_PEAK) {
        s.low_evals = 0u;
    /* The S31's quiet-channel robust component commonly quantizes to exactly
     * the low threshold. Include that boundary or one ambient impulse can
     * reduce gain permanently even after the channel becomes quiet again. */
    } else if (peak <= SDR_AGC_LOW_PEAK) {
        if (now_us - s.last_high_us < SDR_AGC_RELEASE_HOLD_US) {
            s.low_evals = 0u;
        } else if (++s.low_evals >= SDR_AGC_LOW_EVALS_BEFORE_RAISE) {
            uint32_t maximum = gaintable_entry_count() - 1u;
            next = next + 2u < maximum ? next + 2u : maximum;
            s.low_evals = 0u;
        }
    } else {
        s.low_evals = 0u;
    }

    if (next != s.current_gain) {
        select_gain(next);
    }
}

uint32_t sdr_agc_current_gain(void)
{
    return s.current_gain;
}

uint32_t sdr_agc_last_peak(void)
{
    return s.last_peak;
}

uint32_t sdr_agc_gain_changes(void)
{
    return s.gain_changes;
}
