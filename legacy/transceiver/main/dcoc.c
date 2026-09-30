/*
 * Continuous DC offset compensation (DCOC) for the forced-gain modes.
 *
 * At forced high RX gains the residual LO self-mixing DC reaches hundreds of
 * ADC counts and clips the dump samples; the boot-time PHY calibration bakes
 * DC compensation DAC codes into every gain-table entry, but those values are
 * measured for the AGC's operating points and are far too small here.
 *
 * On characterized dump paths, a slow background feedback servo: the producer
 * feeds a few dump samples per emitted chunk, an exponential moving average
 * (fixed point, no FPU) estimates the residual DC as the ADC sees it, and
 * every few chunks the servo nudges the analog DC compensation DAC codes in
 * the forced gain-table entry toward zero. Proportional steps make clipping a
 * non-issue: a saturated estimate (+-511) simply produces maximum-size steps
 * until the signal is back in range, then the steps shrink with the error.
 * Conservative on purpose: it also tracks drift (temperature, LO leakage)
 * over a running session while the sub-second response still averages over
 * live traffic bursts.
 *
 * Two DAC pairs per entry (9-bit codes, neutral 256; layout from the open
 * PHY lib's phy_wr_rx_gain_mem_new): the baseband pair is the primary
 * actuator (fine granularity, measured ~2.6 ADC counts/code at gain 69 on
 * the S31 bench unit; dc_q moves the I rail and dc_i the Q rail, positive
 * sense — mapping and polarity match the C61); when a baseband code rails,
 * the servo spills into the RF-stage pair (bigger lever, ~26 counts/code,
 * direction learned from the response).
 *
 * Ported from the ESPARGOS sensor-firmware (ESP32-C61) implementation. The
 * S31 differences: feeds are per emitted chunk and throttled to a fixed
 * time base (see dcoc_feed), and gain-RAM writes need a special write
 * environment (see modem_dcoc_write_entry).
 */

#include "dcoc.h"

#include <stdlib.h>

#include "esp_timer.h"

#include "app_config.h"
#include "gaintable.h"
#include "modem.h"

#define DCOC_EMA_SHIFT 4        /* EMA time constant: 2^4 = 16 accepted feeds */
#define DCOC_UPDATE_FEEDS 8u    /* servo step cadence, in accepted feeds */
#define DCOC_DEADBAND_COUNTS 3  /* leave sub-LSB/quantization-scale DC alone */
#define DCOC_STEP_DIV 12        /* codes per step = error/12 (slope-safe) */
#define DCOC_STEP_MAX 8         /* max baseband codes per step */
#define DCOC_SPILL_CODES 2      /* RF-stage codes per spill step */
#define DCOC_FEED_MIN_INTERVAL_US 4000   /* servo time base, see dcoc_feed */

static struct {
    bool active;
    uint32_t slot;
    uint32_t w0, w1, w2;
    int32_t ema_i_q8, ema_q_q8;   /* DC estimate, ADC counts in Q8 */
    uint32_t feeds;
    int64_t last_feed_us;
    int32_t spill_dir_i, spill_dir_q;      /* learned RF-pair direction */
    int32_t spill_prev_i, spill_prev_q;    /* |error| at the last spill */
} s;

static volatile uint32_t s_diag;

/* entry field access: pair 0 = baseband (dc_i/dc_q), pair 1 = RF stage */
static uint32_t get_i(uint32_t pair)
{
    return pair != 0u ? (s.w1 >> 8) & 0x1ffu : ((s.w1 & 0xffu) << 1) | (s.w0 >> 31);
}

static uint32_t get_q(uint32_t pair)
{
    return pair != 0u ? (s.w0 >> 22) & 0x1ffu : (s.w0 >> 13) & 0x1ffu;
}

static void set_iq(uint32_t pair, int32_t dc_i, int32_t dc_q)
{
    dc_i = dc_i < 0 ? 0 : (dc_i > 511 ? 511 : dc_i);
    dc_q = dc_q < 0 ? 0 : (dc_q > 511 ? 511 : dc_q);
    if (pair != 0u) {
        s.w1 = (s.w1 & ~(0x1ffu << 8)) | ((uint32_t)dc_i << 8);
        s.w0 = (s.w0 & ~(0x1ffu << 22)) | ((uint32_t)dc_q << 22);
    } else {
        s.w1 = (s.w1 & ~0xffu) | ((uint32_t)dc_i >> 1);
        s.w0 = (s.w0 & 0x7fffffffu) | (((uint32_t)dc_i & 1u) << 31);
        s.w0 = (s.w0 & ~(0x1ffu << 13)) | ((uint32_t)dc_q << 13);
    }
}

void dcoc_arm(const modem_config_t *config)
{
    s.active = false;
    if (config->dc_offset_automatic == 0u ||
        config->rx_filter_override == 0u) {
        /* Production native IQ uses live PARLIO diagnostic lanes. Their
         * analog correction response is not monotonic across the fine-DAC
         * range on the S31 bench unit, so Soapy performs the requested safe
         * digital tracking instead. Keep analog actuation for characterized
         * dump/debug routes only. */
        s_diag = 0u;
        return;
    }
    if (config->loopback != 0u) {
        s_diag = 0u;
        return;   /* loopback has its own gain path and DCO cal */
    }
    if (config->gain_mode == GAIN_MODE_MANUAL) {
        gaintable_entry_t entry;
        if (config->rx_gain >= GAINTABLE_MAX_ENTRIES ||
            !gaintable_get_entry((uint8_t)config->rx_gain, &entry)) {
            /* No pristine entry for this slot (the generated low table has
             * fewer than 80 entries, ~70 on bench units); leave the servo
             * off rather than steer an unknown starting word. */
            s_diag = 0u;
            return;
        }
        s.slot = config->rx_gain;
        s.w0 = entry.word0;
        s.w1 = entry.word1;
        s.w2 = entry.word2;
    } else if (config->gain_mode == GAIN_MODE_EXPERT) {
        s.slot = EXPERT_GAIN_SLOT;
        s.w0 = config->expert_gain_word0;
        s.w1 = config->expert_gain_word1;
        s.w2 = config->expert_gain_word2;
    } else {
        s_diag = 0u;
        return;   /* AGC hops gain indices; a single-point servo is meaningless */
    }
    s.ema_i_q8 = 0;
    s.ema_q_q8 = 0;
    s.feeds = 0u;
    s.last_feed_us = 0;
    s.spill_dir_i = 1;
    s.spill_dir_q = 1;
    s.spill_prev_i = 0;
    s.spill_prev_q = 0;
    s.active = true;
}

void dcoc_disarm(void)
{
    s.active = false;
    s_diag = 0u;
}

bool dcoc_active(void)
{
    return s.active;
}

bool dcoc_feed_due(void)
{
    return s.active &&
        esp_timer_get_time() - s.last_feed_us >= DCOC_FEED_MIN_INTERVAL_US;
}

uint32_t dcoc_diag(void)
{
    return s_diag;
}

/* One rail of the servo: baseband step toward zero; on a baseband rail,
 * spill into the RF-stage code (direction learned: flip when the error grew
 * since the previous spill). `bb_is_q` selects which baseband/RF field acts
 * on this rail (dc_q/rf_q null the I rail, dc_i/rf_i the Q rail). */
static void servo_rail(int32_t err, bool bb_is_q, int32_t *spill_dir, int32_t *spill_prev)
{
    int32_t aerr = err < 0 ? -err : err;
    if (aerr <= DCOC_DEADBAND_COUNTS) {
        return;
    }
    int32_t step = err / DCOC_STEP_DIV;
    if (step == 0) {
        step = err > 0 ? 1 : -1;
    }
    if (step > DCOC_STEP_MAX) {
        step = DCOC_STEP_MAX;
    }
    if (step < -DCOC_STEP_MAX) {
        step = -DCOC_STEP_MAX;
    }
    int32_t bb = (int32_t)(bb_is_q ? get_q(0u) : get_i(0u)) - step;
    if (bb >= 0 && bb <= 511) {
        if (bb_is_q) {
            set_iq(0u, (int32_t)get_i(0u), bb);
        } else {
            set_iq(0u, bb, (int32_t)get_q(0u));
        }
        return;
    }
    /* baseband railed: hand the error to the RF-stage code */
    if (*spill_prev != 0 && aerr > *spill_prev) {
        *spill_dir = -*spill_dir;
    }
    *spill_prev = aerr;
    int32_t rf_step = (err > 0 ? -DCOC_SPILL_CODES : DCOC_SPILL_CODES) * *spill_dir;
    if (bb_is_q) {
        set_iq(1u, (int32_t)get_i(1u), (int32_t)get_q(1u) + rf_step);
    } else {
        set_iq(1u, (int32_t)get_i(1u) + rf_step, (int32_t)get_q(1u));
    }
}

void dcoc_feed(int32_t sum_i, int32_t sum_q)
{
    if (!s.active) {
        return;
    }
    /* The producer feeds every emitted chunk: ~30 Hz in slow interval modes
     * but ~15 kHz in live-copy streaming. Throttle to a fixed time base so
     * the EMA and step cadence remain independent of sample rate, and so the
     * gain-RAM write environment toggling stays rare. */
    int64_t now_us = esp_timer_get_time();
    if (now_us - s.last_feed_us < DCOC_FEED_MIN_INTERVAL_US) {
        return;
    }
    s.last_feed_us = now_us;
    /* Every producer contract supplies DCOC_SAMPLES_PER_FEED effective
     * signed 10-bit samples.  Bound the actuator input even if an
     * experimental source violates that contract; an estimator fault must
     * never drive the analog correction DACs to a rail. */
    const int32_t sum_limit = 512 * DCOC_SAMPLES_PER_FEED;
    sum_i = sum_i < -sum_limit ? -sum_limit
                               : (sum_i > sum_limit - 1 ? sum_limit - 1
                                                       : sum_i);
    sum_q = sum_q < -sum_limit ? -sum_limit
                               : (sum_q > sum_limit - 1 ? sum_limit - 1
                                                       : sum_q);
    /* Sample sum -> Q8 counts, into the EMA. */
    _Static_assert(DCOC_SAMPLES_PER_FEED <= 256 &&
                   (256 % DCOC_SAMPLES_PER_FEED) == 0,
                   "DCOC feed size must divide the Q8 scale");
    int32_t in_i = sum_i * (256 / DCOC_SAMPLES_PER_FEED);
    int32_t in_q = sum_q * (256 / DCOC_SAMPLES_PER_FEED);
    s.ema_i_q8 += (in_i - s.ema_i_q8) >> DCOC_EMA_SHIFT;
    s.ema_q_q8 += (in_q - s.ema_q_q8) >> DCOC_EMA_SHIFT;

    if (++s.feeds % DCOC_UPDATE_FEEDS != 0u) {
        return;
    }
    int32_t err_i = s.ema_i_q8 >> 8;
    int32_t err_q = s.ema_q_q8 >> 8;
    s_diag = (1u << 31) |
        (((uint32_t)err_i & 0x3fffu) << 14) | ((uint32_t)err_q & 0x3fffu);
    uint32_t before_w0 = s.w0, before_w1 = s.w1;
    servo_rail(err_i, true, &s.spill_dir_i, &s.spill_prev_i);
    servo_rail(err_q, false, &s.spill_dir_q, &s.spill_prev_q);
    if (s.w0 != before_w0 || s.w1 != before_w1) {
        modem_dcoc_write_entry(s.slot, s.w0, s.w1, s.w2);
    }
}
