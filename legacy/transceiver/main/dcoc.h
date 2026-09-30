#pragma once

/*
 * Continuous background DC offset compensation (DCOC) for characterized
 * forced-gain dump paths. Production PARLIO uses safe host-side correction
 * because its analog actuator response is not monotonic. See dcoc.c.
 */

#include <stdbool.h>
#include <stdint.h>

#include "modem.h"

/* A rotating, low-discrepancy sample of each selected output chunk.  The
 * producer deliberately uses strides coprime to the 1024-sample chunk so
 * periodic signals cannot pin every tap to one waveform phase. */
#define DCOC_SAMPLES_PER_FEED 32

/* (Re-)arm the servo for the forced gain of `config` (manual: from the
 * pristine gain-table snapshot entry; expert: from the config's raw words).
 * Disarms in AGC and loopback modes. Call after every gain / frequency /
 * engine change. */
void dcoc_arm(const modem_config_t *config);
void dcoc_disarm(void);
bool dcoc_active(void);

/* Check this before collecting the taps.  dcoc_feed() rechecks the time so
 * callers remain correct if some work occurs between the two calls. */
bool dcoc_feed_due(void);

/* Producer hook: sums of DCOC_SAMPLES_PER_FEED sign-extended I/Q samples. */
void dcoc_feed(int32_t sum_i, int32_t sum_q);

/* Servo state for the status endpoint: bit 31 = active, bits 14-27 / 0-13 =
 * last I/Q DC estimate (signed 14-bit, ADC counts). */
uint32_t dcoc_diag(void);
