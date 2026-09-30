#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Pristine snapshot of the boot-calibrated RX gain table (low table, slots
 * 0-79), captured by wrapping the open PHY lib's table writers at link time
 * (-Wl,--wrap=phy_set_rx_gain_table / phy_write_gain_mem). The DCOC servo
 * needs each forced slot's unmodified calibration entry as its starting
 * point, and the second-table mirror needs the entry words verbatim; the
 * gain RAM itself is write-only from this side.
 */

#define GAINTABLE_MAX_ENTRIES 80

typedef struct {
    uint32_t word0;
    uint32_t word1;
    uint32_t word2;
} gaintable_entry_t;

bool gaintable_get_entry(uint8_t index, gaintable_entry_t *out);
uint8_t gaintable_entry_count(void);
