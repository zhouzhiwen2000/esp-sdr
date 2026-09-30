#include "gaintable.h"

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

static portMUX_TYPE s_gaintable_lock = portMUX_INITIALIZER_UNLOCKED;
static struct {
    bool valid;
    uint8_t entry_count;
    gaintable_entry_t entries[GAINTABLE_MAX_ENTRIES];
} s_gain_table;
static uint8_t s_published_entry_count;
static volatile bool s_inside_rx_gain_table;
static volatile bool s_reset_pending;

void __real_phy_set_rx_gain_table(uint32_t arg0, uint32_t arg1);
void __real_phy_write_gain_mem(uint32_t word0, uint32_t word1, uint32_t word2,
                               uint32_t index);

void __wrap_phy_set_rx_gain_table(uint32_t arg0, uint32_t arg1)
{
    /* Reset the snapshot LAZILY, on the first captured write: the real
     * function only rewrites the gain RAM when cal_state_flags bit 0x200 is
     * clear, so it is frequently a no-op. When the rewrite is skipped, the
     * gain RAM is unchanged and the existing snapshot is still accurate —
     * keep it. */
    s_reset_pending = true;
    s_inside_rx_gain_table = true;
    __real_phy_set_rx_gain_table(arg0, arg1);
    s_inside_rx_gain_table = false;
    /* Publish the range only after the writer has completed, so network
     * readers never observe the entry count growing through a partial table. */
    portENTER_CRITICAL(&s_gaintable_lock);
    if (!s_reset_pending && s_gain_table.valid) {
        s_published_entry_count = s_gain_table.entry_count;
    }
    portEXIT_CRITICAL(&s_gaintable_lock);
    s_reset_pending = false;
}

void __wrap_phy_write_gain_mem(uint32_t word0, uint32_t word1, uint32_t word2,
                               uint32_t index)
{
    /* Capture only the table-generation writes: the DCOC servo and the
     * expert slots also write gain RAM entries, and those edits must not
     * pollute the pristine snapshot. */
    if (s_inside_rx_gain_table && index < GAINTABLE_MAX_ENTRIES) {
        portENTER_CRITICAL(&s_gaintable_lock);
        if (s_reset_pending) {
            s_gain_table.valid = false;
            s_gain_table.entry_count = 0u;
            s_reset_pending = false;
        }
        s_gain_table.valid = true;
        s_gain_table.entries[index].word0 = word0;
        s_gain_table.entries[index].word1 = word1;
        s_gain_table.entries[index].word2 = word2;
        if (s_gain_table.entry_count <= index) {
            s_gain_table.entry_count = (uint8_t)(index + 1u);
        }
        portEXIT_CRITICAL(&s_gaintable_lock);
    }
    __real_phy_write_gain_mem(word0, word1, word2, index);
}

bool gaintable_get_entry(uint8_t index, gaintable_entry_t *out)
{
    if (out == NULL || index >= GAINTABLE_MAX_ENTRIES) {
        return false;
    }

    portENTER_CRITICAL(&s_gaintable_lock);
    const bool valid = s_gain_table.valid && index < s_gain_table.entry_count;
    if (valid) {
        *out = s_gain_table.entries[index];
    }
    portEXIT_CRITICAL(&s_gaintable_lock);

    return valid;
}

uint8_t gaintable_entry_count(void)
{
    portENTER_CRITICAL(&s_gaintable_lock);
    const uint8_t entry_count = s_published_entry_count;
    portEXIT_CRITICAL(&s_gaintable_lock);

    return entry_count;
}
