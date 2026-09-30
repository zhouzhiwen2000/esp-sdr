#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "soc/soc.h"

static inline uint32_t reg32_read_addr(uint32_t reg)
{
    return REG_READ(reg);
}

static inline void reg32_write_addr(uint32_t reg, uint32_t value)
{
    REG_WRITE(reg, value);
}

static inline void reg32_set_bits_addr(uint32_t reg, uint32_t mask)
{
    REG_WRITE(reg, REG_READ(reg) | mask);
}

static inline void reg32_clear_bits_addr(uint32_t reg, uint32_t mask)
{
    REG_WRITE(reg, REG_READ(reg) & ~mask);
}

static inline void reg32_write_bits_addr(uint32_t reg, uint32_t mask, uint32_t value)
{
    REG_WRITE(reg, (REG_READ(reg) & ~mask) | (value & mask));
}

static inline bool reg32_read_bit_addr(uint32_t reg, uint32_t mask)
{
    return (REG_READ(reg) & mask) != 0u;
}

static inline void reg32_write_bit_addr(uint32_t reg, uint32_t mask, bool enabled)
{
    if (enabled) {
        reg32_set_bits_addr(reg, mask);
    } else {
        reg32_clear_bits_addr(reg, mask);
    }
}

static inline uint32_t reg32_read_field_addr(uint32_t reg, uint32_t mask, uint32_t shift)
{
    return (REG_READ(reg) & mask) >> shift;
}

static inline void reg32_write_field_addr(uint32_t reg, uint32_t mask, uint32_t shift,
                                          uint32_t value)
{
    REG_WRITE(reg, (REG_READ(reg) & ~mask) | ((value << shift) & mask));
}

static inline uint32_t reg32p_read(volatile uint32_t *reg)
{
    return *reg;
}

static inline void reg32p_write(volatile uint32_t *reg, uint32_t value)
{
    *reg = value;
}

static inline void reg32p_set_bits(volatile uint32_t *reg, uint32_t mask)
{
    *reg |= mask;
}

static inline void reg32p_clear_bits(volatile uint32_t *reg, uint32_t mask)
{
    *reg &= ~mask;
}

static inline void reg32p_write_bits(volatile uint32_t *reg, uint32_t mask, uint32_t value)
{
    *reg = (*reg & ~mask) | (value & mask);
}

static inline bool reg32p_read_bit(volatile uint32_t *reg, uint32_t mask)
{
    return (*reg & mask) != 0u;
}

static inline void reg32p_write_bit(volatile uint32_t *reg, uint32_t mask, bool enabled)
{
    if (enabled) {
        reg32p_set_bits(reg, mask);
    } else {
        reg32p_clear_bits(reg, mask);
    }
}

static inline uint32_t reg32p_read_field(volatile uint32_t *reg, uint32_t mask, uint32_t shift)
{
    return (*reg & mask) >> shift;
}

static inline void reg32p_write_field(volatile uint32_t *reg, uint32_t mask, uint32_t shift,
                                      uint32_t value)
{
    *reg = (*reg & ~mask) | ((value << shift) & mask);
}
