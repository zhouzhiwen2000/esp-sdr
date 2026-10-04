#pragma once
#include <stdint.h>

/* Address-only marker, distinct from the normal PTP timestamp pointer. */
extern const uint32_t s31_emac_direct_token;
extern const uint32_t s31_emac_frame_token;
extern const uint32_t s31_emac_drain_token;
