#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "iq_network.h"
#include "ringbuffer.h"
#include "sdkconfig.h"

/* USB transport: one vendor interface on the S31's high-speed USB-OTG port.
 *
 * Endpoints:
 *   0x01 bulk OUT  control requests
 *   0x81 bulk IN   control responses
 *   0x82 bulk IN   IQ stream
 *   0x02 bulk OUT  packed IQ10 TX upload
 *
 * Control protocol: a 16-byte little-endian request header ("IQRQ", sequence,
 * opcode, payload_bytes) optionally followed by a JSON payload, answered by a
 * 16-byte response header ("IQRS", sequence, status, payload_bytes) plus JSON.
 * The JSON documents are identical to the HTTP API bodies.
 *
 * Stream framing: every stream frame (IQC1/CFG1/WPK1) is prefixed with the
 * same 52-byte IQU1 transport header as the Ethernet UDP path, but sent as a
 * single unfragmented message (fragment_count == 1). Messages are delimited
 * by the header's frame size; every message ends on a short USB packet.
 */
/* Detailed RX/USB/Ethernet/TX telemetry exceeds 2 KiB once counters have their
 * full decimal width. 2304 still leaves the rounded OUT allocation at its
 * existing 2560 bytes and grows only the IN response by 256 bytes. */
#define IQ_USB_CTRL_MAX_PAYLOAD 2304u

enum {
  IQ_USB_OP_GET_STATUS = 1u,
  IQ_USB_OP_GET_CONFIG = 2u,
  IQ_USB_OP_PUT_CONFIG = 3u,
  IQ_USB_OP_STREAM_START = 4u,
  IQ_USB_OP_STREAM_STOP = 5u,
  IQ_USB_OP_TX_ARM = 6u,
  IQ_USB_OP_TX_COMMIT = 7u,
  IQ_USB_OP_TX_ABORT = 8u,
};

/* TX_ARM carries this binary payload. The following bulk-OUT transfer(s) on
 * endpoint 0x02 contain exactly word_count little-endian packed IQ10 words.
 * TX_COMMIT atomically transfers ownership to the replay engine and may start
 * one-shot RF output according to commit_flags. */
typedef struct __attribute__((packed)) {
  uint32_t word_count;
  uint16_t commit_flags;
  uint16_t reserved;
  uint64_t start_time_ns;
} iq_usb_tx_arm_t;

_Static_assert(sizeof(iq_usb_tx_arm_t) == 16u, "USB TX arm wire size changed");

/* Native high-rate TX uses the same packed-IQ10 representation as Ethernet. */
#define IQ_TX_USB_FLAG_PACKED20 IQ_TX_UDP_FLAG_PACKED20

/* Stream-start payload field "stream_format" selects the IQ wire format. */
enum {
  IQ_USB_FORMAT_FULL = 0u, /* IQC1: 32-bit dump words */
  IQ_USB_FORMAT_INT8 = 1u, /* IQC8: interleaved int8 I/Q (8 MSBs of 10) */
  IQ_USB_FORMAT_INT4 = 2u, /* IQC4: packed signed 4-bit I/Q nibbles */
};

typedef struct __attribute__((packed)) {
  char magic[4]; /* "IQRQ" */
  uint32_t sequence;
  uint32_t opcode;
  uint32_t payload_bytes;
} iq_usb_ctrl_request_t;

typedef struct __attribute__((packed)) {
  char magic[4]; /* "IQRS" */
  uint32_t sequence;
  uint32_t status; /* 0 = success, nonzero = error (payload holds message) */
  uint32_t payload_bytes;
} iq_usb_ctrl_response_t;

#if CONFIG_ESP_SDR_TRANSPORT_USB

void iq_usb_init(const iq_network_callbacks_t *callbacks);
bool iq_usb_send_frame(const uint8_t *frame, size_t frame_bytes);
/* Zero-copy full-rate path used by the S31 staged backend. The returned frame
 * objects live immediately after their IQU1 headers in up to two contiguous
 * PSRAM DMA buffers. Commit hands the endpoint-sized batches to DWC2;
 * completion chains and releases them from the ISR. */
bool iq_usb_direct_reserve(uint32_t count, stream_frame_t **frames);
/* Compact counterparts for the production PARLIO RX paths. Frames are
 * written directly into the endpoint slab as IQC8 or IQC4, avoiding the
 * otherwise redundant PSRAM stream-ring copy on native USB. */
bool iq_usb_direct_reserve_iq8(uint32_t count, stream_frame_t **frames);
bool iq_usb_direct_reserve_iq4(uint32_t count, stream_frame_t **frames);
bool iq_usb_direct_commit(stream_frame_t **frames, uint32_t count);
void iq_usb_direct_abort(void);
/* Submit any partially filled frame batch. Call when the stream ring runs
 * empty so batching never adds latency at low rates. */
void iq_usb_flush(void);
uint32_t iq_usb_frames(void);
uint32_t iq_usb_send_errors(void);
uint32_t iq_usb_stream_format(void);
typedef struct {
  uint64_t send_cycles;
  uint64_t wait_cycles;
  uint64_t pack_cycles;
  uint64_t header_cycles;
  uint64_t cache_sync_cycles;
  uint64_t completion_cycles;
  uint32_t submissions;
  uint32_t submission_bytes;
  uint32_t completions;
  uint32_t completion_bytes;
  uint32_t maximum_completion_cycles;
} iq_usb_stream_diag_t;
void iq_usb_stream_diag(iq_usb_stream_diag_t *diag);
void iq_usb_tx_diag(uint32_t *uploads, uint32_t *bytes, uint32_t *errors,
                    uint32_t *backpressure_retries,
                    uint32_t *commit_rejections,
                    uint32_t *received_words, bool *active);
/* Native-USB TX uses a small fixed PSRAM pool so allocator operations never
 * run beside the cycle-exact RF emitter. Returns true for a pool-owned buffer
 * (which has been made available for a later host upload). */
bool iq_usb_tx_recycle(uint32_t *buffer);
void iq_usb_tx_pool_trim(void);
bool iq_usb_mounted(void);

#else

static inline void iq_usb_init(const iq_network_callbacks_t *callbacks) {
  (void)callbacks;
}

static inline bool iq_usb_send_frame(const uint8_t *frame, size_t frame_bytes) {
  (void)frame;
  (void)frame_bytes;
  return false;
}

static inline bool iq_usb_direct_reserve(uint32_t count,
                                         stream_frame_t **frames) {
  (void)count;
  (void)frames;
  return false;
}
static inline bool iq_usb_direct_reserve_iq8(uint32_t count,
                                             stream_frame_t **frames) {
  (void)count;
  (void)frames;
  return false;
}
static inline bool iq_usb_direct_reserve_iq4(uint32_t count,
                                             stream_frame_t **frames) {
  (void)count;
  (void)frames;
  return false;
}
static inline bool iq_usb_direct_commit(stream_frame_t **frames,
                                        uint32_t count) {
  (void)frames;
  (void)count;
  return false;
}
static inline void iq_usb_direct_abort(void) {}

static inline void iq_usb_flush(void) {}
static inline uint32_t iq_usb_frames(void) { return 0u; }
static inline uint32_t iq_usb_send_errors(void) { return 0u; }
static inline uint32_t iq_usb_stream_format(void) { return IQ_USB_FORMAT_FULL; }
typedef struct {
  uint64_t send_cycles;
  uint64_t wait_cycles;
  uint64_t pack_cycles;
  uint64_t header_cycles;
  uint64_t cache_sync_cycles;
  uint64_t completion_cycles;
  uint32_t submissions;
  uint32_t submission_bytes;
  uint32_t completions;
  uint32_t completion_bytes;
  uint32_t maximum_completion_cycles;
} iq_usb_stream_diag_t;
static inline void iq_usb_stream_diag(iq_usb_stream_diag_t *diag) {
  *diag = (iq_usb_stream_diag_t){0};
}
static inline void iq_usb_tx_diag(uint32_t *uploads, uint32_t *bytes,
                                  uint32_t *errors,
                                  uint32_t *backpressure_retries,
                                  uint32_t *commit_rejections,
                                  uint32_t *received_words, bool *active) {
  *uploads = 0u;
  *bytes = 0u;
  *errors = 0u;
  *backpressure_retries = 0u;
  *commit_rejections = 0u;
  *received_words = 0u;
  *active = false;
}
static inline bool iq_usb_tx_recycle(uint32_t *buffer) {
  (void)buffer;
  return false;
}
static inline void iq_usb_tx_pool_trim(void) {}
static inline bool iq_usb_mounted(void) { return false; }

#endif
