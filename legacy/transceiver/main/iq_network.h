#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"

#define IQ_NETWORK_HOSTNAME "esp-sdr"
#define IQ_NETWORK_HTTP_PORT 80u
#define IQ_NETWORK_UDP_DEFAULT_PORT 50000u
#define IQ_NETWORK_TX_UDP_PORT 50001u
#define IQ_RX_BASE_SAMPLE_RATE_HZ 16000000u
#define IQ_UDP_MAGIC "IQU1"
#define IQ_UDP_VERSION 1u
#define IQ_UDP_FRAGMENT_PAYLOAD_BYTES 1400u

typedef struct __attribute__((packed)) {
  char magic[4];
  uint16_t version;
  uint16_t header_bytes;
  uint32_t stream_epoch;
  uint32_t datagram_sequence;
  uint32_t frame_sequence;
  uint32_t source_chunk_index;
  uint32_t frame_bytes;
  uint32_t frame_crc32;
  uint32_t fragment_offset;
  uint16_t fragment_bytes;
  uint8_t fragment_index;
  uint8_t fragment_count;
  char frame_magic[4];
  uint32_t firmware_dropped_chunks;
  uint32_t header_crc32;
} iq_udp_header_t;

_Static_assert(sizeof(iq_udp_header_t) == 52u,
               "IQ UDP header wire size changed");

#define IQ_TX_UDP_MAGIC "IQT1"
#define IQ_TX_UDP_VERSION 1u
#define IQ_TX_UDP_FLAG_RESET (1u << 0)
#define IQ_TX_UDP_FLAG_COMMIT (1u << 1)
#define IQ_TX_UDP_FLAG_ACK_REQUEST (1u << 2)
/* One signed I byte followed by one signed Q byte per sample. Firmware
 * restores the modem's IQ10 layout by multiplying each component by four. */
#define IQ_TX_UDP_FLAG_PACKED16 (1u << 3)
/* A token-gated final packet may atomically request one-shot RF playback.
 * The four-bit raw modem divider code occupies flags 9..12. */
#define IQ_TX_UDP_FLAG_AUTOSTART (1u << 8)
#define IQ_TX_UDP_RATE_CODE_S 9u
#define IQ_TX_UDP_RATE_CODE_M (0x0fu << IQ_TX_UDP_RATE_CODE_S)
/* The committed batch is followed by another batch in the same continuous
 * TXDC stream. The final batch omits MORE and terminates RF cleanly. */
#define IQ_TX_UDP_FLAG_MORE (1u << 13)
/* This batch belongs to an already active TXDC chain. Unlike MORE (which
 * describes what follows the current batch), CONTINUE describes what came
 * before it and lets firmware reject a commit after that chain timed out. */
#define IQ_TX_UDP_FLAG_CONTINUE (1u << 14)
/* Two little-endian IQ10 samples occupy five payload bytes. Packet boundaries
 * restart the packing, so a final odd sample occupies three bytes. */
#define IQ_TX_UDP_FLAG_PACKED20 (1u << 15)
#define IQ_TX_UDP_FLAGS_PACKED                                              \
  (IQ_TX_UDP_FLAG_PACKED16 | IQ_TX_UDP_FLAG_PACKED20)
#define IQ_TX_UDP_FLAGS_ALLOWED                                              \
  (IQ_TX_UDP_FLAG_RESET | IQ_TX_UDP_FLAG_COMMIT | IQ_TX_UDP_FLAG_ACK_REQUEST | \
   IQ_TX_UDP_FLAG_PACKED16 | IQ_TX_UDP_FLAG_AUTOSTART |                       \
   IQ_TX_UDP_RATE_CODE_M | IQ_TX_UDP_FLAG_MORE | IQ_TX_UDP_FLAG_CONTINUE |    \
   IQ_TX_UDP_FLAG_PACKED20)
#define IQ_TX_UDP_ACK_MAGIC "IQA1"

typedef struct __attribute__((packed)) {
  char magic[4];
  uint16_t version;
  uint16_t header_bytes;
  uint32_t session_token;
  uint32_t batch_id;
  uint32_t datagram_sequence;
  uint32_t sample_offset;
  uint16_t sample_count;
  uint16_t flags;
  uint32_t payload_crc32;
  uint32_t header_crc32;
} iq_tx_udp_header_t;

_Static_assert(sizeof(iq_tx_udp_header_t) == 36u,
               "TX UDP header wire size changed");

typedef struct __attribute__((packed)) {
  char magic[4];
  uint16_t version;
  uint16_t header_bytes;
  uint32_t session_token;
  uint32_t batch_id;
  uint32_t next_sequence;
  uint32_t received_words;
  uint32_t committed_words;
  uint32_t error_count;
  uint32_t commit_count;
  uint32_t header_crc32;
} iq_tx_udp_ack_t;

_Static_assert(sizeof(iq_tx_udp_ack_t) == 40u, "TX UDP ACK wire size changed");

typedef void (*iq_network_get_config_fn)(capture_config_t *config);
typedef void (*iq_network_apply_config_fn)(const capture_config_t *config);
typedef uint32_t (*iq_network_get_u32_fn)(void);
typedef bool (*iq_network_get_bool_fn)(void);
typedef void (*iq_network_set_bool_fn)(bool value);
typedef bool (*iq_network_set_tx_waveform_fn)(const uint32_t *words,
                                              uint32_t word_count);
typedef enum {
  IQ_TX_WAVEFORM_REJECTED = 0,
  IQ_TX_WAVEFORM_ADOPTED,
  IQ_TX_WAVEFORM_RETRY,
} iq_tx_waveform_result_t;
typedef iq_tx_waveform_result_t (*iq_network_take_tx_waveform_fn)(
    uint32_t *words, uint32_t word_count, uint16_t commit_flags,
    uint64_t start_time_ns);
typedef void (*iq_network_reclaim_tx_waveforms_fn)(void);

typedef struct {
  uint32_t words;
  uint32_t rate_code;
  uint32_t segments;
  uint32_t total_cycles;
  uint32_t sample_cycles;
  uint32_t gap_cycles;
  uint32_t maximum_gap_cycles;
  uint32_t tcm_stage_copy_max_cycles;
  uint32_t live_probe_write_cycles;
  uint32_t live_probe_match_words;
  uint32_t live_probe_dma_status;
  uint32_t deadline_late_max_cycles;
  uint32_t deadline_late_max_word;
  uint32_t pbus_force_start;
  uint32_t pbus_force_end;
  uint32_t pa_close_start;
  uint32_t pa_close_end;
  uint32_t fe_clocks_start;
  uint32_t fe_clocks_end;
  uint32_t pwdet_start;
  uint32_t pwdet_end;
  uint32_t fe_debug_start;
  uint32_t fe_debug_end;
  uint32_t mac_status_start;
  uint32_t mac_status_end;
  uint32_t pbus_read_start;
  uint32_t pbus_read_end;
  bool live_probe_dma_ok;
  uint64_t requested_start_time_ns;
  uint64_t actual_start_time_ns;
  int64_t start_error_ns;
  bool queue_underflow;
  bool deadline_missed;
} iq_tx_replay_diag_t;
typedef void (*iq_network_get_tx_replay_diag_fn)(iq_tx_replay_diag_t *diag);

typedef struct {
  iq_network_get_config_fn get_config;
  iq_network_apply_config_fn apply_config;
  iq_network_get_u32_fn get_firmware_dropped_chunks;
  iq_network_get_u32_fn get_source_chunk_index;
  /* Live ADC dump registers, exposed for selector/routing diagnostics. */
  iq_network_get_u32_fn get_adc_dump_cfg;
  iq_network_get_u32_fn get_adc_dump_mode;
  iq_network_get_u32_fn get_modem_diag_fix_sel;
  iq_network_get_u32_fn get_modem_diag_exchange;
  iq_network_get_u32_fn get_bb_diag;
  /* DCOC servo state word: bit 31 = active, bits 14-27 / 0-13 = last I/Q DC
   * estimate (signed 14-bit, ADC counts). */
  iq_network_get_u32_fn get_dcoc_diag;
  iq_network_get_bool_fn get_dcoc_active;
  iq_network_get_bool_fn is_config_applying;
  iq_network_set_bool_fn set_capture_armed;
  /* Store packed signed IQ10 words for a later cyclic TX replay start. */
  iq_network_set_tx_waveform_fn set_tx_waveform;
  /* Adopt a PSRAM allocation without copying it. ADOPTED transfers ownership
   * to the replay engine, RETRY retains transport ownership for backpressure,
   * and REJECTED terminates the request. commit_flags may also atomically
   * queue authenticated one-shot playback, preventing an upload/start race.
   * start_time_ns is an absolute esp_timer-domain hardware time; zero requests
   * immediate start. */
  iq_network_take_tx_waveform_fn take_tx_waveform;
  /* Free TXDC batches retired by the realtime core before a transport tries
   * to reserve another maximum-size PSRAM upload. */
  iq_network_reclaim_tx_waveforms_fn reclaim_tx_waveforms;
  /* Timing of the most recently completed replay request. Cycle counts use
   * the 320 MHz CPU clock and distinguish RF-active time from TCM refills. */
  iq_network_get_tx_replay_diag_fn get_tx_replay_diag;
} iq_network_callbacks_t;

/* Initialize the control plane and shared stream-owner state without starting
 * a physical transport.  USB-only builds need this even though they do not
 * install the Ethernet driver. */
void iq_control_init(const iq_network_callbacks_t *callbacks);
void iq_network_init(const iq_network_callbacks_t *callbacks);
/* Native USB owns the realtime data plane while a USB TX chain is active.
 * Quiesce GMAC interrupts/traffic for that interval, then restore Ethernet;
 * control remains available over USB and Ethernet reacquires DHCP afterward. */
void iq_network_suspend_for_usb_tx(void);
/* Once GMAC is stopped, its internal DMA buffers are idle. Native USB TX may
 * borrow them as a scatter-gather realtime staging ring until resume. This
 * relies on the pinned ESP-IDF EMAC layout mirrored in iq_network.c; starting
 * GMAC resets the descriptor ownership before Ethernet traffic resumes. */
uint32_t iq_network_usb_tx_stage_buffers(uint32_t **buffers,
                                         uint32_t max_buffers,
                                         uint32_t *words_per_buffer);
void iq_network_resume_after_usb_tx(void);
bool iq_network_send_frame(const uint8_t *frame, size_t frame_bytes);
bool iq_network_send_frames(const uint8_t *const *frames,
                            const size_t *frame_bytes, uint32_t frame_count);
/* Quantize an IQC1 frame to IQC8 in the normal transport context, then send
 * it. This keeps S31 capture transactions on their proven full-word path. */
bool iq_network_send_frame_int8(const uint8_t *frame, size_t frame_bytes);
bool iq_network_stream_armed(void);

/* The sample stream has one owner at a time; a start on either transport
 * replaces the current stream (new start wins). Control stays available on
 * both transports throughout. */
typedef enum {
  IQ_STREAM_OWNER_NONE = 0,
  IQ_STREAM_OWNER_ETH = 1,
  IQ_STREAM_OWNER_USB = 2,
} iq_stream_owner_t;

iq_stream_owner_t iq_network_stream_owner(void);
/* Wire format selected by the active stream-start request.  Values use the
 * IQ_USB_FORMAT_* enum because USB and Ethernet share IQC1/IQC8 framing. */
void iq_network_stream_set_format(uint32_t format);
uint32_t iq_network_stream_format(void);
/* Claim the stream: bumps the epoch and resets the datagram sequence. */
void iq_network_stream_begin(iq_stream_owner_t owner);
/* Arm capture and start the delayed stream-armed timer. */
void iq_network_stream_arm(void);
/* Stop streaming and release ownership (any owner). */
void iq_network_stream_end(void);
/* Fetch the epoch and allocate the next datagram sequence number for one
 * outgoing message. Fails unless armed and owned by `owner`. */
bool iq_network_stream_tx_ticket(iq_stream_owner_t owner, uint32_t *epoch,
                                 uint32_t *sequence);

/* Shared control-plane helpers (JSON bodies identical on HTTP and USB). */
struct cJSON;
int iq_control_build_config_json(char *buf, size_t cap);
int iq_control_build_status_json(char *buf, size_t cap);
/* Merge a parsed JSON patch onto the current configuration and validate.
 * Returns NULL on success (with *config filled in), else an error string. */
const char *iq_control_parse_config_json(struct cJSON *root,
                                         capture_config_t *config);
