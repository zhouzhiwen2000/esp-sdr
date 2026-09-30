#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "wifi_tx_rx.h"

#define STREAM_FRAME_MAGIC_IQ "IQC1"
/* Compressed variant: same 52-byte header, interleaved int8 I/Q pairs
 * holding the top 8 of 10 sample bits, zero CRC field. */
#define STREAM_FRAME_MAGIC_IQ8 "IQC8"
/* High-rate native-USB variant: one byte per complex sample, with signed
 * four-bit I in the low nibble and signed four-bit Q in the high nibble. */
#define STREAM_FRAME_MAGIC_IQ4 "IQC4"
/* Signed real int8 samples. Each frame carries 2048 consecutive real samples;
 * the host forms the analytic signal and decimates by two. */
#define STREAM_FRAME_MAGIC_REAL8 "IQR8"
#define STREAM_FRAME_MAGIC_CONFIG "CFG1"
/* IQ header flags. Low four bits retain the modem AGC state. The remaining
 * diagnostic fields describe the firmware software AGC at the instant the
 * frame was prepared. Counters saturate on the wire; full values remain
 * available through /status. When timestamp-valid is set, chunk_counter
 * carries the low 32 bits of a monotonic microsecond timestamp. Hosts unwrap
 * it across the roughly 71-minute rollover. */
#define STREAM_FRAME_FLAG_TIMESTAMP_US32 (1u << 31)
#define STREAM_FRAME_FLAG_SOFTWARE_AGC_ACTIVE (1u << 30)
#define STREAM_FRAME_FLAG_DCOC_ACTIVE (1u << 29)
#define STREAM_FRAME_DCOC_ERROR_I_S 4u
#define STREAM_FRAME_DCOC_ERROR_Q_S 16u
#define STREAM_FRAME_DCOC_ERROR_M 0xfffu
#define STREAM_FRAME_AGC_ROBUST_PEAK_S 20u
#define STREAM_FRAME_AGC_ROBUST_PEAK_M \
    (0x1ffu << STREAM_FRAME_AGC_ROBUST_PEAK_S)
#define STREAM_FRAME_AGC_GAIN_CHANGES_S 4u
#define STREAM_FRAME_AGC_GAIN_CHANGES_M \
    (0xffffu << STREAM_FRAME_AGC_GAIN_CHANGES_S)
#define STREAM_FRAME_AGC_STATE_MASK 0x0fu
#define IQ8_FRAME_WIRE_BYTES (52u + 2u * IQ_CHUNK_SAMPLE_WORDS + 4u)
#define IQ4_FRAME_WIRE_BYTES (52u + IQ_CHUNK_SAMPLE_WORDS + 4u)
/* Real mode uses the complete 4 KiB sample area. Three near-MTU datagrams per
 * frame are substantially cheaper than two half-filled datagrams per 2 KiB
 * frame at 40 MSa/s. */
#define REAL8_FRAME_WIRE_BYTES (52u + 4u * IQ_CHUNK_SAMPLE_WORDS + 4u)

typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t sequence;
    uint32_t source_chunk_index;
    uint32_t chunk_counter; /* timestamp_us32 when flags bit 31 is set */
    uint32_t adc_decimation;
    uint32_t sample_rate_hz;
    uint32_t center_freq_mhz;
    uint32_t rx_gain;
    uint32_t flags;
    uint32_t dropped_chunks;
    uint32_t bank_timer_late_misses;
    uint32_t bank_timer_write_ptr;
    uint32_t producer_wake_write_ptr;
    uint32_t samples[IQ_CHUNK_SAMPLE_WORDS];
    uint32_t crc32;
} iq_chunk_t;

typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t sequence;
    uint32_t uptime_ms;
    uint32_t stage;
    uint32_t rx_gain;
    uint32_t loopback;
    uint32_t tx_tone_enable;
    uint32_t adc_source_sel;
    uint32_t expert_gain_word0;
    uint32_t expert_gain_word1;
    uint32_t ctrl;
    uint32_t mode;
    uint32_t hp_tcm_dump_ctrl;
    uint32_t trigger_interval_chunks;
    uint32_t trigger_duration_chunks;
    uint32_t crc32;
} config_report_t;

typedef union {
    iq_chunk_t iq;
    wifi_packet_report_t wifi;
    config_report_t config;
} stream_frame_t;

typedef struct {
    uint32_t source_chunk_index;
    uint32_t adc_decimation;
    uint32_t sample_rate_hz;
    uint32_t center_freq_mhz;
    uint32_t rx_gain;
    uint32_t agc_state;
    bool software_agc_active;
    uint32_t agc_robust_peak;
    uint32_t agc_gain_changes;
    uint32_t dropped_chunks;
    uint32_t bank_timer_late_misses;
    uint32_t bank_timer_write_ptr;
    uint32_t producer_wake_write_ptr;
} iq_chunk_report_meta_t;

void stream_ring_init_storage(void);
void stream_ring_reset(void);
/* Block up to timeout_ms until a producer commits a frame. Wake tokens are
 * a hint: callers must still check stream_ring_peek(). */
bool stream_ring_wait_frames(uint32_t timeout_ms);
bool stream_ring_reserve_slots(uint32_t count, stream_frame_t **slots);
void stream_ring_commit_reserved(uint32_t count);
void stream_ring_release_reserved(uint32_t count);
uint32_t stream_ring_available_slots(void);
bool stream_ring_push_wifi(const wifi_packet_report_t *report);
bool stream_ring_push_config_report(const config_report_t *report);
void stream_ring_fill_iq_chunk(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *source_words);
/* Prepare only the IQC1 header.  Capture backends that move samples while the
 * S31 TCM fabric is switched can fill the PSRAM sample array separately, then
 * finalize the ordinary wire header once normal TCM access is restored. */
void stream_ring_prepare_iq_chunk(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta);
void stream_ring_fill_iq_chunk_strided(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *ring_words, uint32_t start_word,
    uint32_t stride_words, uint32_t ring_word_count);
/* IQC8 variants: pack the top 8 of 10 sample bits at the source, halving
 * producer store traffic and the downstream wire rate. */
void stream_ring_fill_iq_chunk_int8(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *source_words);
void stream_ring_fill_iq_chunk_strided_int8(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *ring_words, uint32_t start_word,
    uint32_t stride_words, uint32_t ring_word_count);
/* Bluetooth adctrig mode 12 places its candidate sample bytes directly in
 * dump-word bytes 0 and 1, rather than in the Wi-Fi 10-bit fields. */
void stream_ring_fill_bt_bytes_chunk_int8(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *source_words);
void stream_ring_fill_bt_bytes_chunk_strided_int8(
    stream_frame_t *frame, const iq_chunk_report_meta_t *meta,
    const volatile uint32_t *ring_words, uint32_t start_word,
    uint32_t stride_words, uint32_t ring_word_count);
stream_frame_t *stream_ring_peek(void);
/* Snapshot up to max_count committed frames in read order. The returned
 * slots remain owned by the consumer until the matching pop operation. */
uint32_t stream_ring_peek_batch(stream_frame_t **frames, uint32_t max_count);
void stream_ring_pop(void);
void stream_ring_pop_batch(uint32_t count);
size_t stream_frame_wire_size(stream_frame_t *frame);
