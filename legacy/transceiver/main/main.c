/* Continuous IQ capture firmware for the ESP32-S31 mac-dump engine. */
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/parlio_rx.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_clk_tree.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_freertos_hooks.h"
#include "esp_heap_caps.h"
#include "esp_ipc_isr.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_netif.h"
#include "esp_phy_cert_test.h"
#include "esp_private/esp_clk.h"
#include "esp_private/esp_gpio_reserve.h"
#include "esp_private/gdma.h"
#include "esp_private/gdma_link.h"
#include "esp_rom_crc.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/axi_dma_ll.h"
#include "hal/mwdt_ll.h"
#include "hal/parlio_ll.h"
#include "heap_memory_layout.h"
#include "modem/modem_widgets_reg.h"
#include "modem/reg_base.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "soc/ahb_dma_struct.h"
#include "soc/axi_dma_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/hp_system_reg.h"
#include "soc/lp_system_reg.h"
#include "soc/soc.h"
#include "soc/timer_group_struct.h"
#if CONFIG_IDF_TARGET_ESP32S31 && !CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF
#include "s31_tcm_probe.h"
#include "ulp_lp_core.h"
#endif

#include "app_config.h"
#include "dcoc.h"
#include "iq_network.h"
#include "iq_usb.h"
#include "modem.h"
#include "phy_pbus_reg_names.h"
#include "phy_regs.h"
#include "reg_helpers.h"
#include "ringbuffer.h"
#include "sdr_agc.h"
#include "wifi_tx_rx.h"

#if CONFIG_IDF_TARGET_ESP32S31
extern void phy_pbus_force_test(uint32_t block, uint32_t bank,
                                uint32_t value);
extern uint32_t phy_pbus_rd(uint32_t block, uint32_t bank);
/*
 * S31 rftest's adctrig path and sentinel probes show the ADC dump engine writes
 * a fixed TCM aperture starting at 0x2f060000 in this mode. Reserving from
 * 0x2f050000 does not enlarge the ring; it keeps heap allocations out of a
 * guard area below the engine aperture so timing slips or misunderstood gate
 * effects are less likely to collide with CPU-owned objects.
 */
#define S31_MAC_DUMP_SRAM_GUARD_START 0x2f050000u
#define S31_ROM_RESERVED_TOP_START 0x2f07afb0u
#define S31_ROM_RESERVED_TOP_END 0x2f07f170u
#define S31_MAC_DUMP_SRAM_SAFE_END S31_ROM_RESERVED_TOP_START
SOC_RESERVE_MEMORY_REGION(S31_MAC_DUMP_SRAM_GUARD_START,
                          S31_MAC_DUMP_SRAM_SAFE_END, rf_adc_dump_window);
SOC_RESERVE_MEMORY_REGION(S31_ROM_RESERVED_TOP_START, S31_ROM_RESERVED_TOP_END,
                          s31_rom_reserved_top);
#else
SOC_RESERVE_MEMORY_REGION(MAC_DUMP_SRAM_BANK2_BASE, MAC_DUMP_SRAM_BANK2_END,
                          rf_adc_dump_bank2_window);
SOC_RESERVE_MEMORY_REGION(MAC_DUMP_SRAM_BANK3_BASE, MAC_DUMP_SRAM_BANK3_END,
                          rf_adc_dump_bank3_window);
#endif

/* Required by a linked component; unused here. */
int cmd_parse(char *cmd, char *name, int *argc, char **argv) {
  (void)cmd;
  (void)name;
  (void)argc;
  (void)argv;
  return -1;
}

/* Firmware policy: which mac-dump SRAM banks this firmware uses (per-bank
 * hardware facts live in phy_regs.h). BANK2+BANK3 as a contiguous window. */
#if CONFIG_IDF_TARGET_ESP32S31
#define MAC_DUMP_SRAM_USAGE MAC_DUMP_SRAM_BANK0_USAGE
#define MAC_DUMP_SRAM_BASE MAC_DUMP_SRAM_BANK0_BASE
#define MAC_DUMP_SRAM_WINDOW_BYTES                                             \
  (S31_MAC_DUMP_SRAM_SAFE_END - MAC_DUMP_SRAM_BANK0_BASE)
#else
#define MAC_DUMP_SRAM_USAGE                                                    \
  (MAC_DUMP_SRAM_BANK2_USAGE | MAC_DUMP_SRAM_BANK3_USAGE)
#define MAC_DUMP_SRAM_BASE MAC_DUMP_SRAM_BANK2_BASE
#define MAC_DUMP_SRAM_WINDOW_BYTES (2u * MAC_DUMP_SRAM_BANK_BYTES)
#endif

#define MAX_DUMP_WORDS (MAC_DUMP_SRAM_WINDOW_BYTES / sizeof(uint32_t))
#if CONFIG_IDF_TARGET_ESP32S31
#define DUMP_BANK_WORDS                                                        \
  ((MAX_DUMP_WORDS / IQ_CHUNK_SAMPLE_WORDS) * IQ_CHUNK_SAMPLE_WORDS)
#else
#define DUMP_BANK_WORDS MAC_DUMP_SRAM_BANK_WORDS /* 16384 */
#endif
_Static_assert(DUMP_BANK_WORDS <= MAX_DUMP_WORDS,
               "dump ring exceeds reserved SRAM");
_Static_assert((DUMP_BANK_WORDS % IQ_CHUNK_SAMPLE_WORDS) == 0u,
               "dump ring must be chunk-aligned");
#if CONFIG_IDF_TARGET_ESP32S31
/* Legacy S31 snapshot path. The production continuous real-IF backend below
 * bypasses this TCM dump source entirely. */
#define ADC_DUMP_SOURCE_CONTINUOUS_RX_IQ 0x49980003u
#else
#define ADC_DUMP_SOURCE_CONTINUOUS_RX_IQ 15u
#endif
#if CONFIG_IDF_TARGET_ESP32S31
#define S31_POISON_WORD 0xa5a0055au
#define S31_ADC_DUMP_SOURCE_SEL_MASK 0x0fu
#define S31_ADCTRIG_TCM_DUMP_CTRL HP_SYSTEM_TCM_DATA_DUMP_MAC_MASK
#define S31_ADC_SOURCE_PULSE_TCM_BEFORE_COPY BIT(19)
#define S31_ADC_SOURCE_TCM_ALWAYS_ON BIT(16)
#define S31_ADC_SOURCE_LONG_GATE_DWELL BIT(17)
#define S31_ADC_SOURCE_FIELD7_DWELL_X2 BIT(18)
#define S31_ADC_SOURCE_FIELD7_DWELL_X4 BIT(24)
#define S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST 38u
#define S31_MODEM_DIAG_PROBE_OVERRIDE_LAST 47u
#define S31_MICRO_GATE_OVERRIDE 48u
#define S31_DATADUMP_MMIO_PROBE_OVERRIDE_FIRST 49u
#define S31_DATADUMP_MMIO_PROBE_OVERRIDE_LAST 55u
#define S31_LP_CORE_PROBE_OVERRIDE 56u
#define S31_HP_TCM_PROBE_OVERRIDE 57u
#define S31_SEAMLESS_MICRO_GATE_OVERRIDE 58u
#define S31_RING_CURSOR_OVERRIDE 59u
#define S31_GPIO_DIAG_OVERRIDE 60u
#define S31_PARLIO_NATIVE_IQ_OVERRIDE 61u
#define S31_PARLIO_HOST_IQ_OVERRIDE 62u
#define S31_PARLIO_BATCH_CHUNKS 14u
#define S31_CONTINUOUS_OUTPUT_CHUNKS (S31_PARLIO_BATCH_CHUNKS / 2u)
#define S31_CONTINUOUS_INPUT_RATE_HZ 4000000u
#define S31_CONTINUOUS_IF_HZ 1000000u
#define S31_PARLIO_DIAG_RATE_HZ 16000000u
#define S31_PARLIO_HOST_SYNC_40M BIT(31)
#define S31_PARLIO_NATIVE_PACKED_IQ BIT(22)
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
#define S31_DEFAULT_FILTER_BW_MHZ 16u
#else
#define S31_DEFAULT_FILTER_BW_MHZ RX_FILTER_BW_OPEN
#endif
#define S31_PARLIO_HOST_SYNC_RATE_HZ 40000000u
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
#define S31_PARLIO_HOST_REAL_RATE_HZ 16000000u
#else
#define S31_PARLIO_HOST_REAL_RATE_HZ 32000000u
#endif
/* selector 22, byte exchange 2/f/2/3. The exhaustive Pluto census found the
 * coherent signed ADC sample in diagnostic bits 8:0 for this route. */
#define S31_CONTINUOUS_DIAG_MODE 0x32f90016u
#define S31_PARLIO_BATCH_BYTES                                              \
  (S31_PARLIO_BATCH_CHUNKS * IQ_CHUNK_SAMPLE_WORDS * sizeof(uint16_t))
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
/* Eight complete banks provide 57 ms of raw acquisition elasticity. USB and
 * its transport consume the same average byte rate, so this absorbs
 * scheduling and endpoint jitter without copying in the ISR or surrendering
 * DMA ownership. */
#define S31_PARLIO_DMA_BYTES (8u * S31_PARLIO_BATCH_BYTES)
#else
#define S31_PARLIO_DMA_BYTES S31_PARLIO_BATCH_BYTES
#endif
#define S31_PARLIO_EVENT_RING_SIZE 128u
#define S31_PARLIO_EVENT_MAX_BYTES 4092u
/* The S31 driver uses 4032-byte cache-line-aligned GDMA nodes. Once this many
 * newer completions exist, the oldest node is already being reused. */
#define S31_PARLIO_DMA_NODE_BYTES 4032u
#define S31_PARLIO_DMA_NODE_COUNT                                           \
  ((S31_PARLIO_DMA_BYTES + S31_PARLIO_DMA_NODE_BYTES - 1u) /                \
   S31_PARLIO_DMA_NODE_BYTES)
/* Keep one complete descriptor between the node being copied and the GDMA
 * writer. Detecting only at NODE_COUNT is too late: GDMA may already have
 * started overwriting the oldest node by the time its predecessor reports
 * completion. */
#define S31_PARLIO_SAFE_NODE_BACKLOG (S31_PARLIO_DMA_NODE_COUNT - 1u)
/* Experimental pipelined snapshot backend.  The 64 KiB guard immediately
 * below the live aperture is used as an internal staging buffer so the dump
 * gate only has to be closed for a TCM-to-TCM copy. */
#define S31_ADC_SOURCE_PIPELINED_STAGE BIT(30)
#define S31_ADC_SOURCE_TCM_MASK_S 8u
#define S31_ADC_SOURCE_TCM_MASK_M (0xffu << S31_ADC_SOURCE_TCM_MASK_S)
#define S31_ADC_SOURCE_HW_DECIM_S 20u
#define S31_ADC_SOURCE_HW_DECIM_M (0xfu << S31_ADC_SOURCE_HW_DECIM_S)
#define S31_ADC_SOURCE_PULSE_DWELL_S 25u
#define S31_ADC_SOURCE_PULSE_DWELL_M (0x7u << S31_ADC_SOURCE_PULSE_DWELL_S)
#define S31_PULSE_META_MARKER 0x5a000000u
#define S31_TX_REPLAY_MODE 2u
#define S31_TX_REPLAY_WORDS 16384u
#define S31_TX_REPLAY_SIZE_M TX_REPLAY_SEGMENT_WORDS_MAX
/* A live handoff traverses eight independently owned physical TCM lanes.
 * Keep a replay lap lane-aligned so word zero returns to the same fabric
 * phase on every wrap; the hardware accepts non-aligned finite lengths, but
 * live replacement then produces alternating old/new lane mixtures. */
#define S31_TX_REPLAY_REFILL_WORDS (S31_TX_REPLAY_SIZE_M & ~7u)
/* Start a live ring update this many samples after enabling the cyclic
 * reader.  The MACTOADCDUMP write does not expose the modem reader's actual
 * start phase; beginning at word zero immediately can race it and create a
 * lane-interleaved blend of the old and new rings.  A fixed chase distance
 * leaves the writer behind the reader for the whole lap while still updating
 * every word before the reader reaches it on the following lap. */
#define S31_TX_REPLAY_REFILL_GUARD_WORDS 12288u
#define S31_TX_REPLAY_LOOP_BIT BIT(19)
#define S31_TX_REPLAY_DONE_BIT BIT(18)
#define S31_TX_REPLAY_ONE_SHOT_REQUEST BIT(8)
#define S31_TX_REPLAY_SEGMENT_COUNT_S 9u
#define S31_TX_REPLAY_SEGMENT_COUNT_M (0x7fu << S31_TX_REPLAY_SEGMENT_COUNT_S)
/* Hardware-reverse-engineering probe, deliberately outside the public TX
 * protocol.  In cyclic mode bit 15 asks the CPU to conjugate the live replay
 * symbol halfway through the bounded burst while the modem owns the TCM
 * aperture.  Success proves that an in-place ping-pong streamer is possible. */
#define S31_TX_REPLAY_LIVE_WRITE_PROBE BIT(15)
#define S31_TX_REPLAY_LIVE_GATE_PROBE BIT(14)
/* Bounded reverse-engineering mode for the undocumented replay path.
 * Patterns 6--8 place opposite complex tones at the starts of the two
 * physical 64 KiB halves of the S31's 128 KiB TCM dump window. Bits 17:21
 * encode a candidate while bit 16 enables the experiment. */
#define S31_TX_REPLAY_ADDRESS_PROBE BIT(16)
#define S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_S 17u
#define S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M (0x1fu << 17u)

#if !CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF
extern const uint8_t s31_tcm_probe_bin_start[]
    asm("_binary_s31_tcm_probe_bin_start");
extern const uint8_t s31_tcm_probe_bin_end[]
    asm("_binary_s31_tcm_probe_bin_end");
#endif
#endif
#define ADC_DECIMATION_MAX 10u
#define HZ_PER_MHZ 1000000u
/* ADC dump timing is derived from the 160 MHz modem clock, independently of
 * the HP CPU clock selected by CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ. */
#define ADC_DUMP_CLOCK_HZ 160000000u
#define STREAM_TASK_PRIORITY 18u
#define IQ_CHUNK_PRODUCER_TASK_STACK_BYTES 4096u
/* The raw S31 dump ring retains only about 333 us at 80 MSa/s.  Keep the
 * producer above ordinary system work on its dedicated core so snapshots
 * cannot be delayed past that hard deadline. */
#define IQ_CHUNK_PRODUCER_TASK_PRIORITY 24u
#define IQ_TASK_IDLE_DELAY_MS 2u

#define IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK 32u
#define IQ_AGC_TRIGGER_STATE_MASK_ALL 0xffffu
#define IQ_AGC_TRIGGER_MATCH_FSM 0u
#define IQ_AGC_TRIGGER_MATCH_MAX_GAIN 1u

enum {
  CHUNK_STREAM_SKIP = 0u,
  CHUNK_STREAM_TRIGGER = 1u,
  CHUNK_STREAM_PRE = 2u,
  CHUNK_STREAM_POST = 3u,
};

/* The dump window as a flat word array: word i is at physical bank i/16384,
 * offset i%16384. Per word: I=bits0-9, Q=bits10-19, rx_gain=20-27, agc=28-31.
 */
static volatile uint32_t *const DUMP_BASE =
    (volatile uint32_t *)MAC_DUMP_SRAM_BASE;

#if CONFIG_IDF_TARGET_ESP32S31
static void s31_enable_data_dump_clocks(const capture_config_t *config);
static void s31_configure_modem_diag(const capture_config_t *config);
static void s31_gpio_diag_prepare(const capture_config_t *config);
static void s31_gpio_diag_start(const capture_config_t *config);
static void s31_gpio_diag_stop(void);
static void s31_gpio_diag_release(void);
static void adctrig_prepare(uint32_t sample_count,
                            const capture_config_t *config);
static portMUX_TYPE s_tcm_dump_gate_mux;
static uint8_t *s_s31_parlio_dma_buffer;
#if CONFIG_ESP_SDR_TRANSPORT_USB && CONFIG_ESP_SDR_TRANSPORT_ETHERNET
static uint8_t *s_s31_parlio_dma_storage;
#endif
static bool s_s31_parlio_running;
static uint32_t s_s31_parlio_data_width;
static uint32_t s_s31_parlio_rate_hz;
static bool s_s31_parlio_swapped_pins;
static volatile bool s_s31_tx_replay_burst_complete;
static uint32_t *s_s31_tx_replay_upload;
static uint32_t s_s31_tx_replay_upload_words;
static uint64_t s_s31_tx_replay_start_time_ns;
static uint16_t s_s31_tx_replay_commit_flags;
static volatile bool s_s31_txdc_stream_active;
static volatile uint32_t s_s31_txdc_iwdt_heartbeat;
static uint32_t s_s31_txdc_stream_rate_code;
#define S31_TXDC_CONTINUATION_QUEUE_DEPTH 5u
/* Four 1,048,320-sample IQ10 continuations plus the active batch consume
 * about 13.1 MiB; allocating the following 2.5 MiB wire buffer then cannot be
 * guaranteed in 16 MiB PSRAM. Three queued batches retain 1.26--1.57 s of
 * continuation reservoir at 2.5--2 MSa/s while leaving upload headroom. */
#define S31_TXDC_ETHERNET_PACKED20_QUEUE_DEPTH 3u
typedef struct {
  uint32_t *upload;
  uint32_t words;
  uint16_t flags;
} s31_txdc_continuation_t;
static s31_txdc_continuation_t
    s_s31_txdc_continuations[S31_TXDC_CONTINUATION_QUEUE_DEPTH];
static volatile uint32_t s_s31_txdc_continuation_head;
static volatile uint32_t s_s31_txdc_continuation_tail;
static uint32_t *s_s31_txdc_reclaim_upload;
static uint32_t *s_s31_txdc_reclaim_upload2;
static uint32_t s_s31_txdc_commit_count;
static uint32_t s_s31_txdc_flags_trace;
static portMUX_TYPE s_s31_tx_replay_upload_mux = portMUX_INITIALIZER_UNLOCKED;
static iq_tx_replay_diag_t s_s31_tx_replay_diag;
static uint32_t s_s31_tx_live_probe_words[16];
static volatile bool s_s31_tx_worker_mode;

static void s31_txdc_release_upload(uint32_t *upload) {
  if (!iq_usb_tx_recycle(upload)) {
    heap_caps_free(upload);
  }
}

/* One transport producer and one RF/stager consumer exchange complete host
 * batches through this bounded FIFO. The former single pointer could be full
 * during the initial replay/configuration latency even though steady Ethernet
 * throughput exceeded RF consumption; the fourth continuation then arrived
 * completely but could not be adopted before the realtime starvation limit. */
static uint32_t s31_txdc_continuation_count(void) {
  const uint32_t head = __atomic_load_n(&s_s31_txdc_continuation_head,
                                        __ATOMIC_ACQUIRE);
  const uint32_t tail = __atomic_load_n(&s_s31_txdc_continuation_tail,
                                        __ATOMIC_ACQUIRE);
  return head - tail;
}

static bool s31_txdc_continuation_has_final(void) {
  const uint32_t head = __atomic_load_n(&s_s31_txdc_continuation_head,
                                        __ATOMIC_ACQUIRE);
  const uint32_t tail = __atomic_load_n(&s_s31_txdc_continuation_tail,
                                        __ATOMIC_ACQUIRE);
  for (uint32_t cursor = tail; cursor != head; ++cursor) {
    if ((s_s31_txdc_continuations[
             cursor % S31_TXDC_CONTINUATION_QUEUE_DEPTH]
             .flags & IQ_TX_UDP_FLAG_MORE) == 0u) {
      return true;
    }
  }
  return false;
}

static bool s31_txdc_continuation_push(uint32_t *upload, uint32_t words,
                                       uint16_t flags) {
  const uint32_t head = __atomic_load_n(&s_s31_txdc_continuation_head,
                                        __ATOMIC_RELAXED);
  const uint32_t tail = __atomic_load_n(&s_s31_txdc_continuation_tail,
                                        __ATOMIC_ACQUIRE);
  if (head - tail >= S31_TXDC_CONTINUATION_QUEUE_DEPTH) {
    return false;
  }
  s31_txdc_continuation_t *slot =
      &s_s31_txdc_continuations[head % S31_TXDC_CONTINUATION_QUEUE_DEPTH];
  slot->upload = upload;
  slot->words = words;
  slot->flags = flags;
  __atomic_store_n(&s_s31_txdc_continuation_head, head + 1u,
                   __ATOMIC_RELEASE);
  return true;
}

static uint32_t *s31_txdc_continuation_pop(uint32_t *words,
                                           uint16_t *flags) {
  const uint32_t tail = __atomic_load_n(&s_s31_txdc_continuation_tail,
                                        __ATOMIC_RELAXED);
  const uint32_t head = __atomic_load_n(&s_s31_txdc_continuation_head,
                                        __ATOMIC_ACQUIRE);
  if (tail == head) {
    return NULL;
  }
  s31_txdc_continuation_t *slot =
      &s_s31_txdc_continuations[tail % S31_TXDC_CONTINUATION_QUEUE_DEPTH];
  uint32_t *upload = slot->upload;
  *words = slot->words;
  *flags = slot->flags;
  __atomic_store_n(&s_s31_txdc_continuation_tail, tail + 1u,
                   __ATOMIC_RELEASE);
  return upload;
}

static void s31_txdc_continuation_drain(void) {
  uint32_t words;
  uint16_t flags;
  uint32_t *upload;
  while ((upload = s31_txdc_continuation_pop(&words, &flags)) != NULL) {
    s31_txdc_release_upload(upload);
  }
}
/* Rate codes 14 and 15 are the native-USB DIRAM-staged backends at 5 MSa/s
 * and 320/60 MSa/s respectively. Core 0 copies packed PSRAM uploads into
 * ordinary application DIRAM while the cycle-exact emitter on core 1 drains
 * another slot. Once Ethernet is stopped, its idle GMAC RX buffers join the
 * seven static slots to form the USB scatter ring. Ethernet uses the static
 * slots alone. Keeping 16 rather than 30 live RX descriptors pays for four
 * extra static slots without increasing boot-time internal-memory pressure;
 * 16 descriptors still cover two complete eight-datagram host flights.
 * Earlier versions used the otherwise convenient modem TCM
 * aperture at 0x2f05xxxx. Hardware captures showed that *any* write into that
 * ownership-controlled aperture can block the other core's reader for
 * milliseconds, even when the slots occupy different nominal 64 KiB banks.
 * Ordinary DIRAM avoids both that special fabric and the HP/LP bridge, which
 * deadlocks under simultaneous dual-core access. */
#define S31_TXDC_TCM_STATIC_SLOT_COUNT 7u
#define S31_TXDC_TCM_SLOT_MAX                                                  \
  ( CONFIG_ETH_DMA_RX_BUFFER_NUM + S31_TXDC_TCM_STATIC_SLOT_COUNT)
#define S31_TXDC_TCM_SLOT_WORDS 1280u
static DRAM_ATTR uint32_t
    s_s31_txdc_diram_stage[S31_TXDC_TCM_STATIC_SLOT_COUNT]
                            [S31_TXDC_TCM_SLOT_WORDS]
        __attribute__((aligned(64)));
static uintptr_t s_s31_txdc_tcm_slot_base[S31_TXDC_TCM_SLOT_MAX];
static uint32_t s_s31_txdc_tcm_slot_capacity[S31_TXDC_TCM_SLOT_MAX];
static uint32_t s_s31_txdc_tcm_slot_count;
static bool s_s31_txdc_tcm_slots_borrowed;
/* The realtime reader and transport-core stager exchange ownership through
 * these descriptors.  Keep every slot on a distinct data-cache line: when
 * the descriptors were packed together, publishing a future slot invalidated
 * the line that core 1 was using for the current slot and could stall the
 * cycle-exact loop for tens of thousands of cycles. */
typedef struct __attribute__((aligned(64))) {
  volatile uint32_t ready;
  uint32_t words;
  uint32_t final;
  uint8_t cache_line_padding[64u - 3u * sizeof(uint32_t)];
} s31_txdc_tcm_slot_t;
_Static_assert(sizeof(s31_txdc_tcm_slot_t) == 64u,
               "TXDC slot descriptors must occupy one cache line");
_Static_assert(_Alignof(s31_txdc_tcm_slot_t) == 64u,
               "TXDC slot descriptors must be cache-line aligned");
static s31_txdc_tcm_slot_t
    s_s31_txdc_static_slots[S31_TXDC_TCM_STATIC_SLOT_COUNT]
        __attribute__((aligned(64)));
static s31_txdc_tcm_slot_t
    *s_s31_txdc_tcm_slots[S31_TXDC_TCM_SLOT_MAX];
static TaskHandle_t s_s31_txdc_stager_task_handle;
static esp_timer_handle_t s_s31_txdc_stager_wakeup_timer;
static uint32_t *s_s31_txdc_stager_initial_upload;
static uint32_t s_s31_txdc_stager_initial_words;
static uint16_t s_s31_txdc_stager_initial_flags;
static volatile bool s_s31_txdc_stager_start;
static volatile bool s_s31_txdc_stager_stop;
static volatile bool s_s31_txdc_stager_done;
static volatile uint32_t s_s31_txdc_stager_copy_max_cycles;
static volatile bool s_s31_txdc_rf_timing_active;
static volatile uint32_t s_s31_txdc_rf_start_cycle;
static volatile uint32_t s_s31_txdc_rf_cycles_per_sample = 64u;

/* Ethernet's stager must neither busy-poll at the same priority as the
 * transport nor disappear below a continuously runnable transport task.
 * Wake the priority-24 stager from a CPU0 timer ISR every 1.4 ms while an
 * Ethernet TX is live. Each 1,280-word slot lasts 320 us at 4 MSa/s, so one
 * wake refills four to five slots while the seven-slot reservoir leaves
 * 840 us of scheduling margin. EMAC/HTTP run uninterrupted between refill
 * bursts. */
static void IRAM_ATTR s31_txdc_stager_wakeup_isr(void *arg) {
  (void)arg;
  TaskHandle_t task = s_s31_txdc_stager_task_handle;
  if (task == NULL || s_s31_txdc_tcm_slots_borrowed ||
      __atomic_load_n(&s_s31_txdc_stager_done, __ATOMIC_RELAXED)) {
    return;
  }
  BaseType_t task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(task, &task_woken);
  if (task_woken == pdTRUE) {
    esp_timer_isr_dispatch_need_yield();
  }
}

static void s31_txdc_stager_wakeup_start(void) {
  if (!s_s31_txdc_tcm_slots_borrowed &&
      s_s31_txdc_stager_wakeup_timer != NULL &&
      !esp_timer_is_active(s_s31_txdc_stager_wakeup_timer)) {
    ESP_ERROR_CHECK(
        esp_timer_start_periodic(s_s31_txdc_stager_wakeup_timer, 1400u));
  }
}

static void s31_txdc_stager_wakeup_stop(void) {
  if (s_s31_txdc_stager_wakeup_timer != NULL &&
      esp_timer_is_active(s_s31_txdc_stager_wakeup_timer)) {
    ESP_ERROR_CHECK(esp_timer_stop(s_s31_txdc_stager_wakeup_timer));
  }
  if (s_s31_txdc_stager_task_handle != NULL) {
    xTaskNotifyGive(s_s31_txdc_stager_task_handle);
  }
}

static void s31_txdc_configure_stage_slots(void) {
  uint32_t *borrowed[S31_TXDC_TCM_SLOT_MAX];
  uint32_t borrowed_words = 0u;
  const uint32_t borrowed_count = iq_network_usb_tx_stage_buffers(
      borrowed, CONFIG_ETH_DMA_RX_BUFFER_NUM, &borrowed_words);
  if (borrowed_count != 0u &&
      borrowed_words > sizeof(s31_txdc_tcm_slot_t) / sizeof(uint32_t)) {
    for (uint32_t i = 0u; i < borrowed_count; ++i) {
      /* Keep the descriptor and data in the same otherwise-idle DMA buffer.
       * This obtains a 64-byte isolated metadata line without consuming the
       * internal heap that Wi-Fi initialization still needs. */
      s_s31_txdc_tcm_slots[i] = (s31_txdc_tcm_slot_t *)borrowed[i];
      s_s31_txdc_tcm_slot_base[i] =
          (uintptr_t)borrowed[i] + sizeof(s31_txdc_tcm_slot_t);
      s_s31_txdc_tcm_slot_capacity[i] =
          borrowed_words - sizeof(s31_txdc_tcm_slot_t) / sizeof(uint32_t);
    }
    for (uint32_t i = 0u; i < S31_TXDC_TCM_STATIC_SLOT_COUNT; ++i) {
      const uint32_t slot = borrowed_count + i;
      s_s31_txdc_tcm_slots[slot] = &s_s31_txdc_static_slots[i];
      s_s31_txdc_tcm_slot_base[slot] = (uintptr_t)&s_s31_txdc_diram_stage[i][0];
      s_s31_txdc_tcm_slot_capacity[slot] = S31_TXDC_TCM_SLOT_WORDS;
    }
    s_s31_txdc_tcm_slot_count = borrowed_count + S31_TXDC_TCM_STATIC_SLOT_COUNT;
    s_s31_txdc_tcm_slots_borrowed = true;
    /* TinyUSB's device task is priority 23. The borrowed-slot wait loop uses
     * taskYIELD(), which only hands control to an equal-priority peer; keeping
     * the Ethernet-oriented priority 24 here busy-spins and starves USB
     * control/OUT completions while RF is active. */
    vTaskPrioritySet(s_s31_txdc_stager_task_handle, 23u);
    return;
  }
  for (uint32_t i = 0u; i < S31_TXDC_TCM_STATIC_SLOT_COUNT; ++i) {
    s_s31_txdc_tcm_slots[i] = &s_s31_txdc_static_slots[i];
    s_s31_txdc_tcm_slot_base[i] =
        (uintptr_t)&s_s31_txdc_diram_stage[i][0];
    s_s31_txdc_tcm_slot_capacity[i] = S31_TXDC_TCM_SLOT_WORDS;
  }
  s_s31_txdc_tcm_slot_count = S31_TXDC_TCM_STATIC_SLOT_COUNT;
  s_s31_txdc_tcm_slots_borrowed = false;
  /* Ethernet's priority-23 receive task is continuously runnable under TX
   * load, so its timer-woken stager must preempt it briefly to refill. */
  vTaskPrioritySet(s_s31_txdc_stager_task_handle, 24u);
}
extern void s31_pie_memcpy_aligned(void *dst, const void *src,
                                   size_t byte_count);

static void s31_tx_replay_copy(void *destination, const uint32_t *source,
                               uint32_t words) {
  size_t bytes = words * sizeof(*source);
#if CONFIG_ESP_SDR_S31_TX_PIE_REFILL
  size_t pie_bytes = bytes & ~(size_t)15u;
  if ((((uintptr_t)destination | (uintptr_t)source) & 15u) == 0u &&
      pie_bytes != 0u) {
    s31_pie_memcpy_aligned(destination, source, pie_bytes);
    destination = (uint8_t *)destination + pie_bytes;
    source = (const uint32_t *)((const uint8_t *)source + pie_bytes);
    bytes -= pie_bytes;
  }
#endif
  if (bytes != 0u) {
    memcpy(destination, source, bytes);
  }
}

/* A continuous lane refill deliberately keeps interrupts disabled and the
 * other CPU stalled while the modem owns the TCM aperture. The normal tick
 * hooks therefore cannot service the shared interrupt watchdog. Feed it only
 * after a complete ring was updated successfully: a wedged inner loop still
 * retains the normal watchdog failure mode. ESP-IDF selects MWDT1 as the IWDT
 * on S31 because this target has two timer-group instances. */
static inline void s31_tx_replay_feed_iwdt(void) {
#if CONFIG_ESP_INT_WDT
  mwdt_ll_write_protect_disable(&TIMERG1);
  mwdt_ll_feed(&TIMERG1);
  mwdt_ll_write_protect_enable(&TIMERG1);
#endif
}

/* The cycle-exact producer cannot take its normal tick interrupt while a
 * TXDC stream is active. Publish progress from the realtime core and let the
 * still-schedulable transport core feed the shared IWDT from its tick hook.
 * This keeps the timer-group MMIO sequence out of the IQ sample loop while
 * preserving watchdog coverage: a wedged producer stops advancing the
 * heartbeat, so the IWDT is no longer fed. */
static void IRAM_ATTR s31_txdc_iwdt_tick_hook(void) {
#if CONFIG_ESP_INT_WDT
  static uint32_t last_heartbeat;
  const uint32_t heartbeat = __atomic_load_n(
      &s_s31_txdc_iwdt_heartbeat, __ATOMIC_RELAXED);
  if (!__atomic_load_n(&s_s31_txdc_stream_active, __ATOMIC_RELAXED)) {
    last_heartbeat = heartbeat;
    return;
  }
  if (heartbeat != last_heartbeat) {
    last_heartbeat = heartbeat;
    s31_tx_replay_feed_iwdt();
  }
#endif
}

/* Keep well inside the 300 ms interrupt-watchdog timeout without perturbing
 * every 65,536-sample block. One feed interval is 118 ms at 40/9 MSa/s and
 * 157 ms at 10/3 MSa/s. */
#define S31_TXDC_IWDT_FEED_WORDS (1u << 17u)

/* Once the prebuffered IQ batches are exhausted, the stream has already
 * underflowed and TXDC merely holds the last sample. Do not keep RF and the
 * realtime critical section alive for the old ~938 ms grace period: hardware
 * starvation tests showed that path ending in a brownout reset. A 200 ms
 * backstop covers measured 4 MSa/s Ethernet handoff jitter while remaining
 * inside the 300 ms interrupt-watchdog interval. */
#define S31_TXDC_CONTINUATION_WAIT_CYCLES 64000000u

/* Refill one cyclic replay ring with the eight fixed TCM ownership lanes
 * fully unrolled. Keeping this helper small and resident in IRAM avoids the
 * variable shift, lane branch, and flash-cache pressure that dominated the
 * generic loop at the 48-cycle (6.667 MSa/s) hardware rate. Source values are
 * loaded a complete 32-byte group at a time so one PSRAM cache-line fill is
 * amortized across eight timed fabric handoffs. */
static void IRAM_ATTR __attribute__((optimize("O3")))
s31_tx_replay_refill_ring(const uint32_t *source, uint32_t words,
                          uint32_t sample_cycles, uint32_t first_deadline) {
  volatile uint32_t *const gate =
      (volatile uint32_t *)(uintptr_t)HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG;
  uint32_t deadline = first_deadline;
  uint32_t word = 0u;

#define S31_TX_WAIT_AND_WRITE(offset, lane_mask, value)                      \
  do {                                                                       \
    while ((int32_t)(esp_cpu_get_cycle_count() - deadline) < 0) {            \
      __asm__ __volatile__("nop");                                          \
    }                                                                        \
    *gate = (lane_mask);                                                     \
    DUMP_BASE[word + (offset)] = (value);                                    \
    *gate = S31_ADCTRIG_TCM_DUMP_CTRL;                                       \
    deadline += sample_cycles;                                               \
  } while (0)

  for (; word + 8u <= words; word += 8u) {
    const uint32_t v0 = source[word + 0u];
    const uint32_t v1 = source[word + 1u];
    const uint32_t v2 = source[word + 2u];
    const uint32_t v3 = source[word + 3u];
    const uint32_t v4 = source[word + 4u];
    const uint32_t v5 = source[word + 5u];
    const uint32_t v6 = source[word + 6u];
    const uint32_t v7 = source[word + 7u];
    S31_TX_WAIT_AND_WRITE(0u, 0xfe000000u, v0);
    S31_TX_WAIT_AND_WRITE(1u, 0xfd000000u, v1);
    S31_TX_WAIT_AND_WRITE(2u, 0xfb000000u, v2);
    S31_TX_WAIT_AND_WRITE(3u, 0xf7000000u, v3);
    S31_TX_WAIT_AND_WRITE(4u, 0xef000000u, v4);
    S31_TX_WAIT_AND_WRITE(5u, 0xdf000000u, v5);
    S31_TX_WAIT_AND_WRITE(6u, 0xbf000000u, v6);
    S31_TX_WAIT_AND_WRITE(7u, 0x7f000000u, v7);
  }
  for (; word < words; ++word) {
    const uint32_t value = source[word];
    const uint32_t lane_mask =
        S31_ADCTRIG_TCM_DUMP_CTRL & ~BIT(24u + (word & 7u));
    S31_TX_WAIT_AND_WRITE(0u, lane_mask, value);
  }
#undef S31_TX_WAIT_AND_WRITE
}

#if CONFIG_ESP_SDR_S31_TX_DIRECT_DMA_REFILL
#define S31_TX_REFILL_DMA_LINK_ITEMS 20u
static gdma_channel_handle_t s_s31_tx_refill_dma_tx;
static gdma_channel_handle_t s_s31_tx_refill_dma_rx;
static gdma_link_list_handle_t s_s31_tx_refill_dma_tx_links;
static gdma_link_list_handle_t s_s31_tx_refill_dma_rx_links;
static gdma_link_list_handle_t s_s31_tx_scatter_dma_tx_links;
static gdma_link_list_handle_t s_s31_tx_scatter_dma_rx_links;
static uint32_t *s_s31_tx_scatter_dma_words;
static gdma_channel_handle_t s_s31_tx_scatter_ahb_tx;
static gdma_channel_handle_t s_s31_tx_scatter_ahb_rx;
static gdma_link_list_handle_t s_s31_tx_scatter_ahb_tx_links;
static gdma_link_list_handle_t s_s31_tx_scatter_ahb_rx_links;
static size_t s_s31_tx_scatter_ahb_tx_ext_align;
static int s_s31_tx_scatter_ahb_channel = -1;
static size_t s_s31_tx_refill_dma_tx_ext_align;
static size_t s_s31_tx_refill_dma_rx_int_align;
static int s_s31_tx_refill_dma_group = -1;
static int s_s31_tx_refill_dma_channel = -1;

static bool s31_tx_direct_dma_prepare(void) {
  if (s_s31_tx_refill_dma_tx != NULL && s_s31_tx_refill_dma_rx != NULL) {
    return true;
  }
  gdma_channel_alloc_config_t allocation = {0};
  esp_err_t error = gdma_new_axi_channel(
      &allocation, &s_s31_tx_refill_dma_tx, &s_s31_tx_refill_dma_rx);
  if (error != ESP_OK) {
    ESP_LOGW("iq_capture", "TX refill AXI-GDMA allocation failed: %s",
             esp_err_to_name(error));
    return false;
  }
  gdma_strategy_config_t strategy = {
      .owner_check = true,
      .auto_update_desc = true,
      .eof_till_data_popped = true,
  };
  gdma_transfer_config_t transfer = {
      .max_data_burst_size = 64u,
      .access_ext_mem = true,
  };
  gdma_trigger_t trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_M2M, 0);
  uint32_t free_mask = 0u;
  if ((error = gdma_apply_strategy(s_s31_tx_refill_dma_tx, &strategy)) !=
          ESP_OK ||
      (error = gdma_apply_strategy(s_s31_tx_refill_dma_rx, &strategy)) !=
          ESP_OK ||
      (error = gdma_config_transfer(s_s31_tx_refill_dma_tx, &transfer)) !=
          ESP_OK ||
      (error = gdma_config_transfer(s_s31_tx_refill_dma_rx, &transfer)) !=
          ESP_OK ||
      (error = gdma_get_free_m2m_trig_id_mask(s_s31_tx_refill_dma_tx,
                                               &free_mask)) != ESP_OK ||
      free_mask == 0u) {
    ESP_LOGW("iq_capture", "TX refill AXI-GDMA setup failed: %s",
             free_mask == 0u ? "no M2M trigger" : esp_err_to_name(error));
    return false;
  }
  trigger.instance_id = __builtin_ctz(free_mask);
  gdma_channel_alignment_info_t tx_alignment = {0};
  gdma_channel_alignment_info_t rx_alignment = {0};
  if ((error = gdma_connect(s_s31_tx_refill_dma_rx, trigger)) != ESP_OK ||
      (error = gdma_connect(s_s31_tx_refill_dma_tx, trigger)) != ESP_OK ||
      (error = gdma_get_channel_alignment_constraints(
           s_s31_tx_refill_dma_tx, &tx_alignment)) != ESP_OK ||
      (error = gdma_get_channel_alignment_constraints(
           s_s31_tx_refill_dma_rx, &rx_alignment)) != ESP_OK ||
      (error = gdma_get_group_channel_id(
           s_s31_tx_refill_dma_rx, &s_s31_tx_refill_dma_group,
           &s_s31_tx_refill_dma_channel)) != ESP_OK) {
    ESP_LOGW("iq_capture", "TX refill AXI-GDMA connection failed: %s",
             esp_err_to_name(error));
    return false;
  }
  s_s31_tx_refill_dma_tx_ext_align = tx_alignment.ext_enc_mem_alignment;
  s_s31_tx_refill_dma_rx_int_align = rx_alignment.int_mem_alignment;
  gdma_link_list_config_t links = {
      .num_items = S31_TX_REFILL_DMA_LINK_ITEMS,
      .item_alignment = 8u,
      .flags = {
          .items_in_ext_mem = false,
          .check_owner = true,
      },
  };
  if ((error = gdma_new_link_list(
           &links, &s_s31_tx_refill_dma_tx_links)) != ESP_OK ||
      (error = gdma_new_link_list(
           &links, &s_s31_tx_refill_dma_rx_links)) != ESP_OK) {
    ESP_LOGW("iq_capture", "TX refill AXI-GDMA links failed: %s",
             esp_err_to_name(error));
    return false;
  }
  if (s_s31_tx_refill_dma_group != GDMA_LL_AXI_GROUP_START_ID) {
    ESP_LOGW("iq_capture", "TX refill allocated unexpected GDMA group %d",
             s_s31_tx_refill_dma_group);
    return false;
  }
  return true;
}

static bool s31_tx_direct_dma_copy(const uint32_t *source, uint32_t words) {
  size_t bytes = words * sizeof(*source);
  esp_err_t error = gdma_reset(s_s31_tx_refill_dma_tx);
  if (error == ESP_OK) {
    error = gdma_reset(s_s31_tx_refill_dma_rx);
  }
  gdma_buffer_mount_config_t tx_buffer = {
      .buffer = (void *)source,
      .buffer_alignment = s_s31_tx_refill_dma_tx_ext_align,
      .length = bytes,
      .flags = {
          .mark_eof = true,
          .mark_final = GDMA_FINAL_LINK_TO_NULL,
          .bypass_buffer_addr_align_check = true,
      },
  };
  gdma_buffer_mount_config_t rx_buffer = {
      .buffer = (void *)DUMP_BASE,
      .buffer_alignment = s_s31_tx_refill_dma_rx_int_align,
      .length = bytes,
      .flags = {
          .mark_final = GDMA_FINAL_LINK_TO_NULL,
          .bypass_buffer_addr_align_check = true,
      },
  };
  if (error == ESP_OK) {
    error = gdma_link_mount_buffers(s_s31_tx_refill_dma_tx_links, 0,
                                    &tx_buffer, 1, NULL);
  }
  if (error == ESP_OK) {
    error = gdma_link_mount_buffers(s_s31_tx_refill_dma_rx_links, 0,
                                    &rx_buffer, 1, NULL);
  }
  if (error != ESP_OK) {
    ESP_LOGW("iq_capture", "TX refill AXI-GDMA descriptor setup failed: %s",
             esp_err_to_name(error));
    return false;
  }
  axi_dma_ll_rx_clear_interrupt_status(&AXI_DMA,
                                        s_s31_tx_refill_dma_channel,
                                        UINT32_MAX);
  axi_dma_ll_tx_clear_interrupt_status(&AXI_DMA,
                                        s_s31_tx_refill_dma_channel,
                                        UINT32_MAX);
  error = gdma_start(s_s31_tx_refill_dma_rx,
                     gdma_link_get_head_addr(s_s31_tx_refill_dma_rx_links));
  if (error == ESP_OK) {
    error = gdma_start(s_s31_tx_refill_dma_tx,
                       gdma_link_get_head_addr(
                           s_s31_tx_refill_dma_tx_links));
  }
  if (error != ESP_OK) {
    ESP_LOGW("iq_capture", "TX refill AXI-GDMA start failed: %s",
             esp_err_to_name(error));
    return false;
  }
  const uint32_t wait_start = esp_cpu_get_cycle_count();
  uint32_t dma_status = 0u;
  while (((dma_status = axi_dma_ll_rx_get_interrupt_status(
               &AXI_DMA, s_s31_tx_refill_dma_channel, true)) &
          (GDMA_LL_EVENT_RX_SUC_EOF | GDMA_LL_EVENT_RX_DESC_ERROR |
           GDMA_LL_EVENT_RX_DESC_EMPTY)) == 0u &&
         esp_cpu_get_cycle_count() - wait_start < 30000000u) {
    __asm__ __volatile__("nop");
  }
  if ((dma_status & GDMA_LL_EVENT_RX_SUC_EOF) == 0u) {
    ESP_LOGW("iq_capture", "TX refill AXI-GDMA failed: status=%08" PRIx32,
             dma_status);
    return false;
  }
  __asm__ __volatile__("fence rw, rw" ::: "memory");
  return true;
}

#define S31_TX_SCATTER_PROBE_WORDS 1024u
#define S31_TX_SCATTER_PROBE_ITEMS (3u * S31_TX_SCATTER_PROBE_WORDS)

/* Build an AXI-GDMA scatter stream whose destination triplets are the TCM
 * ownership register, one replay word, and the ownership register again.
 * Starting this after the cyclic reader has consumed the old ring lets DMA
 * race ahead of the next pass without ever presenting a CPU-owned lane for
 * longer than one four-byte transfer. */
static bool s31_tx_scatter_dma_prepare(uint32_t words) {
  if (words == 0u || words > S31_TX_SCATTER_PROBE_WORDS ||
      !s31_tx_direct_dma_prepare()) {
    return false;
  }
  esp_err_t error = ESP_OK;
  if (s_s31_tx_scatter_dma_words == NULL) {
    s_s31_tx_scatter_dma_words = heap_caps_aligned_alloc(
        64u, S31_TX_SCATTER_PROBE_ITEMS * sizeof(uint32_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_s31_tx_scatter_dma_words == NULL) {
      ESP_LOGW("iq_capture", "TX scatter-GDMA source allocation failed");
      return false;
    }
  }
  gdma_link_list_config_t tx_links = {
      .num_items = 4u,
      .item_alignment = 8u,
      .flags = {.items_in_ext_mem = true, .check_owner = true},
  };
  gdma_link_list_config_t rx_links = {
      .num_items = S31_TX_SCATTER_PROBE_ITEMS,
      .item_alignment = 8u,
      .flags = {.items_in_ext_mem = true, .check_owner = true},
  };
  if (s_s31_tx_scatter_dma_tx_links == NULL) {
    error = gdma_new_link_list(&tx_links, &s_s31_tx_scatter_dma_tx_links);
  }
  if (error == ESP_OK && s_s31_tx_scatter_dma_rx_links == NULL) {
    error = gdma_new_link_list(&rx_links, &s_s31_tx_scatter_dma_rx_links);
  }
  if (error != ESP_OK) {
    ESP_LOGW("iq_capture", "TX scatter-GDMA link allocation failed: %s",
             esp_err_to_name(error));
    return false;
  }

  for (uint32_t word = 0u; word < words; ++word) {
    s_s31_tx_scatter_dma_words[3u * word] =
        S31_ADCTRIG_TCM_DUMP_CTRL & ~BIT(24u + (word & 7u));
    s_s31_tx_scatter_dma_words[3u * word + 1u] =
        s_s31_tx_live_probe_words[word & 15u];
    s_s31_tx_scatter_dma_words[3u * word + 2u] =
        S31_ADCTRIG_TCM_DUMP_CTRL;

    gdma_buffer_mount_config_t destinations[3] = {
        {
            .buffer = (void *)(uintptr_t)HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
            .length = sizeof(uint32_t),
            .flags = {.bypass_buffer_addr_align_check = true},
        },
        {
            .buffer = (void *)(DUMP_BASE + word),
            .length = sizeof(uint32_t),
            .flags = {.bypass_buffer_addr_align_check = true},
        },
        {
            .buffer = (void *)(uintptr_t)HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
            .length = sizeof(uint32_t),
            .flags = {
                .mark_eof = word + 1u == words,
                .mark_final = word + 1u == words
                                  ? GDMA_FINAL_LINK_TO_NULL
                                  : GDMA_FINAL_LINK_TO_DEFAULT,
                .bypass_buffer_addr_align_check = true,
            },
        },
    };
    error = gdma_link_mount_buffers(s_s31_tx_scatter_dma_rx_links,
                                    3u * word, destinations, 3u, NULL);
    if (error != ESP_OK) {
      ESP_LOGW("iq_capture", "TX scatter-GDMA RX setup failed: %s",
               esp_err_to_name(error));
      return false;
    }
  }
  const size_t transfer_bytes = 3u * words * sizeof(uint32_t);
  if (esp_cache_msync(s_s31_tx_scatter_dma_words, transfer_bytes,
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                          ESP_CACHE_MSYNC_FLAG_UNALIGNED) != ESP_OK) {
    ESP_LOGW("iq_capture", "TX scatter-GDMA source publication failed");
    return false;
  }
  gdma_buffer_mount_config_t source = {
      .buffer = s_s31_tx_scatter_dma_words,
      .buffer_alignment = s_s31_tx_refill_dma_tx_ext_align,
      .length = transfer_bytes,
      .flags = {
          .mark_eof = true,
          .mark_final = GDMA_FINAL_LINK_TO_NULL,
          .bypass_buffer_addr_align_check = true,
      },
  };
  error = gdma_link_mount_buffers(s_s31_tx_scatter_dma_tx_links, 0,
                                  &source, 1u, NULL);
  if (error != ESP_OK) {
    ESP_LOGW("iq_capture", "TX scatter-GDMA TX setup failed: %s",
             esp_err_to_name(error));
    return false;
  }
  return true;
}

static bool s31_tx_scatter_dma_run(uint32_t *elapsed_cycles,
                                   uint32_t *dma_status) {
  const uint32_t saved_internal_start = AXI_DMA.intr_mem_start_addr.val;
  const uint32_t saved_internal_end = AXI_DMA.intr_mem_end_addr.val;
  /* The default AXI-GDMA protection window contains the 0x2f06xxxx TCM but
   * excludes HP-system MMIO at 0x20586114. Both addresses are deliberate
   * destinations in this scatter transaction; open only their common region
   * for the bounded transfer and restore the driver's original policy below. */
  AXI_DMA.intr_mem_start_addr.val = 0x20000000u;
  AXI_DMA.intr_mem_end_addr.val = 0x2fffffffu;
  esp_err_t error = gdma_reset(s_s31_tx_refill_dma_tx);
  if (error == ESP_OK) {
    error = gdma_reset(s_s31_tx_refill_dma_rx);
  }
  axi_dma_ll_rx_clear_interrupt_status(&AXI_DMA,
                                        s_s31_tx_refill_dma_channel,
                                        UINT32_MAX);
  axi_dma_ll_tx_clear_interrupt_status(&AXI_DMA,
                                        s_s31_tx_refill_dma_channel,
                                        UINT32_MAX);
  const uint32_t started = esp_cpu_get_cycle_count();
  if (error == ESP_OK) {
    error = gdma_start(s_s31_tx_refill_dma_rx,
                       gdma_link_get_head_addr(
                           s_s31_tx_scatter_dma_rx_links));
  }
  if (error == ESP_OK) {
    error = gdma_start(s_s31_tx_refill_dma_tx,
                       gdma_link_get_head_addr(
                           s_s31_tx_scatter_dma_tx_links));
  }
  uint32_t status = 0u;
  while (error == ESP_OK &&
         ((status = axi_dma_ll_rx_get_interrupt_status(
               &AXI_DMA, s_s31_tx_refill_dma_channel, true)) &
          (GDMA_LL_EVENT_RX_SUC_EOF | GDMA_LL_EVENT_RX_DESC_ERROR |
           GDMA_LL_EVENT_RX_DESC_EMPTY)) == 0u &&
         esp_cpu_get_cycle_count() - started < 30000000u) {
    __asm__ __volatile__("nop");
  }
  *elapsed_cycles = esp_cpu_get_cycle_count() - started;
  *dma_status = status;
  AXI_DMA.intr_mem_start_addr.val = saved_internal_start;
  AXI_DMA.intr_mem_end_addr.val = saved_internal_end;
  return error == ESP_OK && (status & GDMA_LL_EVENT_RX_SUC_EOF) != 0u;
}

/* Repeat the same experiment on the AHB DMA. Unlike AXI DMA, this master
 * shares the CPU-facing system fabric and may therefore honor a partially
 * released TCM lane. */
static bool s31_tx_scatter_ahb_prepare(uint32_t words) {
  if (words == 0u || words > S31_TX_SCATTER_PROBE_WORDS) {
    return false;
  }
  esp_err_t error = ESP_OK;
  if (s_s31_tx_scatter_ahb_tx == NULL) {
    gdma_channel_alloc_config_t allocation = {0};
    error = gdma_new_ahb_channel(&allocation,
                                 &s_s31_tx_scatter_ahb_tx,
                                 &s_s31_tx_scatter_ahb_rx);
    gdma_strategy_config_t strategy = {
        .owner_check = true,
        .auto_update_desc = true,
        .eof_till_data_popped = true,
    };
    gdma_transfer_config_t transfer = {
        .max_data_burst_size = 16u,
        .access_ext_mem = true,
    };
    gdma_trigger_t trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_M2M, 0);
    uint32_t free_mask = 0u;
    gdma_channel_alignment_info_t tx_alignment = {0};
    gdma_channel_alignment_info_t rx_alignment = {0};
    if (error == ESP_OK) {
      error = gdma_apply_strategy(s_s31_tx_scatter_ahb_tx, &strategy);
    }
    if (error == ESP_OK) {
      error = gdma_apply_strategy(s_s31_tx_scatter_ahb_rx, &strategy);
    }
    if (error == ESP_OK) {
      error = gdma_config_transfer(s_s31_tx_scatter_ahb_tx, &transfer);
    }
    if (error == ESP_OK) {
      error = gdma_config_transfer(s_s31_tx_scatter_ahb_rx, &transfer);
    }
    if (error == ESP_OK) {
      error = gdma_get_free_m2m_trig_id_mask(
          s_s31_tx_scatter_ahb_tx, &free_mask);
    }
    if (error == ESP_OK && free_mask == 0u) {
      error = ESP_ERR_NOT_FOUND;
    }
    if (error == ESP_OK) {
      trigger.instance_id = __builtin_ctz(free_mask);
      error = gdma_connect(s_s31_tx_scatter_ahb_rx, trigger);
    }
    if (error == ESP_OK) {
      error = gdma_connect(s_s31_tx_scatter_ahb_tx, trigger);
    }
    if (error == ESP_OK) {
      error = gdma_get_channel_alignment_constraints(
          s_s31_tx_scatter_ahb_tx, &tx_alignment);
    }
    if (error == ESP_OK) {
      error = gdma_get_channel_alignment_constraints(
          s_s31_tx_scatter_ahb_rx, &rx_alignment);
    }
    if (error == ESP_OK) {
      error = gdma_get_channel_id(s_s31_tx_scatter_ahb_rx,
                                  &s_s31_tx_scatter_ahb_channel);
    }
    s_s31_tx_scatter_ahb_tx_ext_align = tx_alignment.ext_enc_mem_alignment;
  }
  if (error != ESP_OK) {
    ESP_LOGW("iq_capture", "TX scatter-AHB setup failed: %s",
             esp_err_to_name(error));
    return false;
  }
  if (s_s31_tx_scatter_dma_words == NULL) {
    s_s31_tx_scatter_dma_words = heap_caps_aligned_alloc(
        64u, S31_TX_SCATTER_PROBE_ITEMS * sizeof(uint32_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_s31_tx_scatter_dma_words == NULL) {
      return false;
    }
  }
  gdma_link_list_config_t tx_links = {
      .num_items = 4u,
      .item_alignment = 4u,
      .flags = {.items_in_ext_mem = true, .check_owner = true},
  };
  gdma_link_list_config_t rx_links = {
      .num_items = S31_TX_SCATTER_PROBE_ITEMS,
      .item_alignment = 4u,
      .flags = {.items_in_ext_mem = true, .check_owner = true},
  };
  if (s_s31_tx_scatter_ahb_tx_links == NULL) {
    error = gdma_new_link_list(&tx_links, &s_s31_tx_scatter_ahb_tx_links);
  }
  if (error == ESP_OK && s_s31_tx_scatter_ahb_rx_links == NULL) {
    error = gdma_new_link_list(&rx_links, &s_s31_tx_scatter_ahb_rx_links);
  }
  for (uint32_t word = 0u; error == ESP_OK && word < words; ++word) {
    s_s31_tx_scatter_dma_words[3u * word] =
        S31_ADCTRIG_TCM_DUMP_CTRL & ~BIT(24u + (word & 7u));
    s_s31_tx_scatter_dma_words[3u * word + 1u] =
        s_s31_tx_live_probe_words[word & 15u];
    s_s31_tx_scatter_dma_words[3u * word + 2u] =
        S31_ADCTRIG_TCM_DUMP_CTRL;
    gdma_buffer_mount_config_t destinations[3] = {
        {.buffer = (void *)(uintptr_t)HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
         .length = sizeof(uint32_t),
         .flags = {.bypass_buffer_addr_align_check = true}},
        {.buffer = (void *)(DUMP_BASE + word),
         .length = sizeof(uint32_t),
         .flags = {.bypass_buffer_addr_align_check = true}},
        {.buffer = (void *)(uintptr_t)HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
         .length = sizeof(uint32_t),
         .flags = {.mark_eof = word + 1u == words,
                   .mark_final = word + 1u == words
                                     ? GDMA_FINAL_LINK_TO_NULL
                                     : GDMA_FINAL_LINK_TO_DEFAULT,
                   .bypass_buffer_addr_align_check = true}},
    };
    error = gdma_link_mount_buffers(s_s31_tx_scatter_ahb_rx_links,
                                    3u * word, destinations, 3u, NULL);
  }
  const size_t transfer_bytes = 3u * words * sizeof(uint32_t);
  if (error == ESP_OK) {
    error = esp_cache_msync(s_s31_tx_scatter_dma_words, transfer_bytes,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                                ESP_CACHE_MSYNC_FLAG_UNALIGNED);
  }
  gdma_buffer_mount_config_t source = {
      .buffer = s_s31_tx_scatter_dma_words,
      .buffer_alignment = s_s31_tx_scatter_ahb_tx_ext_align,
      .length = transfer_bytes,
      .flags = {.mark_eof = true,
                .mark_final = GDMA_FINAL_LINK_TO_NULL,
                .bypass_buffer_addr_align_check = true},
  };
  if (error == ESP_OK) {
    error = gdma_link_mount_buffers(s_s31_tx_scatter_ahb_tx_links, 0,
                                    &source, 1u, NULL);
  }
  if (error != ESP_OK) {
    ESP_LOGW("iq_capture", "TX scatter-AHB descriptors failed: %s",
             esp_err_to_name(error));
    return false;
  }
  return true;
}

static bool s31_tx_scatter_ahb_run(uint32_t *elapsed_cycles,
                                   uint32_t *dma_status) {
  const uint32_t saved_start = AHB_DMA.intr_mem_start_addr.val;
  const uint32_t saved_end = AHB_DMA.intr_mem_end_addr.val;
  AHB_DMA.intr_mem_start_addr.val = 0x20000000u;
  AHB_DMA.intr_mem_end_addr.val = 0x53ffffffu;
  esp_err_t error = gdma_reset(s_s31_tx_scatter_ahb_tx);
  if (error == ESP_OK) {
    error = gdma_reset(s_s31_tx_scatter_ahb_rx);
  }
  const int channel = s_s31_tx_scatter_ahb_channel;
  AHB_DMA.in_intr[channel].clr.val = UINT32_MAX;
  AHB_DMA.out_intr[channel].clr.val = UINT32_MAX;
  const uint32_t started = esp_cpu_get_cycle_count();
  if (error == ESP_OK) {
    error = gdma_start(s_s31_tx_scatter_ahb_rx,
                       gdma_link_get_head_addr(
                           s_s31_tx_scatter_ahb_rx_links));
  }
  if (error == ESP_OK) {
    error = gdma_start(s_s31_tx_scatter_ahb_tx,
                       gdma_link_get_head_addr(
                           s_s31_tx_scatter_ahb_tx_links));
  }
  uint32_t status = 0u;
  while (error == ESP_OK &&
         ((status = AHB_DMA.in_intr[channel].raw.val) &
          (BIT(1) | BIT(3) | BIT(4) | BIT(7))) == 0u &&
         esp_cpu_get_cycle_count() - started < 30000000u) {
    __asm__ __volatile__("nop");
  }
  *elapsed_cycles = esp_cpu_get_cycle_count() - started;
  *dma_status = status;
  AHB_DMA.intr_mem_start_addr.val = saved_start;
  AHB_DMA.intr_mem_end_addr.val = saved_end;
  return error == ESP_OK && (status & BIT(1)) != 0u;
}

#endif
#if CONFIG_ESP_SDR_S31_TX_PIPELINED_REFILL
#define S31_TX_REPLAY_PIPELINE_WORDS 4096u
static async_memcpy_handle_t s_s31_tx_prefetch_dma;
static uint32_t *s_s31_tx_prefetch_stage;
static volatile bool s_s31_tx_prefetch_done;

static bool IRAM_ATTR s31_tx_prefetch_done_cb(
    async_memcpy_handle_t handle, async_memcpy_event_t *event, void *arg) {
  (void)handle;
  (void)event;
  (void)arg;
  s_s31_tx_prefetch_done = true;
  return false;
}

static bool s31_tx_prefetch_prepare(void) {
  if (s_s31_tx_prefetch_dma == NULL) {
    async_memcpy_config_t config = ASYNC_MEMCPY_DEFAULT_CONFIG();
    config.backlog = 1u;
    config.weight = 1u;
    config.dma_burst_size = 32u;
    esp_err_t error = esp_async_memcpy_install_gdma_axi(
        &config, &s_s31_tx_prefetch_dma);
    if (error != ESP_OK) {
      ESP_LOGW("iq_capture", "TX prefetch GDMA install failed: %s",
               esp_err_to_name(error));
      return false;
    }
  }
  if (s_s31_tx_prefetch_stage == NULL) {
    s_s31_tx_prefetch_stage = heap_caps_aligned_alloc(
        32u, S31_TX_REPLAY_PIPELINE_WORDS * sizeof(uint32_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_s31_tx_prefetch_stage == NULL) {
      ESP_LOGW("iq_capture",
               "TX prefetch stage allocation failed; largest internal=%zu",
               heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                                MALLOC_CAP_DMA |
                                                MALLOC_CAP_8BIT));
      return false;
    }
  }
  return true;
}

static bool s31_tx_prefetch_start(const uint32_t *source,
                                  uint32_t words) {
  size_t bytes = words * sizeof(uint32_t);
  (void)esp_cache_msync((void *)source, bytes,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                            ESP_CACHE_MSYNC_FLAG_UNALIGNED);
  s_s31_tx_prefetch_done = false;
  return esp_async_memcpy(s_s31_tx_prefetch_dma,
                          s_s31_tx_prefetch_stage, (void *)source, bytes,
                          s31_tx_prefetch_done_cb, NULL) == ESP_OK;
}

static bool s31_tx_prefetch_wait(uint32_t words) {
  int64_t deadline = esp_timer_get_time() + 100000;
  while (!s_s31_tx_prefetch_done && esp_timer_get_time() < deadline) {
    taskYIELD();
  }
  if (!s_s31_tx_prefetch_done) {
    return false;
  }
  (void)esp_cache_msync(
      s_s31_tx_prefetch_stage, words * sizeof(uint32_t),
      ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE |
          ESP_CACHE_MSYNC_FLAG_UNALIGNED);
  return true;
}

static bool s31_tx_replay_pipelined(uint32_t total_words,
                                    uint32_t *replay_end_ctrl,
                                    uint32_t *sample_cycles,
                                    uint32_t *gap_cycles_total,
                                    uint32_t *gap_cycles_max,
                                    uint64_t requested_start_time_ns,
                                    uint64_t *actual_start_time_ns) {
  bool dma_ok = true;
  uint32_t previous_done_cycle = 0u;
  uint32_t segment_count =
      (total_words + S31_TX_REPLAY_PIPELINE_WORDS - 1u) /
      S31_TX_REPLAY_PIPELINE_WORDS;
  for (uint32_t segment = 0u; segment < segment_count; ++segment) {
    uint32_t offset = segment * S31_TX_REPLAY_PIPELINE_WORDS;
    uint32_t remaining = total_words - offset;
    uint32_t segment_words = remaining > S31_TX_REPLAY_PIPELINE_WORDS
                                 ? S31_TX_REPLAY_PIPELINE_WORDS
                                 : remaining;
    uint32_t next_offset = offset + segment_words;
    uint32_t next_words = 0u;
    bool prefetch_started = false;
    if (next_offset < total_words) {
      next_words = total_words - next_offset;
      if (next_words > S31_TX_REPLAY_PIPELINE_WORDS) {
        next_words = S31_TX_REPLAY_PIPELINE_WORDS;
      }
      prefetch_started = s31_tx_prefetch_start(
          s_s31_tx_replay_upload + next_offset, next_words);
      dma_ok = dma_ok && prefetch_started;
    }

    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    uint32_t replay_ctrl =
        (segment_words & S31_TX_REPLAY_SIZE_M) |
        MODEM_WIFI_TOADCDUMP_ENABLE_BIT;
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                     S31_ADCTRIG_TCM_DUMP_CTRL);
    if (segment == 0u && requested_start_time_ns != 0u) {
      const int64_t requested_us =
          (int64_t)((requested_start_time_ns + 999u) / 1000u);
      while (esp_timer_get_time() < requested_us) {
        __asm__ __volatile__("nop");
      }
    }
    reg32p_write(&MODEM_WIFI_DUMP.MACTOADCDUMP0, replay_ctrl);
    if (segment == 0u) {
      *actual_start_time_ns = (uint64_t)esp_timer_get_time() * 1000u;
    }
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    uint32_t segment_start_cycle = esp_cpu_get_cycle_count();
    if (segment != 0u) {
      uint32_t gap_cycles = segment_start_cycle - previous_done_cycle;
      *gap_cycles_total += gap_cycles;
      if (gap_cycles > *gap_cycles_max) {
        *gap_cycles_max = gap_cycles;
      }
    }
    while ((reg32p_read(&MODEM_WIFI_DUMP.MACTOADCDUMP0) &
            S31_TX_REPLAY_DONE_BIT) == 0u &&
           esp_cpu_get_cycle_count() - segment_start_cycle < 20000000u) {
      __asm__ __volatile__("nop");
    }
    *replay_end_ctrl = reg32p_read(&MODEM_WIFI_DUMP.MACTOADCDUMP0);
    previous_done_cycle = esp_cpu_get_cycle_count();
    *sample_cycles += previous_done_cycle - segment_start_cycle;
    reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                      MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
    esp_ipc_isr_release_other_cpu();

    if (next_words != 0u) {
      bool ready = prefetch_started && s31_tx_prefetch_wait(next_words);
      dma_ok = dma_ok && ready;
      const uint32_t *source = ready ? s_s31_tx_prefetch_stage
                                     : s_s31_tx_replay_upload + next_offset;
      memcpy((void *)DUMP_BASE, source, next_words * sizeof(uint32_t));
      __asm__ __volatile__("fence rw, rw" ::: "memory");
    }
  }
  return dma_ok;
}
#endif

static inline bool s31_tx_replay_selected(const capture_config_t *config) {
  return config->tx.tx_tone_enable == S31_TX_REPLAY_MODE;
}

static uint32_t s31_tx_replay_fill(uint32_t pattern) {
  /* Two deliberately different candidate layouts let OTA measurements
   * identify the replay formatter without relying on the vendor's single
   * sawtooth.  Pattern 0 exactly reproduces the final byte contents made by
   * librftest dactrig(): eight copies of every signed 8-bit ramp value.
   * Pattern 1 is interleaved signed-I/signed-Q, four samples per turn. */
  static const int8_t iq16[16][2] = {
      {96, 0},   {89, 37},   {68, 68},   {37, 89},
      {0, 96},   {-37, 89},  {-68, 68},  {-89, 37},
      {-96, 0},  {-89, -37}, {-68, -68}, {-37, -89},
      {0, -96},  {37, -89},  {68, -68},  {89, -37},
  };
  /* One deterministic 64-point OFDM symbol.  QPSK occupies bins -20..-2
   * and +2..+20; DC and the adjacent bins are intentionally empty.  This is
   * precomputed on the host and only copied/packed by the MCU. */
  static const int16_t ofdm64[64][2] = {
      {0, 34},     {-9, 87},    {74, 48},    {165, -122},
      {147, -237}, {6, -110},   {-57, 25},   {-2, -42},
      {-10, -92},  {-38, -10},  {11, 35},    {2, 13},
      {-61, -4},   {-26, -36},  {10, -45},   {-38, -4},
      {-34, 0},    {12, -36},   {-14, -30},  {-30, 10},
      {7, 36},     {6, 25},     {4, -12},    {48, -4},
      {38, 24},    {-47, -13},  {-82, -26},  {9, 28},
      {147, -4},   {163, -30},  {54, 136},   {-12, 240},
      {-68, 102},  {-197, 16},  {-159, 0},   {67, -173},
      {85, -227},  {-40, 47},   {56, 211},   {113, 73},
      {-58, -44},  {-20, -68},  {126, -83},  {-118, -15},
      {-307, 44},  {20, 9},     {210, 7},    {-22, -67},
      {-34, -272}, {151, -215}, {3, 78},     {-172, 64},
      {33, -116},  {250, 8},    {173, 144},  {-10, 17},
      {-106, -24}, {-43, 47},   {42, -22},   {-114, 3},
      {-323, 236}, {-186, 279}, {93, 77},    {116, -21},
  };

  uint32_t upload_words = 0u;
  if (pattern == 3u) {
    taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
    upload_words = s_s31_tx_replay_upload_words;
    uint32_t first_segment_words = upload_words > S31_TX_REPLAY_SIZE_M
                                       ? S31_TX_REPLAY_REFILL_WORDS
                                       : upload_words;
    /* The HTTP ingress rejects every non-IQ10 upper bit, so replay copies do
     * not need a second scalar validation pass. */
    s31_tx_replay_copy((void *)DUMP_BASE, s_s31_tx_replay_upload,
                       first_segment_words);
    taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
  } else if (pattern == 0u) {
    volatile uint8_t *bytes = (volatile uint8_t *)DUMP_BASE;
    for (uint32_t i = 0u; i < S31_TX_REPLAY_WORDS * sizeof(uint32_t); ++i) {
      bytes[i] = (uint8_t)(i >> 3u);
    }
  } else if (pattern == 11u) {
    /* Zero baseband used by the downstream digital-TXDC modulation probe.
     * Any non-DC RF energy in this mode is therefore produced by live writes
     * to TXDC_DIGITAL rather than by the replay symbol itself. */
    for (uint32_t word = 0u; word < 1024u; ++word) {
      DUMP_BASE[word] = 0u;
    }
  } else if (pattern == 4u || pattern == 6u || pattern == 7u ||
             pattern == 8u) {
    /* Keep the second marker below the ROM-reserved top of TCM.  If an
     * undocumented replay bit redirects the reader by 64 KiB, RF changes
     * from +1.25 MHz to -1.25 MHz without changing the configured length. */
    enum { ADDRESS_PROBE_WORDS = 1024u };
    for (uint32_t word = 0u; word < ADDRESS_PROBE_WORDS; ++word) {
      const uint32_t phase = word & 15u;
      const int32_t i = (int32_t)iq16[phase][0] << 2u;
      const int32_t q = (int32_t)iq16[phase][1] << 2u;
      DUMP_BASE[word] = ((uint32_t)i & 0x3ffu) |
                        (((uint32_t)q & 0x3ffu) << 10u);
      volatile uint32_t *second_marker =
          DUMP_BASE + S31_TX_REPLAY_WORDS;
      second_marker[word] = ((uint32_t)i & 0x3ffu) |
                            (((uint32_t)(-q) & 0x3ffu) << 10u);
    }
  } else if (pattern == 1u || pattern == 5u || pattern == 9u ||
             pattern == 10u || pattern == 12u || pattern == 13u ||
             pattern == 14u || pattern == 15u) {
    for (uint32_t word = 0u; word < S31_TX_REPLAY_WORDS; ++word) {
      uint32_t phase = word & 15u;
      int32_t i = (int32_t)iq16[phase][0] << 2u;
      int32_t q = (int32_t)iq16[phase][1] << 2u;
      if (pattern == 5u) {
        q = -q;
      }
      /* The modem's native complex word uses the same compact signed format
       * as its receive diagnostic words: I[9:0], Q[19:10]. */
      DUMP_BASE[word] = ((uint32_t)i & 0x3ffu) |
                        (((uint32_t)q & 0x3ffu) << 10u);
      if ((pattern == 10u || pattern == 12u || pattern == 13u ||
           pattern == 14u) &&
          word < 16u) {
        s_s31_tx_live_probe_words[word] =
            ((uint32_t)i & 0x3ffu) |
            (((uint32_t)(-q) & 0x3ffu) << 10u);
      }
    }
  } else if (pattern == 2u) {
    for (uint32_t word = 0u; word < S31_TX_REPLAY_WORDS; ++word) {
      int32_t i = ofdm64[word & 63u][0];
      int32_t q = ofdm64[word & 63u][1];
      DUMP_BASE[word] = ((uint32_t)i & 0x3ffu) |
                        (((uint32_t)q & 0x3ffu) << 10u);
    }
  } else {
    for (uint32_t word = 0u; word < S31_TX_REPLAY_WORDS; ++word) {
      DUMP_BASE[word] = 0u;
    }
  }
  __asm__ __volatile__("fence rw, rw" ::: "memory");
  if (upload_words != 0u) {
    return upload_words > S31_TX_REPLAY_SIZE_M ? S31_TX_REPLAY_REFILL_WORDS
                                                : upload_words;
  }
  if (pattern == 10u || pattern == 12u || pattern == 13u ||
      pattern == 14u) {
    return 1024u;
  }
  if (pattern == 1u || pattern == 5u || pattern == 9u || pattern == 15u) {
    return 16u;
  }
  if (pattern == 2u) {
    return 64u;
  }
  if (pattern == 4u || pattern == 6u || pattern == 7u || pattern == 8u) {
    return 1024u;
  }
  return S31_TX_REPLAY_SIZE_M;
}

static uint32_t s31_tx_replay_address_probe_extra(uint32_t step) {
  if ((step & S31_TX_REPLAY_ADDRESS_PROBE) == 0u) {
    return 0u;
  }
  const uint32_t selector =
      (step & S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M) >>
      S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_S;
  /* Bit 26 is value 0x40 in the observed [27:20] repeat-count field.  Keep
   * every address candidate on air for about 3.3 ms at 20 MSa/s so a valid
   * alternate marker cannot be missed merely because count zero is one-shot. */
  const uint32_t repeat_count = BIT(26);
  if (selector == 0u) {
    return repeat_count;
  }
  /* selector 1..4 -> bits 14..17; selector 5..15 -> bits 20..30.
   * Deliberately skip the known DONE, LOOP, and ENABLE bits. */
  return repeat_count |
         (selector <= 4u ? BIT(13u + selector) : BIT(15u + selector));
}

static uint32_t s31_tx_replay_gate_probe_mask(uint32_t step) {
  (void)step;
  /* The lower ownership bytes cover live system SRAM and can survive a CPU
   * reset, leaving the application in a boot loop.  Keep experimental replay
   * requests confined to the verified modem aperture. */
  return S31_ADCTRIG_TCM_DUMP_CTRL;
}

static void s31_tx_replay_publish_missed(uint32_t words, uint32_t rate_code,
                                         uint64_t requested_start_time_ns) {
  s_s31_tx_replay_diag = (iq_tx_replay_diag_t){
      .words = words,
      .rate_code = rate_code,
      .segments = 0u,
      .requested_start_time_ns = requested_start_time_ns,
      .deadline_missed = true,
  };
  s_s31_tx_replay_burst_complete = true;
  ESP_LOGW("iq_capture",
           "TX replay deadline missed before RF preparation: words=%" PRIu32
           " rate_code=%" PRIu32 " requested=%" PRIu64,
           words, rate_code, requested_start_time_ns);
}

/* A large full-speed PSRAM-to-DIRAM memcpy can monopolize the shared cache and
 * memory fabric even though it targets an inactive slot. The unpacked fallback
 * therefore copies eight words every 160 cycles (64 MB/s). Production 5 MSa/s
 * USB uses the phase-synchronous packed-IQ10 decoder below. */
static void IRAM_ATTR __attribute__((noinline, optimize("O3")))
s31_txdc_stage_copy(uint32_t *destination, const uint32_t *source,
                    uint32_t words) {
  enum { WORDS_PER_BURST = 8u, BURST_PERIOD_CYCLES = 160u };
  uint32_t deadline = esp_cpu_get_cycle_count();
  uint32_t copied = 0u;
  while (words - copied >= WORDS_PER_BURST) {
    const uint32_t v0 = source[copied + 0u];
    const uint32_t v1 = source[copied + 1u];
    const uint32_t v2 = source[copied + 2u];
    const uint32_t v3 = source[copied + 3u];
    const uint32_t v4 = source[copied + 4u];
    const uint32_t v5 = source[copied + 5u];
    const uint32_t v6 = source[copied + 6u];
    const uint32_t v7 = source[copied + 7u];
    destination[copied + 0u] = v0 & 0x000fffffu;
    destination[copied + 1u] = v1 & 0x000fffffu;
    destination[copied + 2u] = v2 & 0x000fffffu;
    destination[copied + 3u] = v3 & 0x000fffffu;
    destination[copied + 4u] = v4 & 0x000fffffu;
    destination[copied + 5u] = v5 & 0x000fffffu;
    destination[copied + 6u] = v6 & 0x000fffffu;
    destination[copied + 7u] = v7 & 0x000fffffu;
    copied += WORDS_PER_BURST;
    deadline += BURST_PERIOD_CYCLES;
    while ((int32_t)(esp_cpu_get_cycle_count() - deadline) < 0) {
      __asm__ __volatile__("nop");
    }
  }
  while (copied < words) {
    destination[copied] = source[copied] & 0x000fffffu;
    ++copied;
  }
}

static inline uint32_t IRAM_ATTR
s31_txdc_unpack20_word(const uint8_t *source, uint32_t word_index) {
  /* The five-byte pair alternates between aligned and odd addresses. Keep
   * these accesses explicitly byte-wide: GCC otherwise folds bytes 0 and 1
   * into an unaligned halfword load, which takes the S31 exception path and
   * is far too expensive inside the sample deadline loop. */
  const volatile uint8_t *pair = source + (word_index >> 1u) * 5u;
  if ((word_index & 1u) == 0u) {
    return (uint32_t)pair[0] | ((uint32_t)pair[1] << 8u) |
           (((uint32_t)pair[2] & 0x0fu) << 16u);
  }
  return ((uint32_t)pair[2] >> 4u) | ((uint32_t)pair[3] << 4u) |
         ((uint32_t)pair[4] << 12u);
}

static inline void IRAM_ATTR s31_txdc_unpack20_pair(const uint8_t *source,
                                                    uint32_t *first,
                                                    uint32_t *second) {
  const volatile uint8_t *bytes = source;
  const uint32_t b0 = bytes[0];
  const uint32_t b1 = bytes[1];
  const uint32_t b2 = bytes[2];
  const uint32_t b3 = bytes[3];
  const uint32_t b4 = bytes[4];
  *first = b0 | (b1 << 8u) | ((b2 & 0x0fu) << 16u);
  *second = (b2 >> 4u) | (b3 << 4u) | (b4 << 12u);
}

static inline uint32_t IRAM_ATTR
s31_txdc_unpack16_word(const uint8_t *source, uint32_t word_index) {
  const volatile int8_t *sample =
      (const volatile int8_t *)source + word_index * 2u;
  const int32_t i = (int32_t)sample[0] * 4;
  const int32_t q = (int32_t)sample[1] * 4;
  return ((uint32_t)i & 0x3ffu) | (((uint32_t)q & 0x3ffu) << 10u);
}

/* Ethernet's high-rate IQ8 wire mode trades two converter bits for enough
 * transport margin at 4 and 10/3 MSa/s on a shared LAN. Decode eight samples
 * in one phase-separated refill burst; the modem still receives IQ10 words. */
static void IRAM_ATTR __attribute__((noinline, optimize("O3")))
s31_txdc_stage_unpack16(uint32_t *destination, const uint8_t *source,
                        uint32_t source_offset, uint32_t words) {
  uint32_t copied = 0u;
  if (__atomic_load_n(&s_s31_txdc_rf_timing_active, __ATOMIC_ACQUIRE)) {
    const uint32_t origin = __atomic_load_n(&s_s31_txdc_rf_start_cycle,
                                            __ATOMIC_RELAXED);
    const uint32_t period = __atomic_load_n(
        &s_s31_txdc_rf_cycles_per_sample, __ATOMIC_RELAXED);
    uint32_t now = esp_cpu_get_cycle_count();
    uint32_t deadline =
        now + ((period / 2u + period - ((now - origin) % period)) % period);
    while (words - copied >= 8u) {
      const uint32_t base = source_offset + copied;
      const uint32_t v0 = s31_txdc_unpack16_word(source, base + 0u);
      const uint32_t v1 = s31_txdc_unpack16_word(source, base + 1u);
      const uint32_t v2 = s31_txdc_unpack16_word(source, base + 2u);
      const uint32_t v3 = s31_txdc_unpack16_word(source, base + 3u);
      const uint32_t v4 = s31_txdc_unpack16_word(source, base + 4u);
      const uint32_t v5 = s31_txdc_unpack16_word(source, base + 5u);
      const uint32_t v6 = s31_txdc_unpack16_word(source, base + 6u);
      const uint32_t v7 = s31_txdc_unpack16_word(source, base + 7u);
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      destination[copied + 0u] = v0;
      destination[copied + 1u] = v1;
      destination[copied + 2u] = v2;
      destination[copied + 3u] = v3;
      destination[copied + 4u] = v4;
      destination[copied + 5u] = v5;
      destination[copied + 6u] = v6;
      destination[copied + 7u] = v7;
      copied += 8u;
      deadline += period;
    }
    while (copied < words) {
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      destination[copied] =
          s31_txdc_unpack16_word(source, source_offset + copied);
      ++copied;
    }
    return;
  }
  enum { WORDS_PER_BURST = 8u, BURST_PERIOD_CYCLES = 160u };
  uint32_t deadline = esp_cpu_get_cycle_count();
  while (words - copied >= WORDS_PER_BURST) {
    for (uint32_t i = 0u; i < WORDS_PER_BURST; ++i) {
      destination[copied + i] =
          s31_txdc_unpack16_word(source, source_offset + copied + i);
    }
    copied += WORDS_PER_BURST;
    deadline += BURST_PERIOD_CYCLES;
    while ((int32_t)(esp_cpu_get_cycle_count() - deadline) < 0) {
      __asm__ __volatile__("nop");
    }
  }
  while (copied < words) {
    destination[copied] =
        s31_txdc_unpack16_word(source, source_offset + copied);
    ++copied;
  }
}

/* Decode packed USB IQ10 directly into an inactive ordinary-DIRAM slot; no
 * complete 32-bit host batch is ever expanded in PSRAM. */
static void IRAM_ATTR __attribute__((noinline, optimize("O3")))
s31_txdc_stage_unpack20(uint32_t *destination, const uint8_t *source,
                        uint32_t source_offset, uint32_t words) {
  /* Once RF is live, place four DIRAM writes in the middle of every
   * sample period. The emitter fetches its next word immediately
   * after the previous TXDC MMIO write, near the start of that period. This
   * phase separation prevents the second core's refill from continuously
   * stealing the reader's small ten-cycle timing margin while staging at
   * 20 Mword/s, four times the consumption rate. The higher staging rate
   * leaves roughly 75% of core 0 available to service native USB. */
  if (__atomic_load_n(&s_s31_txdc_rf_timing_active, __ATOMIC_ACQUIRE)) {
    const uint32_t origin = __atomic_load_n(&s_s31_txdc_rf_start_cycle,
                                            __ATOMIC_RELAXED);
    const uint32_t period = __atomic_load_n(
        &s_s31_txdc_rf_cycles_per_sample, __ATOMIC_RELAXED);
    uint32_t copied = 0u;
    uint32_t now = esp_cpu_get_cycle_count();
    uint32_t deadline =
        now + ((period / 2u + period - ((now - origin) % period)) % period);
    if (!s_s31_txdc_tcm_slots_borrowed) {
      /* Ethernet keeps GMAC live and therefore uses the small static DIRAM
       * ring. Decode eight words per RF period so the stager occupies roughly
       * one eighth of core 0 rather than one quarter, leaving enough CPU for
       * the packed UDP receiver and its per-batch arm request. Native USB
       * retains the proven four-word phasing below. */
      while (words - copied >= 8u) {
        const uint32_t base = source_offset + copied;
        uint32_t v0, v1, v2, v3, v4, v5, v6, v7;
        s31_txdc_unpack20_pair(source + (base >> 1u) * 5u, &v0, &v1);
        s31_txdc_unpack20_pair(source + ((base + 2u) >> 1u) * 5u, &v2, &v3);
        s31_txdc_unpack20_pair(source + ((base + 4u) >> 1u) * 5u, &v4, &v5);
        s31_txdc_unpack20_pair(source + ((base + 6u) >> 1u) * 5u, &v6, &v7);
        do {
          now = esp_cpu_get_cycle_count();
        } while ((int32_t)(now - deadline) < 0);
        destination[copied + 0u] = v0;
        destination[copied + 1u] = v1;
        destination[copied + 2u] = v2;
        destination[copied + 3u] = v3;
        destination[copied + 4u] = v4;
        destination[copied + 5u] = v5;
        destination[copied + 6u] = v6;
        destination[copied + 7u] = v7;
        copied += 8u;
        deadline += period;
      }
    }
    while (words - copied >= 4u) {
      const uint32_t base = source_offset + copied; uint32_t v0, v1, v2, v3;
      s31_txdc_unpack20_pair(source + (base >> 1u) * 5u, &v0, &v1);
      s31_txdc_unpack20_pair(source +(( base + 2u) >> 1u) * 5u, &v2, & v3);
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      destination[copied] = v0;
      destination[copied + 1u] = v1;
      destination[copied + 2u] = v2;
      destination[copied + 3u] = v3;
      copied += 4u;
      deadline += period;
    }
    while (copied < words) {
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      destination[copied] =
          s31_txdc_unpack20_word(source, source_offset + copied);
      ++copied;
    }
    return;
  }
  enum { WORDS_PER_BURST = 8u, BURST_PERIOD_CYCLES = 160u };
  uint32_t deadline = esp_cpu_get_cycle_count();
  uint32_t copied = 0u;
  while (words - copied >= WORDS_PER_BURST) {
    const uint32_t base = source_offset + copied;
    const uint32_t v0 = s31_txdc_unpack20_word(source, base + 0u);
    const uint32_t v1 = s31_txdc_unpack20_word(source, base + 1u);
    const uint32_t v2 = s31_txdc_unpack20_word(source, base + 2u);
    const uint32_t v3 = s31_txdc_unpack20_word(source, base + 3u);
    const uint32_t v4 = s31_txdc_unpack20_word(source, base + 4u);
    const uint32_t v5 = s31_txdc_unpack20_word(source, base + 5u);
    const uint32_t v6 = s31_txdc_unpack20_word(source, base + 6u);
    const uint32_t v7 = s31_txdc_unpack20_word(source, base + 7u);
    destination[copied + 0u] = v0;
    destination[copied + 1u] = v1;
    destination[copied + 2u] = v2;
    destination[copied + 3u] = v3;
    destination[copied + 4u] = v4;
    destination[copied + 5u] = v5;
    destination[copied + 6u] = v6;
    destination[copied + 7u] = v7;
    copied += WORDS_PER_BURST;
    deadline += BURST_PERIOD_CYCLES;
    while ((int32_t)(esp_cpu_get_cycle_count() - deadline) < 0) {
      __asm__ __volatile__("nop");
    }
  }
  while (copied < words) {
    destination[copied] =
        s31_txdc_unpack20_word(source, source_offset + copied);
    ++copied;
  }
}

static void s31_txdc_stage_copy_tracked(uint32_t *destination,
                                        const uint32_t *upload,
                                        uint32_t source_offset,
                                        uint32_t words,
                                        uint16_t packed_flags) {
  if (words == 0u) {
    return;
  }
  const uint32_t copy_start = esp_cpu_get_cycle_count();
  if ((packed_flags & IQ_TX_UDP_FLAG_PACKED16) != 0u) {
    s31_txdc_stage_unpack16(destination, (const uint8_t *)upload,
                            source_offset, words);
  } else if ((packed_flags & IQ_TX_UDP_FLAG_PACKED20) != 0u) {
    s31_txdc_stage_unpack20(destination, (const uint8_t *)upload,
                            source_offset, words);
  } else {
    s31_txdc_stage_copy(destination, upload + source_offset, words);
  }
  const uint32_t copy_cycles = esp_cpu_get_cycle_count() - copy_start;
  if (copy_cycles > s_s31_txdc_stager_copy_max_cycles) {
    s_s31_txdc_stager_copy_max_cycles = copy_cycles;
  }
}

static void s31_txdc_tcm_stager_task(void *arg) {
  (void)arg;
  while (true) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!__atomic_exchange_n(&s_s31_txdc_stager_start, false,
                             __ATOMIC_ACQ_REL)) {
      continue;
    }

    uint32_t *upload = s_s31_txdc_stager_initial_upload;
    uint32_t word_count = s_s31_txdc_stager_initial_words;
    uint16_t commit_flags = s_s31_txdc_stager_initial_flags;
    uint32_t slot_index = 0u;
    const uint32_t slot_count = s_s31_txdc_tcm_slot_count;
    s_s31_txdc_stager_copy_max_cycles = 0u;
    __atomic_store_n(&s_s31_txdc_stager_done, false, __ATOMIC_RELEASE);

    while (!__atomic_load_n(&s_s31_txdc_stager_stop, __ATOMIC_ACQUIRE) &&
           upload != NULL && word_count != 0u) {
      uint32_t offset = 0u;
      while (offset < word_count &&
             !__atomic_load_n(&s_s31_txdc_stager_stop, __ATOMIC_ACQUIRE)) {
        s31_txdc_tcm_slot_t *slot = s_s31_txdc_tcm_slots[slot_index];
        while (__atomic_load_n(&slot->ready, __ATOMIC_RELAXED) != 0u &&
               !__atomic_load_n(&s_s31_txdc_stager_stop,
                                __ATOMIC_ACQUIRE)) {
          if (s_s31_txdc_tcm_slots_borrowed) {
            /* TinyUSB and this stager are equal-priority core-0 peers. USB's
             * scatter ring is large enough that a yield is both responsive
             * and cheap while DWC2 is runnable. */
          taskYIELD();
          } else {
            /* The CPU0 timer ISR bounds this sleep to 1.4 ms and immediately
             * preempts priority-23 network work when a refill is due. */
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          }
        }
        if (__atomic_load_n(&s_s31_txdc_stager_stop, __ATOMIC_ACQUIRE)) {
          break;
        }
        uint32_t words = word_count - offset;
        const uint32_t slot_capacity =
            s_s31_txdc_tcm_slot_capacity[slot_index];
        if (words > slot_capacity) {
          words = slot_capacity;
        }

        uint32_t *destination =
            (uint32_t *)s_s31_txdc_tcm_slot_base[slot_index];
        const uint16_t packed_flags =
            commit_flags & IQ_TX_UDP_FLAGS_PACKED;
        s31_txdc_stage_copy_tracked(destination, upload, offset, words,
                                    packed_flags);
        offset += words;
        slot->words = words;
        slot->final = offset == word_count &&
                      (commit_flags & IQ_TX_UDP_FLAG_MORE) == 0u;
        __atomic_store_n(&slot->ready, 1u, __ATOMIC_RELEASE);
        slot_index = (slot_index + 1u) % slot_count;
      }

      s31_txdc_release_upload(upload);
      upload = NULL;
      if (__atomic_load_n(&s_s31_txdc_stager_stop, __ATOMIC_ACQUIRE)) {
        break;
      }
      if ((commit_flags & IQ_TX_UDP_FLAG_MORE) == 0u) {
        break;
      }

      const int64_t continuation_deadline_us =
          esp_timer_get_time() +
          S31_TXDC_CONTINUATION_WAIT_CYCLES /
              (esp_clk_cpu_freq() / 1000000u);
      while ((upload = s31_txdc_continuation_pop(&word_count,
                                                 &commit_flags)) == NULL &&
             !__atomic_load_n(&s_s31_txdc_stager_stop,
                              __ATOMIC_ACQUIRE) &&
             esp_timer_get_time() < continuation_deadline_us) {
        vTaskDelay(1);
      }
      if (upload == NULL) {
        break;
      }
    }

    s31_txdc_release_upload(upload);
    __atomic_store_n(&s_s31_txdc_stager_done, true, __ATOMIC_RELEASE);
  }
}

typedef struct {
  uint32_t total_words;
  uint32_t segment_count;
  uint32_t gap_cycles_total;
  uint32_t gap_cycles_max;
  uint32_t maximum_lateness;
  uint32_t maximum_lateness_word;
  bool stream_ok;
} s31_txdc_tcm_emit_result_t;

/* The ordinary deadline loop deliberately measures every sample. At the
 * 60-cycle point that one diagnostic comparison consumes the remaining
 * budget. Keep the absolute deadline, but use a separately compiled lean loop
 * between descriptor checkpoints; the caller still measures the first sample
 * and every slot boundary. */
static uint32_t IRAM_ATTR __attribute__((noinline, optimize("O3")))
s31_txdc_emit_run60(const uint32_t *data, uint32_t first, uint32_t end,
                    uint32_t txdc_control, uint32_t deadline) {
  const uint32_t *cursor = data + first;
  const uint32_t *const limit = data + end;
  /* Two samples per loop amortizes the pointer comparison and backwards
   * branch.  The second word is fetched before either deadline wait, which
   * also keeps the staging-memory load off the narrow path into TXDC. */
  while (cursor + 1 < limit) {
    const uint32_t value0 = cursor[0];
    const uint32_t value1 = cursor[1];
    cursor += 2;
    while ((int32_t)(esp_cpu_get_cycle_count() - deadline) < 0) {
      __asm__ __volatile__("nop");
    }
    MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_control | value0;
    deadline += 60u;
    while ((int32_t)(esp_cpu_get_cycle_count() - deadline) < 0) {
      __asm__ __volatile__("nop");
    }
    MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_control | value1;
    deadline += 60u;
  }
  while (cursor < limit) {
    const uint32_t value = *cursor++;
    while ((int32_t)(esp_cpu_get_cycle_count() - deadline) < 0) {
      __asm__ __volatile__("nop");
    }
    MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_control | value;
    deadline += 60u;
  }
  __asm__ __volatile__("" : "+r"(deadline));
  return deadline;
}

/* Keep only the cycle-critical TCM reader in IRAM. Marking the much larger
 * control function IRAM_ATTR was insufficient because GCC inlined it back
 * into its flash-resident caller; whole-function O3, on the other hand,
 * consumed enough IRAM to starve PARLIO's internal DMA allocations. This
 * small outlined helper keeps its deadline state in registers and cannot be
 * interrupted by a flash/PSRAM XIP cache miss while RF is active. */
static void IRAM_ATTR __attribute__((noinline, optimize("O3")))
s31_txdc_emit_tcm_slots(uint32_t txdc_control, uint32_t cycles_per_sample,
                       uint32_t start_cycle,
                       s31_txdc_tcm_emit_result_t *result) {
  uint32_t total_words = 0u;
  uint32_t segment_count = 0u;
  uint32_t gap_cycles_total = 0u;
  uint32_t gap_cycles_max = 0u;
  uint32_t maximum_lateness = 0u;
  uint32_t maximum_lateness_word = 0u;
  bool stream_ok = true;
  uint32_t deadline = start_cycle;
  uint32_t next_iwdt_heartbeat_word = S31_TXDC_IWDT_FEED_WORDS;
  uint32_t slot_index = 0u;
  const uint32_t slot_count = s_s31_txdc_tcm_slot_count;
  uint32_t slot_words = s_s31_txdc_tcm_slots[0]->words;
  bool final_slot = s_s31_txdc_tcm_slots[0]->final;

  while (true) {
    s31_txdc_tcm_slot_t *slot = s_s31_txdc_tcm_slots[slot_index];
    const uint32_t *slot_data =
        (const uint32_t *)s_s31_txdc_tcm_slot_base[slot_index];
    uint32_t sample = 0u;
    if (slot_words != 0u) {
      const uint32_t value = slot_data[0];
      uint32_t now;
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      const uint32_t late = now - deadline;
      if (late > maximum_lateness) {
        maximum_lateness = late;
        maximum_lateness_word = total_words;
      }
      if (segment_count != 0u && late > 0u) {
        gap_cycles_total += late;
        if (late > gap_cycles_max) {
          gap_cycles_max = late;
        }
      }
      MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_control | value;
      deadline += cycles_per_sample;
      sample = 1u;
    }
    ++segment_count;
    const uint32_t previous_total = total_words;
    const uint32_t slot_end_total = previous_total + slot_words;
    /* Announce an upcoming IWDT feed one complete staging slot before its
     * sample threshold, while retaining exactly one feed per 524,288
     * samples. */
    if (slot_end_total + S31_TXDC_TCM_SLOT_WORDS >=
        next_iwdt_heartbeat_word) {
      __atomic_add_fetch(&s_s31_txdc_iwdt_heartbeat, 1u,
                         __ATOMIC_RELAXED);
      next_iwdt_heartbeat_word += S31_TXDC_IWDT_FEED_WORDS;
    }
    bool next_cached = final_slot;
    uint32_t next_words = 0u;
    bool next_final = false;
    /* Cache the next descriptor away from the exact seam. The phase-locked
     * stager normally keeps both successor slots ready, and placing this
     * shared-cache-line read at seven eighths avoids a boundary-time refill. */
    const uint32_t prefetch_sample = (slot_words * 7u) / 8u;
    if (cycles_per_sample == 60u) {
      deadline = s31_txdc_emit_run60(slot_data, sample, prefetch_sample,
                                     txdc_control, deadline);
      sample = prefetch_sample;
    }
    for (; sample < prefetch_sample; ++sample) {
      const uint32_t value = slot_data[sample];
      uint32_t now;
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      const uint32_t late = now - deadline;
      if (late > maximum_lateness) {
        maximum_lateness = late;
        maximum_lateness_word = total_words + sample;
      }
      MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_control | value;
      deadline += cycles_per_sample;
    }
    if (!next_cached) {
      const uint32_t next_index =
          (slot_index + 1u) % slot_count;
      s31_txdc_tcm_slot_t *next = s_s31_txdc_tcm_slots[next_index];
      if (__atomic_load_n(&next->ready, __ATOMIC_ACQUIRE) != 0u) {
        next_words = next->words;
        next_final = next->final;
        next_cached = true;
      }
    }
    total_words = slot_end_total;
    if (cycles_per_sample == 60u) {
      deadline = s31_txdc_emit_run60(slot_data, sample, slot_words,
                                     txdc_control, deadline);
      sample = slot_words;
    }
    for (; sample < slot_words; ++sample) {
      const uint32_t value = slot_data[sample];
      uint32_t now;
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      const uint32_t late = now - deadline;
      if (late > maximum_lateness) {
        maximum_lateness = late;
        maximum_lateness_word = previous_total + sample;
      }
      MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_control | value;
      deadline += cycles_per_sample;
    }
    __atomic_store_n(&slot->ready, 0u, __ATOMIC_RELAXED);
    if (final_slot) {
      break;
    }
    slot_index = (slot_index + 1u) % slot_count;
    s31_txdc_tcm_slot_t *next = s_s31_txdc_tcm_slots[slot_index];
    if (!next_cached) {
      const uint32_t wait_start = esp_cpu_get_cycle_count();
      while (__atomic_load_n(&next->ready, __ATOMIC_ACQUIRE) == 0u &&
             !__atomic_load_n(&s_s31_txdc_stager_done,
                              __ATOMIC_ACQUIRE) &&
             esp_cpu_get_cycle_count() - wait_start <
                 S31_TXDC_CONTINUATION_WAIT_CYCLES) {
        if ((esp_cpu_get_cycle_count() & 0x3fffffu) == 0u) {
          __atomic_add_fetch(&s_s31_txdc_iwdt_heartbeat, 1u,
                             __ATOMIC_RELAXED);
        }
        __asm__ __volatile__("nop");
      }
      if (__atomic_load_n(&next->ready, __ATOMIC_ACQUIRE) == 0u) {
        stream_ok = false;
        break;
      }
      next_words = next->words;
      next_final = next->final;
    }
    slot_words = next_words;
    final_slot = next_final;
  }

  *result = (s31_txdc_tcm_emit_result_t){
      .total_words = total_words,
      .segment_count = segment_count,
      .gap_cycles_total = gap_cycles_total,
      .gap_cycles_max = gap_cycles_max,
      .maximum_lateness = maximum_lateness,
      .maximum_lateness_word = maximum_lateness_word,
      .stream_ok = stream_ok,
  };
}

/* TXDC_DIGITAL is sampled by the live transmit front end independently of
 * the modem replay reader. A direct MMIO update costs about 54 cycles with
 * the transport core active. Derive pacing from the actual CPU clock: S31
 * production builds run at 320 MHz (not the early probe's assumed 300 MHz),
 * giving 60 cycles at the fastest DIRAM-staged mode, 64 cycles at 5 MSa/s,
 * 72 cycles at 40/9 MSa/s, 80 cycles at 4 MSa/s, or 96 cycles at
 * 3.333 MSa/s. */
static void IRAM_ATTR s31_txdc_emit_uploaded(
    uint32_t word_count, uint32_t rate_code, uint16_t commit_flags,
    uint64_t requested_start_time_ns) {
  const uint32_t target_sample_rate_hz =
      rate_code == 11u ? 40000000u / 9u
      : rate_code == 9u ? 4000000u
      : rate_code == 10u ? 10000000u / 3u
      : rate_code == 12u ? 2500000u
      : rate_code == 13u ? 2000000u
      : rate_code == 15u ? 320000000u / 60u
                         : 5000000u;
  const uint32_t cpu_frequency_hz = (uint32_t)esp_clk_cpu_freq();
  const uint32_t cycles_per_sample =
      (cpu_frequency_hz + target_sample_rate_hz / 2u) /
      target_sample_rate_hz;
  uint32_t *upload;
  taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
  upload = s_s31_tx_replay_upload;
  taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);

  if (upload == NULL || word_count == 0u) {
    modem_stop_tx_replay();
    s31_tx_replay_publish_missed(word_count, rate_code,
                                 requested_start_time_ns);
    return;
  }
  const bool tcm_staged_stream = rate_code == 14u || rate_code == 15u ||
                                 (commit_flags & IQ_TX_UDP_FLAGS_PACKED) != 0u;
  if (tcm_staged_stream &&
      (commit_flags & IQ_TX_UDP_FLAG_MORE) != 0u) {
    /* Packed IQ10 over HS USB runs close to the RF consumption rate, while
     * shared-LAN Ethernet has occasional upload-latency spikes even when its
     * mean rate is sufficient. Hold RF until the transport's continuation
     * budget is resident (four USB, five IQ10 Ethernet, or six IQ8 Ethernet
     * RF batches total), or
     * until a queued continuation is already final. */
    const int64_t prebuffer_deadline_us = esp_timer_get_time() + 3000000;
    const uint32_t prebuffer_continuations =
        rate_code == 14u || rate_code == 15u
            ? 3u
        : (commit_flags & IQ_TX_UDP_FLAG_PACKED16) != 0u
            ? S31_TXDC_CONTINUATION_QUEUE_DEPTH
            : S31_TXDC_ETHERNET_PACKED20_QUEUE_DEPTH;
    while (s31_txdc_continuation_count() < prebuffer_continuations &&
           !s31_txdc_continuation_has_final() &&
           esp_timer_get_time() < prebuffer_deadline_us) {
      vTaskDelay(1);
    }
    if (s31_txdc_continuation_count() == 0u) {
      taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
      s_s31_tx_replay_upload = NULL;
      s_s31_tx_replay_upload_words = 0u;
      s_s31_tx_replay_commit_flags = 0u;
      s_s31_txdc_stream_active = false;
      s_s31_txdc_stream_rate_code = 0u;
      taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
      s31_txdc_release_upload(upload);
      modem_stop_tx_replay();
      s31_tx_replay_publish_missed(word_count, rate_code,
                                   requested_start_time_ns);
      return;
    }
  }
  if (tcm_staged_stream) {
    if (s_s31_txdc_stager_task_handle == NULL ||
        !__atomic_load_n(&s_s31_txdc_stager_done, __ATOMIC_ACQUIRE)) {
      s31_txdc_release_upload(upload);
      taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
      s_s31_tx_replay_upload = NULL;
      s_s31_tx_replay_upload_words = 0u;
      s_s31_tx_replay_commit_flags = 0u;
      s_s31_txdc_stream_active = false;
      s_s31_txdc_stream_rate_code = 0u;
      taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
      modem_stop_tx_replay();
      s31_tx_replay_publish_missed(word_count, rate_code,
                                   requested_start_time_ns);
      return;
    }
    s31_txdc_configure_stage_slots();
    for (uint32_t slot = 0u; slot < s_s31_txdc_tcm_slot_count; ++slot) {
      __atomic_store_n(&s_s31_txdc_tcm_slots[slot]->ready, 0u,
                       __ATOMIC_RELEASE);
    }
    s_s31_txdc_stager_initial_upload = upload;
    s_s31_txdc_stager_initial_words = word_count;
    s_s31_txdc_stager_initial_flags = commit_flags;
    __atomic_store_n(&s_s31_txdc_stager_stop, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_s31_txdc_stager_done, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_s31_txdc_rf_cycles_per_sample, cycles_per_sample,
                     __ATOMIC_RELAXED);
    __atomic_store_n(&s_s31_txdc_stager_start, true, __ATOMIC_RELEASE);
    taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
    s_s31_tx_replay_upload = NULL;
    s_s31_tx_replay_upload_words = 0u;
    taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
    s31_txdc_stager_wakeup_start();
    xTaskNotifyGive(s_s31_txdc_stager_task_handle);
  }
  uint32_t *queued_upload = NULL;
  uint32_t *overflow_reclaims[S31_TXDC_CONTINUATION_QUEUE_DEPTH] = {0};
  uint32_t overflow_reclaim_count = 0u;
  uint32_t queued_words = 0u;
  uint16_t queued_flags = 0u;

  /* Start a continuous stream only after the second batch is resident. The
   * first USB commit response can overlap modem/config work by tens of
   * milliseconds; without this one-batch cushion, a transport that easily
   * exceeds the steady-state byte rate can still underrun the first seam. */
  if (!tcm_staged_stream &&
      (commit_flags & IQ_TX_UDP_FLAG_MORE) != 0u) {
    const int64_t prebuffer_deadline_us = esp_timer_get_time() + 1000000;
    while (s31_txdc_continuation_count() == 0u &&
           esp_timer_get_time() < prebuffer_deadline_us) {
      vTaskDelay(1);
    }
    if (s31_txdc_continuation_count() == 0u) {
      taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
      s_s31_tx_replay_upload = NULL;
      s_s31_tx_replay_upload_words = 0u;
      s_s31_tx_replay_commit_flags = 0u;
      s_s31_txdc_stream_active = false;
      s_s31_txdc_stream_rate_code = 0u;
      taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
      s31_txdc_release_upload(upload);
      s31_txdc_continuation_drain();
      modem_stop_tx_replay();
      s31_tx_replay_publish_missed(word_count, rate_code,
                                   requested_start_time_ns);
      return;
    }
    queued_upload =
        s31_txdc_continuation_pop(&queued_words, &queued_flags);
  }

  if (tcm_staged_stream) {
    const int64_t stage_deadline_us = esp_timer_get_time() + 1000000;
    uint32_t initial_slots = 0u;
    uint32_t initial_capacity = 0u;
    while (initial_slots < s_s31_txdc_tcm_slot_count &&
           initial_capacity < word_count) {
      initial_capacity += s_s31_txdc_tcm_slot_capacity[initial_slots];
      ++initial_slots;
    }
    bool initial_stage_ready = true;
    for (uint32_t slot = 0u; slot < initial_slots; ++slot) {
      while (__atomic_load_n(&s_s31_txdc_tcm_slots[slot]->ready,
                             __ATOMIC_ACQUIRE) == 0u &&
             !__atomic_load_n(&s_s31_txdc_stager_done,
                              __ATOMIC_ACQUIRE) &&
             esp_timer_get_time() < stage_deadline_us) {
        vTaskDelay(1);
      }
      if (__atomic_load_n(&s_s31_txdc_tcm_slots[slot]->ready,
                          __ATOMIC_ACQUIRE) == 0u) {
        initial_stage_ready = false;
        break;
      }
    }
    if (!initial_stage_ready) {
      __atomic_store_n(&s_s31_txdc_stager_stop, true, __ATOMIC_RELEASE);
      s31_txdc_stager_wakeup_stop();
      while (!__atomic_load_n(&s_s31_txdc_stager_done,
                              __ATOMIC_ACQUIRE) &&
             esp_timer_get_time() < stage_deadline_us + 100000) {
        vTaskDelay(1);
      }
      s31_txdc_continuation_drain();
      taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
      s_s31_txdc_stream_active = false;
      s_s31_txdc_stream_rate_code = 0u;
      taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
      modem_stop_tx_replay();
      s31_tx_replay_publish_missed(word_count, rate_code,
                                   requested_start_time_ns);
      return;
    }
    /* USB combines 16 368-word stopped-GMAC slots with seven 1,280-word
     * static slots (14,848 words, 2.78 ms at 320/60 MSa/s). Ethernet keeps
     * GMAC live and uses the seven static slots (2.24 ms at 4 MSa/s). */
  }

  modem_enter_debug_mode();
  int64_t requested_start_us = requested_start_time_ns == 0u
                                   ? 0
                                   : (int64_t)((requested_start_time_ns +
                                                999u) /
                                               1000u);
  while (requested_start_us != 0 &&
         requested_start_us - esp_timer_get_time() > 2100) {
    int64_t remaining_us = requested_start_us - esp_timer_get_time();
    TickType_t ticks = pdMS_TO_TICKS((remaining_us - 1100) / 1000);
    vTaskDelay(ticks > 0 ? ticks : 1);
  }
  if (requested_start_us != 0 && esp_timer_get_time() >= requested_start_us) {
    modem_stop_tx_replay();
    s31_tx_replay_publish_missed(word_count, rate_code,
                                 requested_start_time_ns);
    return;
  }

  modem_rearm_tx_replay();
  const uint32_t txdc_original =
      reg32p_read(&MODEM_WIFI_FE_WIFI.TXDC_DIGITAL);
  const uint32_t txdc_control =
      (txdc_original & ~0x000fffffu) | 0x06000000u;
  const uint32_t txon_ctrl_start =
      reg32p_read(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL);
  const uint32_t pbus_force_start =
      reg32p_read(&MODEM_WIFI_FE_CTRL.PBUS_FORCE_CTRL);
  const uint32_t pa_close_start =
      reg32p_read(&MODEM_WIFI_FE_CTRL.PA_CLOSE_FORCE_CTRL);
  const uint32_t fe_clocks_start =
      (reg32p_read(&MODEM_WIFI_FE_CTRL.CLK_ENABLE) & 0xffffu) |
      (reg32p_read(&MODEM_WIFI_FE_DATA.CLK_ENABLE) << 16u);
  const uint32_t pwdet_start =
      reg32p_read(&MODEM_LP_ANA.PWDET_DEBUG_STATUS);
  const uint32_t fe_debug_start =
      reg32p_read(&MODEM_WIFI_FE_CTRL.DEBUG_STATUS);
  const uint32_t mac_status_start =
      reg32p_read(&WIFI_MAC_SCHED.INT_STATUS);
  const uint32_t pbus_read_start =
      (phy_pbus_rd(PHY_PBUS_BLOCK_BB, PHY_PBUS_BANK_EN1) & 0x1ffu) |
      ((phy_pbus_rd(PHY_PBUS_BLOCK_RFTX1, PHY_PBUS_BANK_EN1) & 0x1ffu)
       << 9u) |
      ((phy_pbus_rd(PHY_PBUS_BLOCK_RFTX2, PHY_PBUS_BANK_EN1) & 0x1ffu)
       << 18u);
  uint32_t maximum_lateness = 0u;
  uint64_t actual_start_time_ns = 0u;
  uint32_t total_words = 0u;
  uint32_t segment_count = 0u;
  uint32_t gap_cycles_total = 0u;
  uint32_t gap_cycles_max = 0u;
  bool stream_ok = true;
  uint32_t tcm_max_lateness_word = 0u;

  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  while (requested_start_us != 0 &&
         esp_timer_get_time() < requested_start_us) {
    __asm__ __volatile__("nop");
  }
  actual_start_time_ns = (uint64_t)esp_timer_get_time() * 1000u;
  const uint32_t start_cycle = esp_cpu_get_cycle_count();
  uint32_t deadline = start_cycle;
  if (tcm_staged_stream) {
    __atomic_store_n(&s_s31_txdc_rf_start_cycle, start_cycle,
                     __ATOMIC_RELAXED);
    __atomic_store_n(&s_s31_txdc_rf_timing_active, true,
                     __ATOMIC_RELEASE);
    s31_txdc_tcm_emit_result_t result;
    s31_txdc_emit_tcm_slots(txdc_control, cycles_per_sample, start_cycle,
                           &result);
    total_words = result.total_words;
    segment_count = result.segment_count;
    gap_cycles_total = result.gap_cycles_total;
    gap_cycles_max = result.gap_cycles_max;
    maximum_lateness = result.maximum_lateness;
    tcm_max_lateness_word = result.maximum_lateness_word;
    stream_ok = result.stream_ok;
    __atomic_store_n(&s_s31_txdc_rf_timing_active, false,
                     __ATOMIC_RELEASE);
  } else while (true) {
    ++segment_count;
    uint32_t sample = 0u;
    const bool has_more = (commit_flags & IQ_TX_UDP_FLAG_MORE) != 0u;
    const uint32_t prefetch_sample =
        word_count > 65536u ? word_count - 65536u : 0u;
    bool prefetch_checked = queued_upload != NULL || !has_more ||
                            word_count <= 65536u;
    while (sample < word_count) {
        /* Keep protocol and watchdog work outside the per-sample hot loop. At
         * 40/9 MSa/s the 72-cycle period leaves little headroom beyond the
         * modem MMIO write, so even always-false housekeeping branches matter.
         */
      uint32_t run_end = word_count;
      const uint32_t watchdog_end =
          sample +
          (S31_TXDC_IWDT_FEED_WORDS -
           ((total_words + sample) & (S31_TXDC_IWDT_FEED_WORDS - 1u)));
      if (watchdog_end < run_end) {
        run_end = watchdog_end;
      }
      if (!prefetch_checked && prefetch_sample >= sample &&
          prefetch_sample < run_end) {
        run_end = prefetch_sample;
      }
      if (!prefetch_checked && sample == prefetch_sample) {
          /* Pull the following batch into local registers well before the seam.
           * Clearing the shared slot here also lets the transport queue one
           * more batch ahead without adding any work to the boundary itself. */
        uint32_t *candidate =
            s31_txdc_continuation_pop(&queued_words, &queued_flags);
        if (candidate != NULL) {
          queued_upload = candidate;
        }
        prefetch_checked = true;
        continue;
      }
      for (; sample < run_end; ++sample) {
          /* Fetch from PSRAM before the deadline wait. A cache-line fill can
           * then consume otherwise-idle pacing cycles instead of delaying the
           * modem MMIO write after its deadline (and escaping the lateness
           * measurement below). Packed streams always take the staged branch
           * above and therefore never reach this loop. */
        const uint32_t value = upload[sample] & 0x000fffffu;
        uint32_t now;
        do {
          now = esp_cpu_get_cycle_count();
        } while ((int32_t)(now - deadline) < 0);
        uint32_t late = now - deadline;
        if (late > maximum_lateness) {
          maximum_lateness = late;
        }
        MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_control | value;
        deadline += cycles_per_sample;
      }
      if (sample == watchdog_end) {
        __atomic_add_fetch(&s_s31_txdc_iwdt_heartbeat, 1u,
                           __ATOMIC_RELAXED);
      }
    }
    total_words += word_count;
    if ((commit_flags & IQ_TX_UDP_FLAG_MORE) == 0u) {
      break;
    }

    /* The transport core owns the next PSRAM allocation. It publishes the
     * pointer last with release ordering, so the realtime core can consume
     * count/flags locklessly without ever blocking on a FreeRTOS spinlock. */
    if (queued_upload == NULL) {
      const uint32_t wait_start = esp_cpu_get_cycle_count();
      while ((queued_upload = s31_txdc_continuation_pop(
                  &queued_words, &queued_flags)) == NULL &&
             esp_cpu_get_cycle_count() - wait_start <
                 S31_TXDC_CONTINUATION_WAIT_CYCLES) {
        if ((esp_cpu_get_cycle_count() & 0x3fffffu) == 0u) {
          __atomic_add_fetch(&s_s31_txdc_iwdt_heartbeat, 1u,
                             __ATOMIC_RELAXED);
        }
        __asm__ __volatile__("nop");
      }
      if (queued_upload == NULL) {
        stream_ok = false;
        break;
      }
      const uint32_t ready_cycle = esp_cpu_get_cycle_count();
      const int32_t boundary_late = (int32_t)(ready_cycle - deadline);
      if (boundary_late > 0) {
        gap_cycles_total += (uint32_t)boundary_late;
        if ((uint32_t)boundary_late > gap_cycles_max) {
          gap_cycles_max = (uint32_t)boundary_late;
        }
      }
    }
    uint32_t *old_reclaim = __atomic_exchange_n(
        &s_s31_txdc_reclaim_upload, upload, __ATOMIC_ACQ_REL);
    if (old_reclaim != NULL) {
      /* With two batches prebuffered, a finite three-batch stream can finish
       * before another host callback gets a chance to reclaim batch one.
       * Retain one extra retired pointer; steady-state callbacks normally
       * drain both slots before the following boundary. */
      uint32_t *old_reclaim2 = __atomic_exchange_n(
          &s_s31_txdc_reclaim_upload2, old_reclaim, __ATOMIC_ACQ_REL);
      if (old_reclaim2 != NULL) {
        __atomic_store_n(&s_s31_txdc_reclaim_upload2, old_reclaim2,
                         __ATOMIC_RELEASE);
          if (overflow_reclaim_count < S31_TXDC_CONTINUATION_QUEUE_DEPTH) {
            /* A finite deeply-prebuffered stream can finish its host uploads
             * before another arm callback reclaims retired RF batches. Retain
             * up to one continuation FIFO's worth locally and avoid heap work
             * at the RF seam. */
            overflow_reclaims[overflow_reclaim_count++] = old_reclaim;
          } else {
            /* The current upload was already published in reclaim_upload by
             * the exchange above. Do not also free its stale local alias during
             * teardown if an unexpected fourth retired buffer forces abort. */
            upload = NULL;
        stream_ok = false;
        break;
          }
      }
    }
    upload = queued_upload;
    word_count = queued_words;
    commit_flags = queued_flags;
    queued_upload = NULL;
  }
  const uint32_t sample_cycles = esp_cpu_get_cycle_count() - start_cycle;
  const uint32_t txon_ctrl_end =
      reg32p_read(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL);
  const uint32_t pbus_force_end =
      reg32p_read(&MODEM_WIFI_FE_CTRL.PBUS_FORCE_CTRL);
  const uint32_t pa_close_end =
      reg32p_read(&MODEM_WIFI_FE_CTRL.PA_CLOSE_FORCE_CTRL);
  const uint32_t fe_clocks_end =
      (reg32p_read(&MODEM_WIFI_FE_CTRL.CLK_ENABLE) & 0xffffu) |
      (reg32p_read(&MODEM_WIFI_FE_DATA.CLK_ENABLE) << 16u);
  const uint32_t pwdet_end =
      reg32p_read(&MODEM_LP_ANA.PWDET_DEBUG_STATUS);
  const uint32_t fe_debug_end =
      reg32p_read(&MODEM_WIFI_FE_CTRL.DEBUG_STATUS);
  const uint32_t mac_status_end =
      reg32p_read(&WIFI_MAC_SCHED.INT_STATUS);
  const uint32_t pbus_read_end =
      (phy_pbus_rd(PHY_PBUS_BLOCK_BB, PHY_PBUS_BANK_EN1) & 0x1ffu) |
      ((phy_pbus_rd(PHY_PBUS_BLOCK_RFTX1, PHY_PBUS_BANK_EN1) & 0x1ffu)
       << 9u) |
      ((phy_pbus_rd(PHY_PBUS_BLOCK_RFTX2, PHY_PBUS_BANK_EN1) & 0x1ffu)
       << 18u);
  MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_original;
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  modem_stop_tx_replay();

  if (tcm_staged_stream) {
    ESP_LOGI("iq_capture",
             "TXDC staged timing: max_late=%" PRIu32 " at word=%" PRIu32
             " gap_total=%" PRIu32 " gap_max=%" PRIu32,
             maximum_lateness, tcm_max_lateness_word, gap_cycles_total,
             gap_cycles_max);
  }

  if (tcm_staged_stream) {
    __atomic_store_n(&s_s31_txdc_stager_stop, true, __ATOMIC_RELEASE);
    s31_txdc_stager_wakeup_stop();
    const int64_t stop_deadline_us = esp_timer_get_time() + 100000;
    while (!__atomic_load_n(&s_s31_txdc_stager_done,
                            __ATOMIC_ACQUIRE) &&
           esp_timer_get_time() < stop_deadline_us) {
      vTaskDelay(1);
    }
    for (uint32_t slot = 0u; slot < s_s31_txdc_tcm_slot_count; ++slot) {
      __atomic_store_n(&s_s31_txdc_tcm_slots[slot]->ready, 0u,
                       __ATOMIC_RELEASE);
    }
  }

  uint32_t *reclaim = __atomic_exchange_n(
      &s_s31_txdc_reclaim_upload, NULL, __ATOMIC_ACQ_REL);
  uint32_t *reclaim2 = __atomic_exchange_n(
      &s_s31_txdc_reclaim_upload2, NULL, __ATOMIC_ACQ_REL);
  taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
  s_s31_tx_replay_upload = NULL;
  s_s31_tx_replay_upload_words = 0u;
  s_s31_tx_replay_commit_flags = 0u;
  s_s31_txdc_stream_active = false;
  s_s31_txdc_stream_rate_code = 0u;
  taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
  s31_txdc_release_upload(reclaim);
  s31_txdc_release_upload(reclaim2);
  if (!tcm_staged_stream) {
    s31_txdc_release_upload(upload);
  }
  s31_txdc_continuation_drain();
  s31_txdc_release_upload(queued_upload);
  for (uint32_t i = 0u; i < overflow_reclaim_count; ++i) {
  s31_txdc_release_upload(overflow_reclaims[i]);
  }
  if (tcm_staged_stream) {
    /* All pool buffers have now returned and RF is off, so allocator work is
     * safe again and RX can reclaim the PSRAM capacity immediately. */
    iq_usb_tx_pool_trim();
  }

  s_s31_tx_replay_diag = (iq_tx_replay_diag_t){
      .words = total_words,
      .rate_code = rate_code,
      .segments = segment_count,
      .total_cycles = sample_cycles,
      .sample_cycles = sample_cycles,
      .gap_cycles = gap_cycles_total,
      .maximum_gap_cycles = gap_cycles_max,
      .tcm_stage_copy_max_cycles =
          tcm_staged_stream ? s_s31_txdc_stager_copy_max_cycles : 0u,
      .live_probe_write_cycles = txon_ctrl_start,
      .live_probe_match_words = s_s31_txdc_commit_count,
      .live_probe_dma_status = txon_ctrl_end,
      .deadline_late_max_cycles = maximum_lateness,
      .deadline_late_max_word = tcm_max_lateness_word,
      .pbus_force_start = pbus_force_start,
      .pbus_force_end = pbus_force_end,
      .pa_close_start = pa_close_start,
      .pa_close_end = pa_close_end,
      .fe_clocks_start = fe_clocks_start,
      .fe_clocks_end = fe_clocks_end,
      .pwdet_start = pwdet_start,
      .pwdet_end = pwdet_end,
      .fe_debug_start = fe_debug_start,
      .fe_debug_end = fe_debug_end,
      .mac_status_start = mac_status_start,
      .mac_status_end = mac_status_end,
      .pbus_read_start = pbus_read_start,
      .pbus_read_end = pbus_read_end,
      .live_probe_dma_ok = stream_ok &&
                           maximum_lateness < cycles_per_sample &&
                           gap_cycles_total == 0u,
      .requested_start_time_ns = requested_start_time_ns,
      .actual_start_time_ns = actual_start_time_ns,
      .start_error_ns = requested_start_time_ns == 0u
                            ? 0
                            : (int64_t)actual_start_time_ns -
                                  (int64_t)requested_start_time_ns,
      .queue_underflow = !stream_ok,
  };
  s_s31_tx_replay_burst_complete = true;
}

static void s31_tx_replay_enable(const capture_config_t *config) {
  s_s31_tx_replay_burst_complete = false;
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);

#if CONFIG_ESP_SDR_S31_TX_PIE_REFILL
  /* Prime this task's lazily saved PIE context before interrupts are masked
   * for the replay aperture handoff. */
  uint32_t __attribute__((aligned(16))) pie_prime_src[4] = {0};
  uint32_t __attribute__((aligned(16))) pie_prime_dst[4];
  s31_pie_memcpy_aligned(pie_prime_dst, pie_prime_src,
                         sizeof(pie_prime_src));
#endif

  const uint32_t replay_pattern =
      (uint32_t)config->tx.tx_tone0_step & 15u;
  const uint32_t rate_code =
      ((uint32_t)config->tx.tx_tone0_step >> 4u) & 0x0fu;
  const bool digital_txdc_upload =
      replay_pattern == 3u &&
      (rate_code >= 9u && rate_code <= 15u);
  uint32_t waveform_words = digital_txdc_upload
                                ? 0u
                                : s31_tx_replay_fill(replay_pattern);
  bool uploaded_waveform =
      replay_pattern == 3u;
  uint32_t upload_total_words = waveform_words;
  uint64_t requested_start_time_ns = 0u;
  uint16_t upload_commit_flags = 0u;
  if (uploaded_waveform) {
    taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
    upload_total_words = s_s31_tx_replay_upload_words;
    requested_start_time_ns = s_s31_tx_replay_start_time_ns;
    upload_commit_flags = s_s31_tx_replay_commit_flags;
    /* A schedule is consumed by exactly one replay. Subsequent explicit
     * configuration changes must not unexpectedly replay the old deadline. */
    s_s31_tx_replay_start_time_ns = 0u;
    taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
  }
  if (requested_start_time_ns != 0u &&
      (uint64_t)esp_timer_get_time() * 1000u >= requested_start_time_ns) {
    modem_stop_tx_replay();
    s31_tx_replay_publish_missed(upload_total_words, rate_code,
                                 requested_start_time_ns);
    return;
  }
  if (digital_txdc_upload) {
    s31_txdc_emit_uploaded(upload_total_words, rate_code, upload_commit_flags,
                           requested_start_time_ns);
    return;
  }
  modem_enter_debug_mode();

  capture_config_t replay_config = *config;
  replay_config.iq_engine.adc_source_sel &= ~S31_ADC_SOURCE_HW_DECIM_M;
  replay_config.iq_engine.adc_source_sel |=
      (((uint32_t)config->tx.tx_tone0_step >> 4u) & 0x0fu)
      << S31_ADC_SOURCE_HW_DECIM_S;

  /* adctrig's data-dump reset also resets the TX route.  Prepare the replay
   * clock and TCM aperture first, then restore calibrated RF state. */
  /* The ownership register applies to the physical HP SRAM fabric, not just
   * the nominal replay address.  Letting the peer execute USB/Ethernet or
   * Wi-Fi code while the modem owns all eight lanes eventually faults that
   * core.  The waveform is fully prebuffered, so park the peer for each
   * bounded replay window; the control transfer completes immediately after
   * ownership returns to the CPUs. */
  const bool stall_peer_cpu = true;
  if (stall_peer_cpu) {
    esp_ipc_isr_stall_other_cpu();
  }
  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                   S31_ADCTRIG_TCM_DUMP_CTRL);
  adctrig_prepare(waveform_words, &replay_config);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
  if (((uint32_t)config->tx.tx_tone0_step & 15u) == 7u) {
    const uint32_t selector =
        ((uint32_t)config->tx.tx_tone0_step &
         S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M) >>
        S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_S;
    const uint32_t pointer_value =
        selector == 0u ? 0u : BIT(selector - 1u);
    reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_MODE,
                       MODEM_WIFI_DUMP_MODE_STORE_ADDR_M,
                       MODEM_WIFI_DUMP_MODE_STORE_ADDR_S,
                       pointer_value);
  }
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  if (stall_peer_cpu) {
    esp_ipc_isr_release_other_cpu();
  }

  /* MACTOADCDUMP0 is a replay reader in this mode.  size is the sample count,
   * loop keeps the uploaded symbol seamless, and enable starts it. */
  bool one_shot = ((uint32_t)config->tx.tx_tone0_step &
                   S31_TX_REPLAY_ONE_SHOT_REQUEST) != 0u;
  const uint32_t address_probe_extra =
      replay_pattern == 4u || replay_pattern == 7u || replay_pattern == 8u
          ? BIT(26)
          : replay_pattern == 9u || replay_pattern == 10u ||
                replay_pattern == 12u || replay_pattern == 13u ||
                replay_pattern == 14u
                ? 0u
          : s31_tx_replay_address_probe_extra(
                (uint32_t)config->tx.tx_tone0_step);
  const uint32_t replay_gate_mask = s31_tx_replay_gate_probe_mask(
      (uint32_t)config->tx.tx_tone0_step);
  volatile uint32_t *replay_aux_probe_reg = NULL;
  uint32_t replay_aux_probe_original = 0u;
  const bool address_probe =
      ((uint32_t)config->tx.tx_tone0_step &
       S31_TX_REPLAY_ADDRESS_PROBE) != 0u;
  bool live_write_probe = !one_shot &&
      (!address_probe || replay_pattern == 9u || replay_pattern == 10u ||
       replay_pattern == 12u || replay_pattern == 13u ||
       replay_pattern == 14u) &&
      (((uint32_t)config->tx.tx_tone0_step &
        S31_TX_REPLAY_LIVE_WRITE_PROBE) != 0u);
  bool live_gate_probe = live_write_probe &&
      (((uint32_t)config->tx.tx_tone0_step &
        S31_TX_REPLAY_LIVE_GATE_PROBE) != 0u);
#if CONFIG_ESP_SDR_S31_TX_DIRECT_DMA_REFILL
  bool scatter_dma_probe = live_write_probe && replay_pattern == 13u &&
                           s31_tx_scatter_dma_prepare(waveform_words);
  bool scatter_ahb_probe = live_write_probe && replay_pattern == 12u &&
                           s31_tx_scatter_ahb_prepare(waveform_words);
#else
  const bool scatter_dma_probe = false;
  const bool scatter_ahb_probe = false;
#endif
  uint32_t segment_count =
      ((uint32_t)config->tx.tx_tone0_step & S31_TX_REPLAY_SEGMENT_COUNT_M) >>
      S31_TX_REPLAY_SEGMENT_COUNT_S;
  bool uploaded_batch = one_shot && uploaded_waveform &&
                        upload_total_words > S31_TX_REPLAY_SIZE_M;
#if CONFIG_ESP_SDR_S31_TX_DIRECT_DMA_REFILL
  bool direct_dma_batch = uploaded_batch && s31_tx_direct_dma_prepare();
  if (direct_dma_batch &&
      esp_cache_msync(
          s_s31_tx_replay_upload,
          upload_total_words * sizeof(*s_s31_tx_replay_upload),
          ESP_CACHE_MSYNC_FLAG_DIR_C2M |
              ESP_CACHE_MSYNC_FLAG_UNALIGNED) != ESP_OK) {
    ESP_LOGW("iq_capture", "TX refill source cache publication failed");
    direct_dma_batch = false;
  }
#else
  const bool direct_dma_batch = false;
#endif
#if CONFIG_ESP_SDR_S31_TX_PIPELINED_REFILL
  bool upload_external = esp_ptr_external_ram(s_s31_tx_replay_upload);
  bool pipelined_batch = uploaded_batch && upload_external &&
                         s31_tx_prefetch_prepare();
  if (uploaded_batch && !pipelined_batch) {
    ESP_LOGW("iq_capture",
             "TX prefetch unavailable: upload=%p external=%u internal_largest=%zu",
             s_s31_tx_replay_upload, upload_external ? 1u : 0u,
             heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                              MALLOC_CAP_8BIT));
  }
#else
  const bool pipelined_batch = false;
#endif
  if (uploaded_batch) {
    uint32_t segment_words = pipelined_batch
#if CONFIG_ESP_SDR_S31_TX_PIPELINED_REFILL
                                 ? S31_TX_REPLAY_PIPELINE_WORDS
#else
                                 ? S31_TX_REPLAY_SIZE_M
#endif
                                 : S31_TX_REPLAY_REFILL_WORDS;
    segment_count = (upload_total_words + segment_words - 1u) /
                    segment_words;
  }
  if (!one_shot || segment_count == 0u) {
    segment_count = 1u;
  }
  /* Only the uploaded-waveform path has a persistent PSRAM source from which
   * to refill the modem aperture between segments. */
  if (!uploaded_waveform) {
    segment_count = 1u;
  }
  int64_t requested_start_us = 0;
  if (requested_start_time_ns != 0u) {
    requested_start_us =
        (int64_t)((requested_start_time_ns + 999u) / 1000u);
    int64_t remaining_us = requested_start_us - esp_timer_get_time();
    /* Do not monopolize the config task while waiting for a comfortably
     * future burst. The final sub-millisecond interval is intentionally a
     * busy wait so RTOS tick quantization does not dominate RF start error. */
    while (remaining_us > 2100) {
      TickType_t ticks = pdMS_TO_TICKS((remaining_us - 1100) / 1000);
      vTaskDelay(ticks > 0 ? ticks : 1);
      remaining_us = requested_start_us - esp_timer_get_time();
    }
    while (esp_timer_get_time() + 100 < requested_start_us) {
      __asm__ __volatile__("nop");
    }
  }
  bool deadline_missed =
      requested_start_time_ns != 0u &&
      (uint64_t)esp_timer_get_time() * 1000u >= requested_start_time_ns;
  if (!deadline_missed) {
    /* Keep the RF path cold during upload, cache publication, and long timer
     * waits. Only force TXON immediately before the protected replay gate. */
    modem_rearm_tx_replay();
  }
  /* TXDC_DIGITAL is a modem MMIO sink and does not use replay SRAM.  In the
   * no-replay probe the writer core still runs interrupt-free, but the peer
   * remains schedulable so USB/Ethernet can continue draining and filling
   * host buffers. */
  const bool txdc_without_replay =
      replay_pattern == 11u &&
      (((uint32_t)config->tx.tx_tone0_step &
        S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M) != 0u);
  if (stall_peer_cpu && !txdc_without_replay) {
    esp_ipc_isr_stall_other_cpu();
  }
  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  /* TCM ownership must remain asserted for every replay sample.  Keep both
   * CPUs quiescent until a one-shot completes, or for a deterministic ~20 ms
   * cyclic burst; returning to the scheduler with this gate open lets
   * unrelated modem users fault. */
  uint32_t replay_start_cycle = esp_cpu_get_cycle_count();
  uint32_t replay_end_ctrl = 0u;
  uint32_t sample_cycles = 0u;
  uint32_t gap_cycles_total = 0u;
  uint32_t gap_cycles_max = 0u;
  uint32_t live_probe_write_cycles = 0u;
  uint32_t live_probe_ctrl_min = UINT32_MAX;
  uint32_t live_probe_ctrl_max = 0u;
  uint32_t live_probe_ctrl_changes = 0u;
  uint32_t live_probe_deadline_late_max = 0u;
  uint32_t live_probe_match_words = 0u;
  uint32_t live_probe_dma_status = 0u;
  uint64_t actual_start_time_ns = 0u;
  bool pipeline_dma_ok = true;
  bool lane_refill_batch = false;
  const uint32_t replay_probe_selector =
      ((uint32_t)config->tx.tx_tone0_step &
       S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M) >>
      S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_S;
  const bool tcm_mem_enable_hold_probe =
      replay_pattern == 5u ||
      (replay_pattern == 12u && replay_probe_selector != 0u);
  const uint32_t tcm_performance_ctrl_original =
      reg32_read_addr(HP_SYSTEM_TCM_PERFORMACE_CTRL_REG);
  if (tcm_mem_enable_hold_probe) {
    /* S31 exposes one otherwise-unused TCM arbitration control: keeping the
     * AHB-to-TCM memory-enable asserted while an access is busy.  Pattern 5
     * compares it with pattern 1 during the same ownership-safe live rewrite
     * probe.  If it permits the CPU write to complete without disturbing the
     * modem read port, it is the missing primitive for a DMA-fed ring. */
    reg32_write_addr(HP_SYSTEM_TCM_PERFORMACE_CTRL_REG,
                     tcm_performance_ctrl_original |
                         HP_SYSTEM_AHB_TO_TCM_MEM_EN_HOLD);
  }
  if (replay_pattern == 4u || replay_pattern == 8u) {
    const uint32_t selector =
        ((uint32_t)config->tx.tx_tone0_step &
         S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M) >>
        S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_S;
    replay_aux_probe_reg = (volatile uint32_t *)(uintptr_t)(
        DR_REG_MODEM_DATADUMP_BASE + (replay_pattern == 8u ? 0x0cu : 0x10u));
    replay_aux_probe_original = reg32p_read(replay_aux_probe_reg);
    reg32p_write(replay_aux_probe_reg,
                 replay_aux_probe_original ^ BIT(selector));
  }
  if (deadline_missed) {
    /* Upload and control-plane staging may outlive an otherwise valid timed
     * request.  Never turn that into an unexpected late RF transmission:
     * publish a completed request with actual_start_time_ns == 0 instead. */
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  } else if (replay_pattern == 11u) {
    /* Bounded digital-TXDC modulation probe.  The vendor's set_txdc_dig()
     * writes signed I[9:0]/Q[19:10] here, downstream of the replay SRAM.  A
     * single full-word store per IQ update tests both whether the value is
     * live during TX and the upper bound on a CPU-fed streaming backend. */
    static const uint32_t txdc_tone[16] = {
        (192u & 0x3ffu) | ((0u & 0x3ffu) << 10u),
        (177u & 0x3ffu) | ((73u & 0x3ffu) << 10u),
        (136u & 0x3ffu) | ((136u & 0x3ffu) << 10u),
        (73u & 0x3ffu) | ((177u & 0x3ffu) << 10u),
        (0u & 0x3ffu) | ((192u & 0x3ffu) << 10u),
        ((uint32_t)-73 & 0x3ffu) | ((177u & 0x3ffu) << 10u),
        ((uint32_t)-136 & 0x3ffu) | ((136u & 0x3ffu) << 10u),
        ((uint32_t)-177 & 0x3ffu) | ((73u & 0x3ffu) << 10u),
        ((uint32_t)-192 & 0x3ffu) | ((0u & 0x3ffu) << 10u),
        ((uint32_t)-177 & 0x3ffu) | (((uint32_t)-73 & 0x3ffu) << 10u),
        ((uint32_t)-136 & 0x3ffu) | (((uint32_t)-136 & 0x3ffu) << 10u),
        ((uint32_t)-73 & 0x3ffu) | (((uint32_t)-177 & 0x3ffu) << 10u),
        (0u & 0x3ffu) | (((uint32_t)-192 & 0x3ffu) << 10u),
        (73u & 0x3ffu) | (((uint32_t)-177 & 0x3ffu) << 10u),
        (136u & 0x3ffu) | (((uint32_t)-136 & 0x3ffu) << 10u),
        (177u & 0x3ffu) | (((uint32_t)-73 & 0x3ffu) << 10u),
    };
    const uint32_t txdc_probe_samples =
        replay_probe_selector >= 2u ? 40000000u : 4000000u;
    const uint32_t txdc_replay_ctrl =
        1024u | S31_TX_REPLAY_LOOP_BIT |
        MODEM_WIFI_TOADCDUMP_ENABLE_BIT;
    const bool txdc_use_replay = replay_probe_selector == 0u;
    const uint32_t txdc_original = reg32p_read(&MODEM_WIFI_FE_WIFI.TXDC_DIGITAL);
    const uint32_t txdc_control =
        (txdc_original & ~0x000fffffu) | 0x06000000u;
    if (txdc_use_replay) {
      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                       S31_ADCTRIG_TCM_DUMP_CTRL);
      reg32p_write(&MODEM_WIFI_DUMP.MACTOADCDUMP0, txdc_replay_ctrl);
    }
    actual_start_time_ns = (uint64_t)esp_timer_get_time() * 1000u;
    const uint32_t txdc_start_cycle = esp_cpu_get_cycle_count();
    for (uint32_t sample = 0u; sample < txdc_probe_samples; ++sample) {
      MODEM_WIFI_FE_WIFI.TXDC_DIGITAL =
          txdc_control | txdc_tone[sample & 15u];
      if ((sample & 0x3ffffu) == 0x3ffffu) {
        s31_tx_replay_feed_iwdt();
      }
    }
    sample_cycles = esp_cpu_get_cycle_count() - txdc_start_cycle;
    MODEM_WIFI_FE_WIFI.TXDC_DIGITAL = txdc_original;
    replay_end_ctrl = reg32p_read(&MODEM_WIFI_DUMP.MACTOADCDUMP0);
    if (txdc_use_replay) {
      reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                        MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    }
  } else {
#if CONFIG_ESP_SDR_S31_TX_PIPELINED_REFILL
  if (one_shot && pipelined_batch) {
    /* GDMA and its completion ISR must run while the modem owns the TCM
     * fabric. Release the CPU stall around the pipelined per-segment helper;
     * it reasserts ownership only for each exact replay interval. */
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
    esp_ipc_isr_release_other_cpu();
    pipeline_dma_ok = s31_tx_replay_pipelined(
        upload_total_words, &replay_end_ctrl, &sample_cycles,
        &gap_cycles_total, &gap_cycles_max, requested_start_time_ns,
        &actual_start_time_ns);
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  } else
#endif
  if (one_shot && uploaded_batch) {
    /* The S31 TCM ownership switch is eight-way word interleaved: replay word
     * n is released by clearing bit 24 + (n mod 8). Hand each word to the CPU
     * only after the cyclic reader has consumed it, replace it, and restore
     * modem ownership before that lane is visited eight samples later. This
     * turns the single 16,383-word aperture into a true continuous ring. */
    static const uint8_t cycles_per_sample[16] = {
        [0] = 4u,  [1] = 8u,  [2] = 12u, [3] = 16u,
        [7] = 40u, [8] = 48u, [9] = 80u, [10] = 96u,
    };
    const uint32_t replay_sample_cycles = cycles_per_sample[rate_code];
    const uint32_t ring_words = waveform_words;
    const uint32_t ring_cycles = ring_words * replay_sample_cycles;
    uint32_t replay_ctrl = (ring_words & S31_TX_REPLAY_SIZE_M) |
                           S31_TX_REPLAY_LOOP_BIT |
                           MODEM_WIFI_TOADCDUMP_ENABLE_BIT;
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                     S31_ADCTRIG_TCM_DUMP_CTRL);
    while (requested_start_us != 0 &&
           esp_timer_get_time() < requested_start_us) {
      __asm__ __volatile__("nop");
    }
    reg32p_write(&MODEM_WIFI_DUMP.MACTOADCDUMP0, replay_ctrl);
    actual_start_time_ns = (uint64_t)esp_timer_get_time() * 1000u;
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    const uint32_t cyclic_start_cycle = esp_cpu_get_cycle_count();
    uint32_t output_cycle_start = cyclic_start_cycle;
    uint32_t upload_offset_words = ring_words;
    uint32_t final_segment_words = ring_words;
    lane_refill_batch = true;

    while (upload_offset_words < upload_total_words) {
      uint32_t remaining_words = upload_total_words - upload_offset_words;
      uint32_t next_segment_words =
          remaining_words > ring_words ? ring_words : remaining_words;
      const uint32_t *source =
          s_s31_tx_replay_upload + upload_offset_words;
      const uint32_t first_deadline =
          output_cycle_start +
          S31_TX_REPLAY_REFILL_GUARD_WORDS * replay_sample_cycles;
      s31_tx_replay_refill_ring(source, next_segment_words,
                                replay_sample_cycles, first_deadline);
      /* When the loop cannot keep up, lateness is monotonic after it exhausts
       * all idle slots, so an end-of-segment sample captures the meaningful
       * worst case without adding a cycle-counter read to every IQ word. */
      const int32_t final_late =
          (int32_t)(esp_cpu_get_cycle_count() -
                    (first_deadline +
                     (next_segment_words - 1u) * replay_sample_cycles));
      if (final_late > 0 &&
          (uint32_t)final_late > live_probe_deadline_late_max) {
        live_probe_deadline_late_max = (uint32_t)final_late;
      }
      if (final_late >= (int32_t)ring_cycles) {
        pipeline_dma_ok = false;
      }
      s31_tx_replay_feed_iwdt();
      upload_offset_words += next_segment_words;
      output_cycle_start += ring_cycles;
      final_segment_words = next_segment_words;
    }

    const uint32_t replay_end_cycle =
        output_cycle_start + final_segment_words * replay_sample_cycles;
    while ((int32_t)(esp_cpu_get_cycle_count() - replay_end_cycle) < 0) {
      __asm__ __volatile__("nop");
    }
    replay_end_ctrl = reg32p_read(&MODEM_WIFI_DUMP.MACTOADCDUMP0);
    reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                      MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    sample_cycles = esp_cpu_get_cycle_count() - cyclic_start_cycle;
    const uint32_t final_source_offset =
        upload_total_words - final_segment_words;
    for (uint32_t word = 0u; word < final_segment_words; ++word) {
      if (DUMP_BASE[word] ==
          s_s31_tx_replay_upload[final_source_offset + word]) {
        ++live_probe_match_words;
      }
    }
  } else if (one_shot) {
    uint32_t previous_done_cycle = 0u;
    for (uint32_t segment = 0u; segment < segment_count; ++segment) {
      uint32_t segment_words = waveform_words;
      uint32_t upload_offset_words = 0u;
      if (uploaded_batch) {
        upload_offset_words = segment * S31_TX_REPLAY_REFILL_WORDS;
        uint32_t remaining_words = upload_total_words - upload_offset_words;
        segment_words = remaining_words > S31_TX_REPLAY_REFILL_WORDS
                            ? S31_TX_REPLAY_REFILL_WORDS
                            : remaining_words;
      }
      if (segment != 0u) {
#if CONFIG_ESP_SDR_S31_TX_DIRECT_DMA_REFILL
        bool copied = false;
        if (direct_dma_batch) {
          taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
          esp_ipc_isr_release_other_cpu();
          copied = s31_tx_direct_dma_copy(
              s_s31_tx_replay_upload + upload_offset_words, segment_words);
          esp_ipc_isr_stall_other_cpu();
          taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
        }
        if (!copied)
#endif
        {
          s31_tx_replay_copy((void *)DUMP_BASE,
                             s_s31_tx_replay_upload + upload_offset_words,
                             segment_words);
        }
        __asm__ __volatile__("fence rw, rw" ::: "memory");
      }
      uint32_t replay_ctrl =
          (segment_words & S31_TX_REPLAY_SIZE_M) |
          MODEM_WIFI_TOADCDUMP_ENABLE_BIT;
      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                       S31_ADCTRIG_TCM_DUMP_CTRL);
      if (segment == 0u) {
        while (requested_start_us != 0 &&
               esp_timer_get_time() < requested_start_us) {
          __asm__ __volatile__("nop");
        }
      }
      reg32p_write(&MODEM_WIFI_DUMP.MACTOADCDUMP0, replay_ctrl);
      if (segment == 0u) {
        actual_start_time_ns = (uint64_t)esp_timer_get_time() * 1000u;
      }
      __asm__ __volatile__("fence rw, rw" ::: "memory");
      uint32_t segment_start_cycle = esp_cpu_get_cycle_count();
      if (segment != 0u) {
        uint32_t gap_cycles = segment_start_cycle - previous_done_cycle;
        gap_cycles_total += gap_cycles;
        if (gap_cycles > gap_cycles_max) {
          gap_cycles_max = gap_cycles;
        }
      }
      while ((reg32p_read(&MODEM_WIFI_DUMP.MACTOADCDUMP0) &
              S31_TX_REPLAY_DONE_BIT) == 0u &&
             esp_cpu_get_cycle_count() - segment_start_cycle < 20000000u) {
        __asm__ __volatile__("nop");
      }
      replay_end_ctrl = reg32p_read(&MODEM_WIFI_DUMP.MACTOADCDUMP0);
      previous_done_cycle = esp_cpu_get_cycle_count();
      sample_cycles += previous_done_cycle - segment_start_cycle;
      reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                        MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    }
  } else {
    uint32_t replay_ctrl = (waveform_words & S31_TX_REPLAY_SIZE_M) |
                           S31_TX_REPLAY_LOOP_BIT |
                           MODEM_WIFI_TOADCDUMP_ENABLE_BIT |
                           address_probe_extra;
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, replay_gate_mask);
    while (requested_start_us != 0 &&
           esp_timer_get_time() < requested_start_us) {
      __asm__ __volatile__("nop");
    }
    reg32p_write(&MODEM_WIFI_DUMP.MACTOADCDUMP0, replay_ctrl);
    actual_start_time_ns = (uint64_t)esp_timer_get_time() * 1000u;
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    const uint32_t cyclic_start_cycle = esp_cpu_get_cycle_count();
    uint32_t previous_probe_ctrl = replay_ctrl;
#if !CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF
    const uint32_t lp_completed_before = ulp_completed;
    ulp_command = 0u;
#endif
    for (uint32_t i = 0u; i < 6400000u; ++i) {
#if !CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF
      if (replay_pattern == 15u && i == 3200000u) {
        ulp_command = 1u;
      }
#endif
      if (live_write_probe && i == 3200000u) {
        uint32_t write_start = esp_cpu_get_cycle_count();
        if (replay_pattern == 12u) {
#if CONFIG_ESP_SDR_S31_TX_DIRECT_DMA_REFILL
          pipeline_dma_ok = scatter_ahb_probe &&
                            s31_tx_scatter_ahb_run(
                                &live_probe_write_cycles,
                                &live_probe_dma_status);
#else
          pipeline_dma_ok = false;
#endif
        } else if (replay_pattern == 13u) {
#if CONFIG_ESP_SDR_S31_TX_DIRECT_DMA_REFILL
          pipeline_dma_ok = scatter_dma_probe &&
                            s31_tx_scatter_dma_run(
                                &live_probe_write_cycles,
                                &live_probe_dma_status);
#else
          pipeline_dma_ok = false;
#endif
        } else if (replay_pattern == 14u) {
          const uint32_t selector =
              ((uint32_t)config->tx.tx_tone0_step &
               S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M) >>
              S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_S;
          if (selector != 0u) {
            static const uint8_t cycles_per_sample[16] = {
                [0] = 4u,  [1] = 8u,  [2] = 12u, [3] = 16u,
                [7] = 40u, [8] = 48u, [9] = 80u, [10] = 96u,
            };
            const uint32_t sample_cycles = cycles_per_sample[rate_code];
            const uint32_t period_cycles = waveform_words * sample_cycles;
            uint32_t now = esp_cpu_get_cycle_count();
            uint32_t elapsed = now - cyclic_start_cycle;
            uint32_t update_start =
                cyclic_start_cycle +
                ((elapsed / period_cycles) + 2u) * period_cycles;
            for (uint32_t word = 0u; word < waveform_words; ++word) {
              uint32_t deadline = update_start + word * sample_cycles + 2u;
              while ((int32_t)(esp_cpu_get_cycle_count() - deadline) < 0) {
                __asm__ __volatile__("nop");
              }
              uint32_t current = esp_cpu_get_cycle_count();
              uint32_t late = current - deadline;
              if (late > live_probe_deadline_late_max) {
                live_probe_deadline_late_max = late;
              }
              const uint32_t partial_gate =
                  S31_ADCTRIG_TCM_DUMP_CTRL & ~BIT(24u + (word & 7u));
              reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                               partial_gate);
              DUMP_BASE[word] = s_s31_tx_live_probe_words[word & 15u];
              reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                               S31_ADCTRIG_TCM_DUMP_CTRL);
            }
          }
        } else if (replay_pattern == 10u) {
          const uint32_t selector =
              ((uint32_t)config->tx.tx_tone0_step &
               S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M) >>
              S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_S;
          if (selector != 0u) {
            for (uint32_t lane = 0u; lane < 8u; ++lane) {
              const uint32_t partial_gate =
                  S31_ADCTRIG_TCM_DUMP_CTRL & ~BIT(24u + lane);
              reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                               partial_gate);
              __asm__ __volatile__("fence rw, rw" ::: "memory");
              const bool exhaustive = selector == 1u;
              const uint32_t block_words =
                  selector >= 2u && selector <= 8u
                      ? BIT(selector - 2u)
                      : 1u;
              for (uint32_t word = 0u; word < waveform_words; ++word) {
                uint32_t address_lane = (word / block_words) & 7u;
                if (selector == 9u) {
                  address_lane = 7u - (word & 7u);
                }
                if (exhaustive || address_lane == lane) {
                  DUMP_BASE[word] =
                      s_s31_tx_live_probe_words[word & 15u];
                }
              }
              __asm__ __volatile__("fence rw, rw" ::: "memory");
              reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                               S31_ADCTRIG_TCM_DUMP_CTRL);
              __asm__ __volatile__("fence rw, rw" ::: "memory");
            }
          }
        } else {
          for (uint32_t word = 0u; word < waveform_words; ++word) {
            uint32_t value = DUMP_BASE[word];
            uint32_t raw_q = (value >> 10u) & 0x3ffu;
            int32_t q = (raw_q & 0x200u) != 0u
                            ? (int32_t)raw_q - 1024
                            : (int32_t)raw_q;
            DUMP_BASE[word] = (value & ~(0x3ffu << 10u)) |
                              (((uint32_t)(-q) & 0x3ffu) << 10u);
          }
        }
        __asm__ __volatile__("fence rw, rw" ::: "memory");
        if (live_gate_probe) {
          /* Publish the CPU-side TCM view without stopping or rearming the
           * cyclic reader.  The two MMIO stores intentionally form the
           * shortest ownership handoff we can express from software. */
          reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
          __asm__ __volatile__("fence rw, rw" ::: "memory");
          reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                           S31_ADCTRIG_TCM_DUMP_CTRL);
          __asm__ __volatile__("fence rw, rw" ::: "memory");
        }
        if (replay_pattern != 12u && replay_pattern != 13u) {
          live_probe_write_cycles = esp_cpu_get_cycle_count() - write_start;
        }
      }
      if (live_write_probe && (i & 1023u) == 0u) {
        uint32_t ctrl = reg32p_read(&MODEM_WIFI_DUMP.MACTOADCDUMP0);
        if (ctrl < live_probe_ctrl_min) {
          live_probe_ctrl_min = ctrl;
        }
        if (ctrl > live_probe_ctrl_max) {
          live_probe_ctrl_max = ctrl;
        }
        if (ctrl != previous_probe_ctrl) {
          ++live_probe_ctrl_changes;
          previous_probe_ctrl = ctrl;
        }
      }
      __asm__ __volatile__("nop");
    }
    replay_end_ctrl = reg32p_read(&MODEM_WIFI_DUMP.MACTOADCDUMP0);
    sample_cycles = esp_cpu_get_cycle_count() - replay_start_cycle;
    reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                      MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
#if !CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF
    if (replay_pattern == 15u && ulp_completed != lp_completed_before) {
      live_probe_match_words = ulp_match_words;
    }
#endif
    if (replay_pattern == 12u || replay_pattern == 13u) {
      for (uint32_t word = 0u; word < waveform_words; ++word) {
        if (DUMP_BASE[word] ==
            s_s31_tx_live_probe_words[word & 15u]) {
          ++live_probe_match_words;
        }
      }
    }
  }
  }
  if (replay_aux_probe_reg != NULL) {
    reg32p_write(replay_aux_probe_reg, replay_aux_probe_original);
  }
  if (tcm_mem_enable_hold_probe) {
    reg32_write_addr(HP_SYSTEM_TCM_PERFORMACE_CTRL_REG,
                     tcm_performance_ctrl_original);
  }
  uint32_t replay_cycles = esp_cpu_get_cycle_count() - replay_start_cycle;
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  if (stall_peer_cpu && !txdc_without_replay) {
    esp_ipc_isr_release_other_cpu();
  }
  modem_stop_tx_replay();
  s_s31_tx_replay_burst_complete = true;
  s_s31_tx_replay_diag = (iq_tx_replay_diag_t){
      .words = one_shot ? upload_total_words : waveform_words,
      .rate_code = rate_code,
      .segments = deadline_missed ? 0u : segment_count,
      .total_cycles = replay_cycles,
      .sample_cycles = sample_cycles,
      .gap_cycles = gap_cycles_total,
      .maximum_gap_cycles = gap_cycles_max,
      .live_probe_write_cycles = live_probe_write_cycles,
      .live_probe_match_words = live_probe_match_words,
      .live_probe_dma_status = live_probe_dma_status,
      .deadline_late_max_cycles = live_probe_deadline_late_max,
      .live_probe_dma_ok = pipeline_dma_ok,
      .requested_start_time_ns = requested_start_time_ns,
      .actual_start_time_ns = actual_start_time_ns,
      .start_error_ns = requested_start_time_ns == 0u
                            ? 0
                            : deadline_missed
                                  ? 0
                                  : (int64_t)actual_start_time_ns -
                                        (int64_t)requested_start_time_ns,
      .deadline_missed = deadline_missed,
  };

  /* The cycle-exact worker intentionally avoids libc/logging.  A task's
   * first formatted log lazily allocates a newlib mutex from scarce internal
   * RAM; diagnostics are already published atomically through status. */
  if (!s_s31_tx_worker_mode) {
    ESP_LOGI("iq_capture",
           "TX replay burst complete: words=%" PRIu32 " rate_code=%" PRIu32
           " one_shot=%u segments=%" PRIu32 " cycles=%" PRIu32
           " sample_cycles=%" PRIu32 " gap_cycles=%" PRIu32
           " max_gap_cycles=%" PRIu32
           " pipeline=%u direct_dma=%u lane_refill=%u dma_ok=%u"
           " deadline_missed=%u"
           " ctrl=0x%08" PRIx32 " live_probe=%u gate_probe=%u"
           " write_cycles=%" PRIu32
           " ctrl_min=0x%08" PRIx32 " ctrl_max=0x%08" PRIx32
           " ctrl_changes=%" PRIu32 " deadline_late_max=%" PRIu32,
           upload_total_words,
           rate_code,
           one_shot ? 1u : 0u, segment_count, replay_cycles, sample_cycles,
           gap_cycles_total, gap_cycles_max, pipelined_batch ? 1u : 0u,
           direct_dma_batch ? 1u : 0u, lane_refill_batch ? 1u : 0u,
           pipeline_dma_ok ? 1u : 0u,
           deadline_missed ? 1u : 0u,
           replay_end_ctrl,
           live_write_probe ? 1u : 0u, live_gate_probe ? 1u : 0u,
           live_probe_write_cycles,
           live_probe_ctrl_min, live_probe_ctrl_max,
             live_probe_ctrl_changes, live_probe_deadline_late_max);
  }
}
#endif

typedef struct {
  uint32_t mode;
  uint32_t interval_chunks;
  uint32_t interval_offset_chunks;
  uint32_t interval_duration_chunks;
  uint32_t checks_per_chunk;
  uint32_t pre_chunks;
  uint32_t post_chunks;
  uint32_t max_gain;
  uint32_t state_mask;
  uint32_t threshold;
  uint32_t dc_shift;
  bool use_fsm_match;
  bool use_max_gain_match;
  uint16_t sample_offsets[IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK];
} trigger_plan_t;

_Static_assert(sizeof(capture_config_t) == 172u,
               "Python GUI CaptureConfig must match firmware");
_Static_assert(sizeof(iq_chunk_t) == 4152u,
               "Python GUI IqChunk must match firmware");

static TaskHandle_t s_producer_task_handle;
#if CONFIG_IDF_TARGET_ESP32S31
static volatile bool s_s31_tx_worker_pending;
static capture_config_t s_s31_tx_worker_config;
static TaskHandle_t s_s31_tx_worker_requester;
#endif

#if CONFIG_IDF_TARGET_ESP32S31
typedef struct {
  const uint8_t *data;
  uint32_t bytes;
  uint32_t sequence;
} s31_parlio_event_t;

static parlio_rx_unit_handle_t s_s31_parlio_unit;
static parlio_rx_delimiter_handle_t s_s31_parlio_delimiter;
static parlio_rx_delimiter_handle_t s_s31_parlio_delimiter_full;
static parlio_rx_delimiter_handle_t s_s31_parlio_delimiter_half;
static void *s_s31_parlio_internal_dma_reserve;
static s31_parlio_event_t s_s31_parlio_events[S31_PARLIO_EVENT_RING_SIZE];
static volatile uint32_t s_s31_parlio_event_head;
static uint32_t s_s31_parlio_event_tail;
static volatile uint32_t s_s31_parlio_callback_overruns;
static volatile uint32_t s_s31_parlio_discontinuities;
/* Descriptor completions and seven-frame output batches are deliberately
 * independent. Retain the unconsumed suffix of one copied ISR event so no
 * sample is discarded when a completion crosses a batch boundary. */
static s31_parlio_event_t s_s31_parlio_pending_event;
static uint32_t s_s31_parlio_pending_offset;
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
static const int8_t s_s31_diag_gpio[16] = {
    8,  9,  10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23,
};
#else
/* RGMII uses GPIO8..19, MDIO/MDC use GPIO5/6, and Wi-Fi reserves GPIO26..32. */
static const int8_t s_s31_diag_gpio[16] = {
    20, 21, 22, 23, 24, 25, 33, 34,
    36, 37, 38, 39, 40, 42, 43, 44,
};
#endif
#endif

static uint32_t s_isr_prev_sa;    /* ISR: previous store_addr (wrap detect) */
static uint32_t s_isr_wrap_count; /* ISR: number of store_addr wraps */
static volatile uint32_t s_dbg_lo;

static uint32_t s_emit_chunk; /* next absolute chunk index to emit (cursor) */
static bool s_s31_ring_started;
static uint8_t s_chunk_mark[2u * IQ_CHUNKS_PER_BANK];

static volatile uint32_t s_stream_dropped_chunks;
static volatile uint32_t s_source_chunk_index;
static volatile bool s_producer_active;
static volatile bool s_stream_write_active;
static volatile bool s_network_capture_armed;
#if CONFIG_IDF_TARGET_ESP32S31
static volatile uint32_t s_s31_copy_ctrl_diag;
static volatile uint32_t s_s31_copy_mode_before_diag;
static volatile uint32_t s_s31_copy_mode_after_diag;
static volatile uint32_t s_s31_copy_physical_chunk;
static volatile uint32_t s_s31_copy_source_chunk;
static volatile uint64_t s_s31_diag_abs_write;
static volatile uint32_t s_s31_diag_c_hi;
static volatile uint32_t s_s31_diag_emit_chunk;
static volatile uint32_t s_s31_diag_ring_started;
static uint64_t s_s31_abs_write_accum;
static int64_t s_s31_abs_track_us;
static uint32_t s_s31_decim_buf[IQ_CHUNK_SAMPLE_WORDS];
static bool s_s31_stage_valid;
static uint32_t s_s31_stage_source_chunk;
static uint32_t s_s31_stage_prev_end_raw;
static bool s_s31_stage_have_prev_end;
static uint32_t s_s31_micro_gate_advances;
static uint32_t s_s31_micro_gate_max_cycles;
static uint32_t s_s31_micro_gate_batches;
static uint32_t s_s31_cursor_min_backlog;
static uint32_t s_s31_cursor_max_backlog;
static uint32_t s_s31_cursor_batches;
static uint32_t s_s31_cursor_last_writer_cycle;
static uint32_t s_s31_cursor_last_backlog;
static bool s_s31_cursor_have_writer_cycle;
static uint32_t s_s31_cursor_overruns;
static uint32_t s_s31_probe_source_chunk;
static uint32_t s_s31_probe_end_cycle;
static bool s_s31_probe_have_end_cycle;
static uint32_t s_s31_probe_logged_selector = UINT32_MAX;
#endif
static bool s_power_trigger_dc_valid;
static int32_t s_power_trigger_dc_i_q16;
static int32_t s_power_trigger_dc_q_q16;

static portMUX_TYPE s_config_mux = portMUX_INITIALIZER_UNLOCKED;
#if CONFIG_IDF_TARGET_ESP32S31
static portMUX_TYPE s_tcm_dump_gate_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

static capture_config_t s_config = {
    .stream = {.stream_wifi_packets = 0},
    .radio = {.rf_freq_hz = RF_FREQ_DEFAULT_HZ, .frequency_correction_ppb = 0},
    .gain = {.gain_mode = GAIN_MODE_MANUAL,
             .rx_gain = 32,
             .tx_gain = TX_GAIN_DEFAULT},
    .bandwidth = {.bw_mhz = 20, .second_chan = SECOND_CHAN_NONE},
    .loopback = {.loopback = 0,
                 .loopback_tx_gain = 124,
                 .loopback_rx_gain = 0x73,
                 .loopback_bb_gain = 0x3f},
    .tx = {.tx_tone_enable = 0, .tx_tone0_step = 16},
    .iq_engine = {.adc_decimation = 1,
                  .adc_source_sel = ADC_DUMP_SOURCE_CONTINUOUS_RX_IQ},
    /* interval in CHUNKS. Keep this coprime with the S31 31-slot dump ring so
     * displayed chunks walk slots instead of reusing one physical slot forever.
     */
    .trigger = {.trigger_mode = IQ_TRIGGER_MODE_INTERVAL,
                .trigger_config = {2501u, 0u, 1u}},
    .rx_filter = {.filter_bw_mhz = S31_DEFAULT_FILTER_BW_MHZ,
                  .rx_filter_override = 0,
                  .rx_filter_mode = 16,
                  .rx_filter_dcap = RX_FILTER_NARROW_DCAP},
    .wifi_tx = {.wifi_dummy_tx_enable = 0, .wifi_dummy_tx_interval_ms = 1000},
    .dc_offset = {.automatic = 1},
};
static trigger_plan_t s_trigger_plan;
static capture_config_t s_pending_config;
static trigger_plan_t s_pending_trigger_plan;
static volatile bool s_config_apply_pending;
static volatile bool s_config_apply_enabled;
static volatile bool s_config_apply_in_progress;
static volatile int64_t s_config_apply_started_us;
#if CONFIG_IDF_TARGET_ESP32S31
/* A newly adopted authenticated batch may deliberately reapply an otherwise
 * identical replay configuration. */
static bool s_s31_tx_replay_force_rearm;
#endif

/* Source-side IQC8 is disabled on S31: the closed staging view contains the
 * poison sentinel. Ethernet quantizes a proven IQC1 frame in its transport
 * task; high-speed USB retains IQC1 on the wire. */
static inline bool stream_output_int8(void) {
  return false;
}

static void apply_pending_config(void);
static void service_pending_config(void);
static bool config_apply_in_progress(void);
static bool capture_engine_running(const capture_config_t *config);
static trigger_plan_t build_trigger_plan(const capture_config_t *config);
static uint32_t adc_dump_store_addr_raw(void);
#if CONFIG_IDF_TARGET_ESP32S31
static inline bool s31_continuous_real_if_selected(
    const capture_config_t *config) {
#if CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF && CONFIG_ESP_SDR_TRANSPORT_USB && \
    !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  return config->rx_filter.rx_filter_override == 0u;
#else
  (void)config;
  return false;
#endif
}

static inline bool s31_production_native_iq_selected(
    const capture_config_t *config) {
#if CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF && CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  return config->rx_filter.rx_filter_override == 0u;
#else
  (void)config;
  return false;
#endif
}

static capture_config_t
s31_engine_config(const capture_config_t *config) {
  capture_config_t engine = *config;
  if (s31_continuous_real_if_selected(config)) {
    engine.rx_filter.rx_filter_override = S31_GPIO_DIAG_OVERRIDE;
    engine.rx_filter.rx_filter_mode = S31_CONTINUOUS_DIAG_MODE;
  } else if (s31_production_native_iq_selected(config)
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
             || config->rx_filter.rx_filter_override ==
                    S31_PARLIO_HOST_IQ_OVERRIDE
#endif
  ) {
    /* Mode 62 is the legacy SoapyESPSDR request.  It must use the same modem
     * formatter/filter setup as production mode 0; changing only the GPIO
     * route leaves a single tone recognizable but corrupts broadband IQ. */
    engine.rx_filter.rx_filter_override = S31_PARLIO_NATIVE_IQ_OVERRIDE;
    engine.rx_filter.rx_filter_mode = S31_PARLIO_NATIVE_PACKED_IQ;
  }
  return engine;
}

static bool s31_pulse_tcm_before_copy(const capture_config_t *config);
static void s31_pulse_tcm_dump_gate(const capture_config_t *config);
#endif

/* ---- config -> subsystem mappers ---- */

static void init_idf_services(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  } else {
    ESP_ERROR_CHECK(ret);
  }

  ret = esp_netif_init();
  if (ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(ret);
  }

  ret = esp_event_loop_create_default();
  if (ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(ret);
  }
}

#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_ESP_PHY_ENABLE_CERT_TEST
static void init_s31_rftest_services(void) {
  esp_wifi_power_domain_on();
  esp_phy_rftest_config(1);
}
#endif

static wifi_tx_rx_config_t
wifi_tx_rx_config_from_capture(const capture_config_t *config) {
  return (wifi_tx_rx_config_t){
      .rf_freq_hz = config->radio.rf_freq_hz,
      .bw_mhz = config->bandwidth.bw_mhz,
      .second_chan = config->bandwidth.second_chan,
      .stream_packets = config->stream.stream_wifi_packets,
      .dummy_tx_enable = config->wifi_tx.wifi_dummy_tx_enable,
      .dummy_tx_interval_ms = config->wifi_tx.wifi_dummy_tx_interval_ms,
  };
}

static modem_config_t
modem_config_from_capture(const capture_config_t *config) {
  uint32_t hardware_rf_hz = config->radio.rf_freq_hz;
#if CONFIG_IDF_TARGET_ESP32S31
  if (s31_continuous_real_if_selected(config)) {
    hardware_rf_hz = hardware_rf_hz > RF_FREQ_MIN_HZ + S31_CONTINUOUS_IF_HZ
                         ? hardware_rf_hz - S31_CONTINUOUS_IF_HZ
                         : RF_FREQ_MIN_HZ;
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  } else if (config->rx_filter.rx_filter_override ==
             S31_PARLIO_HOST_IQ_OVERRIDE) {
    /* The host selects the positive-frequency half of the real ADC stream,
     * translates this Fs/4 low IF to baseband, and decimates by two. */
    uint32_t host_rate = S31_PARLIO_HOST_REAL_RATE_HZ;
#if !CONFIG_ESP_SDR_TRANSPORT_USB
    if ((config->rx_filter.rx_filter_mode & S31_PARLIO_HOST_SYNC_40M) != 0u) {
      host_rate = S31_PARLIO_HOST_SYNC_RATE_HZ;
    }
#endif
    uint32_t host_if_hz = host_rate / 4u;
    hardware_rf_hz = hardware_rf_hz > RF_FREQ_MIN_HZ + host_if_hz
                         ? hardware_rf_hz - host_if_hz
                         : RF_FREQ_MIN_HZ;
#endif
  }
#endif
  return (modem_config_t){
      .rf_freq_hz = hardware_rf_hz,
      .frequency_correction_ppb = config->radio.frequency_correction_ppb,
      .bw_mhz = config->bandwidth.bw_mhz,
      .second_chan = config->bandwidth.second_chan,
      .gain_mode = config->gain.gain_mode,
      .rx_gain = config->gain.rx_gain,
      .tx_gain = config->gain.tx_gain,
      .expert_gain_word0 = config->gain.expert_gain_word0,
      .expert_gain_word1 = config->gain.expert_gain_word1,
      .expert_gain_word2 = config->gain.expert_gain_word2,
      .loopback = config->loopback.loopback,
      .loopback_tx_gain = config->loopback.loopback_tx_gain,
      .loopback_rx_gain = config->loopback.loopback_rx_gain,
      .loopback_bb_gain = config->loopback.loopback_bb_gain,
      .tx_tone_enable = config->tx.tx_tone_enable,
      .tx_tone0_step = config->tx.tx_tone0_step,
      .filter_bw_mhz = config->rx_filter.filter_bw_mhz,
      .rx_filter_override = config->rx_filter.rx_filter_override,
      .rx_filter_mode = config->rx_filter.rx_filter_mode,
      .rx_filter_dcap = config->rx_filter.rx_filter_dcap,
      .dc_offset_automatic = config->dc_offset.automatic,
  };
}

static inline int32_t sign_extend_10_u32(uint32_t value) {
  value &= 0x3ffu;
  return (value & 0x200u) != 0u ? (int32_t)value - 0x400 : (int32_t)value;
}

static inline uint32_t abs_i32_u32(int32_t value) {
  return (uint32_t)(value < 0 ? -value : value);
}

/* ---- ADC dump engine ---- */

static void IRAM_ATTR engine_own_bank(uint32_t bank) {
#if CONFIG_IDF_TARGET_ESP32S31
  (void)bank;
  /*
   * ESP32-S31 does not expose the C61 HP_SYSTEM_SRAM_USAGE_CONF_REG. Its
   * HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG is a dump gate used by S31 rftest
   * adctrig, not a per-bank mac-dump SRAM ownership selector.
   */
  __asm__ __volatile__("fence" ::: "memory");
  (void)reg32_read_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG);
#else
  (void)bank;
#endif
}

#if CONFIG_IDF_TARGET_ESP32S31
static void s31_poison_dump_window(void) {
  for (uint32_t i = 0u; i < DUMP_BANK_WORDS; ++i) {
    DUMP_BASE[i] = S31_POISON_WORD ^ i;
  }
  __asm__ __volatile__("fence" ::: "memory");
  (void)esp_cache_msync((void *)DUMP_BASE, DUMP_BANK_WORDS * sizeof(uint32_t),
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                            ESP_CACHE_MSYNC_FLAG_INVALIDATE);
}
#endif

static uint32_t adc_decimation_value(const capture_config_t *config) {
  uint32_t decimation = config->iq_engine.adc_decimation;
  if (decimation == 0u) {
    return 1u;
  }
  if (decimation > ADC_DECIMATION_MAX) {
    return ADC_DECIMATION_MAX;
  }
  return decimation;
}

#if CONFIG_IDF_TARGET_ESP32S31
static uint32_t s31_hardware_decimation_factor(
    const capture_config_t *config) {
  uint32_t field =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_HW_DECIM_M) >>
      S31_ADC_SOURCE_HW_DECIM_S;
  /* Differential Pluto-tone measurements identify the useful lower-rate
   * hardware divisors.  All retain the same densely packed IQ word layout. */
  switch (field) {
  case 7u:
    return 10u; /* 8.000 MS/s */
  case 8u:
    return 12u; /* 6.667 MS/s */
  case 9u:
    return 20u; /* 4.000 MS/s */
  case 10u:
    return 24u; /* 3.333 MS/s */
  default:
    return 1u;
  }
}

static uint32_t s31_software_decimation(const capture_config_t *config) {
  if (s31_hardware_decimation_factor(config) != 1u) {
    return 1u;
  }
  return adc_decimation_value(config);
}
#endif

static uint32_t adc_dump_write_sample_cycles(const capture_config_t *config) {
#if CONFIG_IDF_TARGET_ESP32S31
  return 2u * s31_hardware_decimation_factor(config);
#else
  (void)config;
  return 2u;
#endif
}

static uint32_t adc_dump_sample_cycles(const capture_config_t *config) {
#if CONFIG_IDF_TARGET_ESP32S31
  return adc_dump_write_sample_cycles(config) *
         s31_software_decimation(config);
#else
  return 2u * adc_decimation_value(config);
#endif
}

#if CONFIG_IDF_TARGET_ESP32S31
static void s31_enable_data_dump_clocks(const capture_config_t *config) {
  (void)config;
  reg32_set_bits_addr(HP_SYSTEM_TCM_RAM_PWR_CTRL0_REG,
                      HP_SYSTEM_REG_HP_SYSTEM_TCM_CLK_FORCE_ON);
  reg32p_set_bits(&MODEM_SYSCON.CLK_CONF, BIT(31) | BIT(21));
  reg32p_set_bits(&MODEM_SYSCON.CLK_CONF_FORCE_ON, BIT(31));
  reg32p_set_bits(&MODEM_SYSCON.CLK_CONF1, 0x0000e400u);
  reg32p_write_field(&MODEM_SYSCON.CLK_CONF_POWER_ST, 0x0000f000u, 12u, 4u);
  reg32p_write_field(&MODEM_SYSCON.CLK_CONF_POWER_ST, 0x0f000000u, 24u, 4u);
  reg32p_write_field(&MODEM_SYSCON.CLK_CONF_POWER_ST, 0xf0000000u, 28u, 4u);
  reg32p_set_bits(&MODEM_SYSCON.MODEM_RST_CONF, BIT(31));
  reg32p_clear_bits(&MODEM_SYSCON.MODEM_RST_CONF, BIT(31));
}
#endif

#if CONFIG_IDF_TARGET_ESP32S31
static void s31_configure_modem_diag(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  if (!((expert >= S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST &&
         expert <= S31_MODEM_DIAG_PROBE_OVERRIDE_LAST) ||
        expert == S31_GPIO_DIAG_OVERRIDE ||
        expert == S31_PARLIO_NATIVE_IQ_OVERRIDE ||
        expert == S31_PARLIO_HOST_IQ_OVERRIDE)) {
    reg32_write_addr(HP_SYSTEM_MODEM_DIAG_EN_REG, 0u);
    return;
  }
  uint32_t diagnostic_mode = config->rx_filter.rx_filter_mode;
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  if (expert == S31_PARLIO_HOST_IQ_OVERRIDE) {
    /* SoapyESPSDR versions that predate native I/Q request expert mode 62.
     * Keep them wire-compatible, but feed the new native complex route. */
    expert = S31_PARLIO_NATIVE_IQ_OVERRIDE;
    diagnostic_mode = S31_PARLIO_NATIVE_PACKED_IQ;
  }
#else
  if (expert == S31_PARLIO_HOST_IQ_OVERRIDE) {
    /* Reuse the proven production real-IF route. Only the eight ADC MSBs are
     * sampled below; no per-sample conversion is performed on the MCU. */
    expert = S31_GPIO_DIAG_OVERRIDE;
    diagnostic_mode = S31_CONTINUOUS_DIAG_MODE;
  }
#endif
  uint32_t selector = diagnostic_mode & 31u;
  uint32_t variant = (expert == S31_GPIO_DIAG_OVERRIDE ||
                      expert == S31_PARLIO_NATIVE_IQ_OVERRIDE)
                         ? 0u
                         : (expert - S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST) / 2u;
  uint32_t exchange =
      (variant == 1u || variant == 3u) ? 0x0123u : 0x3210u;
  if (expert == S31_GPIO_DIAG_OVERRIDE &&
      (diagnostic_mode & BIT(23)) != 0u) {
    uint32_t byte0 = (diagnostic_mode >> 15u) & 15u;
    uint32_t byte1 = (diagnostic_mode >> 19u) & 15u;
    uint32_t byte2 = (diagnostic_mode >> 24u) & 15u;
    uint32_t byte3 = (diagnostic_mode >> 28u) & 15u;
    exchange = (byte3 << 12u) | (byte2 << 8u) | (byte1 << 4u) | byte0;
  }
  if (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE) {
    selector = S31_CONTINUOUS_DIAG_MODE & 31u;
    if ((diagnostic_mode & BIT(31)) != 0u) {
      exchange = (diagnostic_mode & BIT(30)) != 0u
                     ? (diagnostic_mode >> 8u) & 0xffffu
                     : 0x3210u;
    } else {
      exchange = 0x32f2u;
    }
  }
  bool gpio_high_half = expert == S31_GPIO_DIAG_OVERRIDE &&
                        (diagnostic_mode & BIT(14)) != 0u;
  uint32_t fix_sel = gpio_high_half
                         ? selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S
                         : selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_LOW_S;
  if (variant == 2u || variant == 3u) {
    fix_sel |= selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S;
  } else if (variant == 4u) {
    fix_sel = selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S;
  }
  bool gpio_dynamic_exchange = expert == S31_GPIO_DIAG_OVERRIDE &&
                               (diagnostic_mode & BIT(13)) != 0u;
  if (variant != 4u && !gpio_high_half && !gpio_dynamic_exchange) {
    fix_sel |= MODEM_WIDGETS_DIAG_BUS_FIX_LOW_EN;
  }
  if (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE) {
    uint32_t high_selector = (diagnostic_mode >> 9u) & 31u;
    if ((diagnostic_mode & BIT(31)) != 0u) {
      /* Exchange-search mode: leave the low half on the byte exchange.  The
       * optional full-low route (bit 30) observes both exchanged bytes; the
       * older mixed route keeps the high half fixed for diagnostic A/Bs. */
      high_selector = (diagnostic_mode & BIT(30)) != 0u
                          ? selector
                          : (diagnostic_mode >> 8u) & 31u;
      fix_sel = high_selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S;
    } else {
      /* Fixed-half census mode. */
      fix_sel = MODEM_WIDGETS_DIAG_BUS_FIX_LOW_EN |
                (selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_LOW_S) |
                (high_selector << MODEM_WIDGETS_DIAG_BUS_FIX_SEL_HIGH_S);
    }
  }
  reg32_set_bits_addr(HP_SYSTEM_CLK_EN_REG, HP_SYSTEM_REG_CLK_EN);
  reg32_set_bits_addr(HP_SYSTEM_PROBEA_CTRL_REG,
                      HP_SYSTEM_REG_PROBE_GLOBAL_EN);
  reg32_set_bits_addr(MODEM_WIDGETS_CLK_CONF_REG, MODEM_WIDGETS_CLK_EN);
  reg32_write_addr(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG, exchange);
  reg32_write_addr(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG, fix_sel);
  if (expert == S31_GPIO_DIAG_OVERRIDE ||
      expert == S31_PARLIO_NATIVE_IQ_OVERRIDE) {
    uint32_t bb_diag = reg32p_read(&MODEM_WIFI_BB.BB_DIAG0);
    bb_diag &= ~(0xffu | BIT(30));
    bb_diag |= (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE
                    ? ((diagnostic_mode & BIT(31)) != 0u
                           ? ((diagnostic_mode >> 24u) & 0x07u) |
                                 (diagnostic_mode & BIT(3)) |
                                 (((diagnostic_mode >> 4u) & 0x0fu) << 4u)
                           : diagnostic_mode >> 14u)
                    : (diagnostic_mode >> 5u)) &
               0xffu;
    if ((expert == S31_GPIO_DIAG_OVERRIDE &&
         (diagnostic_mode & BIT(13)) != 0u) ||
        (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE &&
         (diagnostic_mode & BIT(31)) != 0u)) {
      /* BB_DIAG0[30] is the undocumented enable for the widgets byte
       * exchange.  Without it, MODEM_DIAG_EXCHANGE reads back correctly but
       * the dynamic low half remains on its reset/default sources. */
      bb_diag |= BIT(30);
    }
    reg32p_write(&MODEM_WIFI_BB.BB_DIAG0, bb_diag);
  }
  uint32_t diag_enable =
      gpio_high_half ? 0xffff0000u : 0x0000ffffu;
  if (expert == S31_PARLIO_NATIVE_IQ_OVERRIDE) {
    /* The S31 substitution gate is slice-wide in practice: enabling only the
     * eight routed bits leaves LCD_DATA on the matrix on rev-0 silicon. */
    diag_enable = (diagnostic_mode & BIT(30)) != 0u
                      ? 0x0000ffffu
                      : 0xffffffffu;
  }
  reg32_write_addr(HP_SYSTEM_MODEM_DIAG_EN_REG, diag_enable);
}
#endif

static void adctrig_prepare(uint32_t sample_count,
                            const capture_config_t *config) {
  uint32_t source_sel = config->iq_engine.adc_source_sel;
#if CONFIG_IDF_TARGET_ESP32S31
  source_sel &= S31_ADC_DUMP_SOURCE_SEL_MASK;
#endif
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_clear_bits_addr(PHY_MODEM_BASE_ADDR + 0x08ccu, 0x00780000u);
  reg32_clear_bits_addr(PHY_MODEM_BASE_ADDR + 0x70b8u, 0x7u);
  reg32_set_bits_addr(PHY_MODEM_BASE_ADDR + 0x70b8u, BIT(0));
  reg32_set_bits_addr(MODEM_WIFI_S31_ADCTRIG_SETUP_REG,
                      MODEM_WIFI_S31_ADCTRIG_SETUP_BIT);
  reg32_write_addr(MODEM_WIFI_S31_ADCTRIG_CFG_REG, UINT32_MAX);
#endif
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
  uint32_t hardware_decimation_field =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_HW_DECIM_M) >>
      S31_ADC_SOURCE_HW_DECIM_S;
#if CONFIG_IDF_TARGET_ESP32S31
  /* This register-only reset must occur while the S31 dump fabric is open;
   * otherwise the formatter/source setup does not latch into the active dump
   * path. PHY blob and I2C calls remain outside this transaction. */
  s31_enable_data_dump_clocks(config);
#else
  MODEM_SYSCON.CLK_CONF = 0xffffffffu;
#endif
  /* S31 clock bring-up pulses the modem reset, which clears ADC_DUMP_MODE.
   * Program the latched rate field only after that reset has completed. */
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_MODE,
                     MODEM_WIFI_DUMP_MODE_DECIM_M, MODEM_WIFI_DUMP_MODE_DECIM_S,
                     hardware_decimation_field);
  reg32p_set_bits(&MODEM_WIFI_FE_CTRL.CLK_ENABLE, 0x00000004u);

  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                     MODEM_WIFI_DUMP_CTRL_SAMPLE_COUNT_M,
                     MODEM_WIFI_DUMP_CTRL_SAMPLE_COUNT_S, sample_count);
  reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                  MODEM_WIFI_DUMP_CTRL_DONE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_DONE_BIT);

  uint32_t byte_sel[4] = {24u, 25u, 26u, 27u};
  const bool vendor_mode11_pack =
      config->rx_filter.rx_filter_override == 14u ||
      config->rx_filter.rx_filter_override == 15u;
  if (vendor_mode11_pack) {
    /* Exact S31 librftest high-level dump mode 11 packing.  Its source-map
     * branch separately selects raw ADC_DUMP_MODE source 11. */
    byte_sel[0] = 20u;
    byte_sel[1] = 21u;
    byte_sel[2] = 22u;
    byte_sel[3] = 23u;
  } else if (config->rx_filter.rx_filter_override == 30u ||
             config->rx_filter.rx_filter_override == 31u ||
             config->rx_filter.rx_filter_override == 32u ||
             config->rx_filter.rx_filter_override == 33u) {
    /* Stable candidate pair from the full BT-loopback census.  Keep filter
     * control in rx_filter_mode while exposing selectors 8/9 directly. */
    byte_sel[0] = 8u;
    byte_sel[1] = 9u;
    byte_sel[2] = 8u;
    byte_sel[3] = 9u;
  } else if ((config->rx_filter.rx_filter_override >= 2u &&
              config->rx_filter.rx_filter_override < 8u) ||
             config->rx_filter.rx_filter_override == 28u ||
             config->rx_filter.rx_filter_override == 29u) {
    uint32_t pair = config->rx_filter.rx_filter_mode;
    if (pair < 32u) {
      /* Diagnostic pair scan: expose every adjacent pair in the 64-byte
       * adctrig debug bus through word bytes 0/1.  Repeat it in bytes 2/3 so
       * both the direct-byte and raw-word readers see the same candidate. */
      byte_sel[0] = pair * 2u;
      byte_sel[1] = pair * 2u + 1u;
      byte_sel[2] = byte_sel[0];
      byte_sel[3] = byte_sel[1];
    } else {
      /* Exact S31 librftest adctrig mode-12 Bluetooth packing. */
      byte_sel[0] = 20u;
      byte_sel[1] = 21u;
      byte_sel[2] = 8u;
      byte_sel[3] = 11u;
    }
  } else if (config->rx_filter.rx_filter_override == 34u ||
             config->rx_filter.rx_filter_override == 35u) {
    /* The real BTRX-path census found activity only on selectors
     * 8,9,20,21,24,25.  Adjacent pairs have poor image rejection, so test
     * every ordered cross-combination instead of assuming I and Q are
     * adjacent on the debug bus. */
    static const uint8_t active_sel[6] = {8u, 9u, 20u, 21u, 24u, 25u};
    uint32_t pair = config->rx_filter.rx_filter_mode;
    byte_sel[0] = active_sel[(pair / 6u) % 6u];
    byte_sel[1] = active_sel[pair % 6u];
    byte_sel[2] = byte_sel[0];
    byte_sel[3] = byte_sel[1];
  } else if (config->rx_filter.rx_filter_override == 36u ||
             config->rx_filter.rx_filter_override == 37u) {
    /* Normal Wi-Fi antenna route, exhaustive adjacent debug-byte pair. */
    uint32_t pair = config->rx_filter.rx_filter_mode & 31u;
    byte_sel[0] = pair * 2u;
    byte_sel[1] = pair * 2u + 1u;
    byte_sel[2] = byte_sel[0];
    byte_sel[3] = byte_sel[1];
  }
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                     MODEM_WIFI_DUMP_CFG_BYTE3_SEL_M,
                     MODEM_WIFI_DUMP_CFG_BYTE3_SEL_S, byte_sel[3]);
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                     MODEM_WIFI_DUMP_CFG_BYTE2_SEL_M,
                     MODEM_WIFI_DUMP_CFG_BYTE2_SEL_S, byte_sel[2]);
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                     MODEM_WIFI_DUMP_CFG_BYTE1_SEL_M,
                     MODEM_WIFI_DUMP_CFG_BYTE1_SEL_S, byte_sel[1]);
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                     MODEM_WIFI_DUMP_CFG_BYTE0_SEL_M,
                     MODEM_WIFI_DUMP_CFG_BYTE0_SEL_S, byte_sel[0]);
  if (config->rx_filter.rx_filter_override ==
          S31_SEAMLESS_MICRO_GATE_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_GPIO_DIAG_OVERRIDE ||
      config->rx_filter.rx_filter_override ==
          S31_PARLIO_NATIVE_IQ_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_PARLIO_HOST_IQ_OVERRIDE) {
    reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                    MODEM_WIFI_DUMP_CFG_AGC_DBG_EN);
  } else if (config->rx_filter.rx_filter_override >= 2u &&
      (config->rx_filter.rx_filter_override & 1u) == 0u) {
    reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                      MODEM_WIFI_DUMP_CFG_AGC_DBG_EN);
  } else {
    /* The production 24/25/26/27 IQ selectors require the debug formatter.
     * Override parity is an expert A/B control only; never let calibrated
     * override 0 disable the normal antenna producer. */
    reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CFG,
                    MODEM_WIFI_DUMP_CFG_AGC_DBG_EN);
  }

  /* engine owns bank 0 to start; CPU owns bank 1. */
  engine_own_bank(0u);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_RESERVED_MODE_M);
  /* Continuous mode maps the writer into the reserved streaming aperture.
   * The vendor's finite Bluetooth test route clears this bit and writes from
   * TCM offset zero, which overlaps the application in an IDF firmware. */
  reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                  MODEM_WIFI_DUMP_CTRL_CONTINUOUS_TRIGGER_GATE_BIT);

#if !CONFIG_IDF_TARGET_ESP32S31
  reg32p_set_bits(&MODEM_WIFI_FE_TRIGGER.FE_DUMP_TRIGGER_CTRL,
                  MODEM_WIFI_FE_DUMP_TRIGGER_CTRL_ENABLE);
  reg32p_write_field(&MODEM_WIFI_FE_TRIGGER.FE_DUMP_TRIGGER_CTRL,
                     MODEM_WIFI_FE_DUMP_TRIGGER_CTRL_ARG_M, 0u, 0u);
#endif

#if !CONFIG_IDF_TARGET_ESP32S31
  reg32p_clear_bits(&MODEM_WIFI_DUMP_MISC.DUMP_SOURCE_GATE,
                    MODEM_WIFI_DUMP_SOURCE_GATE_EN);
#endif
  reg32p_write_field(&MODEM_WIFI_DUMP.ADC_DUMP_MODE,
                     MODEM_WIFI_DUMP_MODE_SOURCE_SEL_M,
                     MODEM_WIFI_DUMP_MODE_SOURCE_SEL_S, source_sel);

  /* The S31 exposes the modem's live diagnostic bus independently of TCM. */
  s31_configure_modem_diag(config);

  if (config->rx_filter.rx_filter_override == S31_LP_CORE_PROBE_OVERRIDE) {
    /* Route all four bytes of one LP-core internal signal group to the LP
     * probe output.  Group 15 is documented as the target identifier; the
     * remaining groups are swept on real hardware to find the load/GPR bus. */
    uint32_t group = config->rx_filter.rx_filter_mode & 15u;
    uint32_t mod_sel = group * 0x1111u;
    uint32_t ctrl = mod_sel |
                    (3u << LP_SYSTEM_REG_PROBE_A_TOP_SEL_S) |
                    (1u << LP_SYSTEM_REG_PROBE_H_SEL_S) |
                    LP_SYSTEM_REG_PROBE_GLOBAL_EN;
    reg32_write_addr(LP_SYSTEM_REG_LP_PROBEB_CTRL_REG, 0u);
    reg32_write_addr(LP_SYSTEM_REG_LP_PROBEA_CTRL_REG, ctrl);
  } else {
    reg32_write_addr(LP_SYSTEM_REG_LP_PROBEA_CTRL_REG, 0u);
    reg32_write_addr(LP_SYSTEM_REG_LP_PROBEB_CTRL_REG, 0u);
  }

  if (config->rx_filter.rx_filter_override == S31_HP_TCM_PROBE_OVERRIDE) {
    /* Observe the TCM controller itself, rather than attempting a memory
     * access through a CPU/DMA master.  If its write-data group is exposed,
     * this remains live while the modem owns the aperture. */
    uint32_t mode = config->rx_filter.rx_filter_mode;
    uint32_t group = mode & 15u;
    uint32_t mod_sel = group * 0x1111u;
    if ((mode & BIT(23)) != 0u) {
      /* Expert census mode: reuse the modem-probe script's four-nibble
       * encoding so each byte of probe channel A can select an independent
       * TCM signal group.  Only byte 0/1 reach GPIO in the lower-16 route,
       * but retaining all four makes the register readback unambiguous. */
      mod_sel = ((mode >> 15u) & 0x0fu) |
                (((mode >> 19u) & 0x0fu) << 4u) |
                (((mode >> 24u) & 0x0fu) << 8u) |
                (((mode >> 28u) & 0x0fu) << 12u);
    }
    uint32_t ctrl = mod_sel |
                    (3u << HP_SYSTEM_REG_PROBE_A_TOP_SEL_S) |
                    (1u << HP_SYSTEM_REG_PROBE_H_SEL_S) |
                    HP_SYSTEM_REG_PROBE_GLOBAL_EN;
    reg32_write_addr(HP_SYSTEM_PROBEB_CTRL_REG, 0u);
    reg32_write_addr(HP_SYSTEM_PROBEA_CTRL_REG, ctrl);
  }
}

static inline uint32_t IRAM_ATTR adc_dump_store_addr_raw(void) {
  return (MODEM_WIFI_DUMP.ADC_DUMP_MODE & MODEM_WIFI_DUMP_MODE_STORE_ADDR_M) >>
         MODEM_WIFI_DUMP_MODE_STORE_ADDR_S;
}

static inline uint32_t IRAM_ATTR adc_dump_store_addr_from_raw(uint32_t addr) {
  return addr;
}

static inline uint32_t IRAM_ATTR adc_dump_store_addr(void) {
  return adc_dump_store_addr_from_raw(adc_dump_store_addr_raw());
}

static inline uint32_t IRAM_ATTR dump_ring_mod(uint32_t value) {
  return value % DUMP_BANK_WORDS;
}

static inline uint32_t IRAM_ATTR dump_ring_delta(uint32_t cur, uint32_t prev) {
  return (cur + DUMP_BANK_WORDS - prev) % DUMP_BANK_WORDS;
}

static bool IRAM_ATTR s31_parlio_partial_receive(
    parlio_rx_unit_handle_t unit, const parlio_rx_event_data_t *event,
    void *user_data) {
  (void)unit;
  (void)user_data;
  uint32_t sequence = s_s31_parlio_event_head;
  s31_parlio_event_t *slot =
      &s_s31_parlio_events[sequence % S31_PARLIO_EVENT_RING_SIZE];
  uint32_t bytes = event->recv_bytes;
  if (bytes <= S31_PARLIO_EVENT_MAX_BYTES) {
    /* Publish the completed circular GDMA node itself. The producer retains
     * ownership until it has consumed every byte, avoiding a full
     * PSRAM-to-PSRAM copy in this ISR. */
    slot->data = event->data;
    slot->bytes = bytes;
  } else {
    slot->data = NULL;
    slot->bytes = 0u;
    ++s_s31_parlio_callback_overruns;
  }
  slot->sequence = sequence;
  __atomic_store_n(&s_s31_parlio_event_head, sequence + 1u,
                   __ATOMIC_RELEASE);
  BaseType_t task_woken = pdFALSE;
  if (s_producer_task_handle != NULL) {
    vTaskNotifyGiveFromISR(s_producer_task_handle, &task_woken);
  }
  return task_woken == pdTRUE;
}

static uint32_t s31_parlio_diag_rate_hz(const capture_config_t *config) {
  uint32_t divider = adc_decimation_value(config);
  return S31_PARLIO_DIAG_RATE_HZ / divider;
}

static bool s31_parlio_host_iq_selected(const capture_config_t *config) {
#if CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  return config->rx_filter.rx_filter_override ==
         S31_PARLIO_HOST_IQ_OVERRIDE;
#else
  (void)config;
  return false;
#endif
}

static bool s31_parlio_native_packed_iq_selected(
    const capture_config_t *config) {
  return s31_production_native_iq_selected(config) ||
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
         config->rx_filter.rx_filter_override ==
             S31_PARLIO_HOST_IQ_OVERRIDE ||
#endif
         (config->rx_filter.rx_filter_override ==
              S31_PARLIO_NATIVE_IQ_OVERRIDE &&
          (config->rx_filter.rx_filter_mode &
           S31_PARLIO_NATIVE_PACKED_IQ) != 0u &&
          (config->rx_filter.rx_filter_mode & BIT(31)) == 0u);
}

static bool s31_parlio_usb_iq4_selected(const capture_config_t *config) {
#if CONFIG_ESP_SDR_TRANSPORT_USB
  return s31_production_native_iq_selected(config) &&
         iq_network_stream_owner() == IQ_STREAM_OWNER_USB &&
         iq_usb_stream_format() == IQ_USB_FORMAT_INT4;
#else
  (void)config;
  return false;
#endif
}

static bool s31_parlio_host_real_selected(const capture_config_t *config) {
  return s31_parlio_host_iq_selected(config) ||
         s31_continuous_real_if_selected(config);
}

static uint32_t s31_parlio_capture_rate_hz(const capture_config_t *config) {
  if (s31_continuous_real_if_selected(config)) {
    return S31_CONTINUOUS_INPUT_RATE_HZ;
  }
  if (!s31_parlio_host_real_selected(config)) {
    return s31_parlio_diag_rate_hz(config);
  }
#if !CONFIG_ESP_SDR_TRANSPORT_USB
  if ((config->rx_filter.rx_filter_mode & S31_PARLIO_HOST_SYNC_40M) != 0u) {
    return S31_PARLIO_HOST_SYNC_RATE_HZ;
  }
#endif
  return S31_PARLIO_HOST_REAL_RATE_HZ;
}

static uint32_t s31_gpio_diag_output_signal(const capture_config_t *config,
                                            uint32_t bit) {
  /* ESP32-S31 exposes the two halves of modem_diag[] through different GPIO
   * matrix sources.  HP_SYSTEM_MODEM_DIAG_EN[15:0] substitutes the low half
   * onto HP_PROBE_TOP_OUT[15:0], while [31:16] substitutes the high half onto
   * LCD_DATA_OUT[15:0] (see hp_system_reg.h). */
  bool high_half = config->rx_filter.rx_filter_override ==
                       S31_GPIO_DIAG_OVERRIDE &&
                   (config->rx_filter.rx_filter_mode & BIT(14)) != 0u;
  if (high_half) {
    return LCD_DATA_OUT_PAD_OUT0_IDX + bit;
  }
  if (s31_parlio_usb_iq4_selected(config)) {
    /* Capture one complex sample per byte without a CPU pack pass. The low
     * nibble is signed I[9:6] from modem_diag[19:16], and the high nibble is
     * signed Q[9:6] from modem_diag[9:6]. */
    if (bit < 4u) {
      return LCD_DATA_OUT_PAD_OUT0_IDX + bit;
    }
    uint32_t source_bit = bit + 2u;
    return source_bit < 8u ? HP_PROBE_TOP_OUT0_IDX + source_bit
                           : HP_PROBE_TOP_OUT8_IDX + source_bit - 8u;
  }
  if (config->rx_filter.rx_filter_override ==
          S31_PARLIO_NATIVE_IQ_OVERRIDE ||
      s31_parlio_native_packed_iq_selected(config)) {
    uint32_t source_bit;
    if (s31_parlio_native_packed_iq_selected(config)) {
      /* Like the TCM dump word, the live diagnostic word carries signed Q in
       * modem_diag[9:0] and signed I in modem_diag[19:10]. Keep their top
       * eight bits and splice I across the two physical substitution groups.
       * The PARLIO pin map below exchanges the two complete physical byte
       * groups, so the captured word is native signed I followed by native
       * signed Q without CPU-side sample conversion:
       *
       *   output[7:0]  = modem_diag[19:12]
       *   output[15:8] = modem_diag[9:2]
       */
      if (bit < 8u) {
        source_bit = bit + 2u;
        return source_bit < 8u ? HP_PROBE_TOP_OUT0_IDX + source_bit
                               : HP_PROBE_TOP_OUT8_IDX + source_bit - 8u;
      }
      if (bit < 12u) {
        return HP_PROBE_TOP_OUT8_IDX + bit - 4u;
      }
      return LCD_DATA_OUT_PAD_OUT0_IDX + bit - 12u;
    }
    if ((config->rx_filter.rx_filter_mode & BIT(30)) != 0u) {
      /* Full-low mode carries two complete exchanged bytes.  Do not apply
       * either of the mixed-route bit-window shifts, because even a one-bit
       * shift would splice the two signed samples together. */
      return bit < 8u ? HP_PROBE_TOP_OUT0_IDX + bit
                      : HP_PROBE_TOP_OUT8_IDX + bit - 8u;
    }
    if (bit < 8u) {
      source_bit = (config->rx_filter.rx_filter_mode & 7u) + bit;
      return source_bit < 8u ? HP_PROBE_TOP_OUT0_IDX + source_bit
                             : HP_PROBE_TOP_OUT8_IDX + source_bit - 8u;
    }
    uint32_t high_shift =
        (config->rx_filter.rx_filter_mode & BIT(31)) != 0u
            ? (config->rx_filter.rx_filter_mode >> 27u) & 7u
            : (config->rx_filter.rx_filter_mode >> 4u) & 7u;
    source_bit = high_shift + bit - 8u;
    return LCD_DATA_OUT_PAD_OUT0_IDX + source_bit;
  }
  if (s31_parlio_host_real_selected(config)) {
    /* modem_diag[9:2] is the signed ADC word with its two least-significant
     * bits discarded. PARLIO packs these lanes directly. */
    return bit < 6u ? HP_PROBE_TOP_OUT0_IDX + bit + 2u
                    : HP_PROBE_TOP_OUT8_IDX + bit - 6u;
  }
  return bit < 8u ? HP_PROBE_TOP_OUT0_IDX + bit
                  : HP_PROBE_TOP_OUT8_IDX + bit - 8u;
}

/* IDF exposes the PARLIO sample clock only as an immutable creation
 * property. Rebuilding the unit for every ordinary sample-rate change is not
 * viable in the combined USB/Ethernet image: the driver gets its descriptors
 * and ISR objects from the small internal heap, which becomes fragmented once
 * both network stacks have run. The S31 divider is independent of the unit's
 * DMA geometry, so retime it while the receiver is disabled and retain every
 * allocation. */
static bool s31_gpio_diag_retime(uint32_t capture_rate_hz) {
  if (s_s31_parlio_unit == NULL || capture_rate_hz == 0u) {
    return false;
  }
  s31_gpio_diag_stop();

  uint32_t source_rate_hz = 0u;
  if (esp_clk_tree_src_get_freq_hz(PARLIO_CLK_SRC_DEFAULT,
                                   ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                   &source_rate_hz) != ESP_OK ||
      source_rate_hz == 0u) {
    return false;
  }
  hal_utils_clk_info_t clock_info = {
      .src_freq_hz = source_rate_hz,
      .exp_freq_hz = capture_rate_hz,
      .max_integ = PARLIO_LL_RX_MAX_CLK_INT_DIV,
      .min_integ = 1u,
      .max_fract = PARLIO_LL_RX_MAX_CLK_FRACT_DIV,
  };
  hal_utils_clk_div_t divider = {.integer = 1u};
  uint32_t actual_rate_hz =
      hal_utils_calc_clk_div_frac_accurate(&clock_info, &divider);
  if (actual_rate_hz == 0u) {
    return false;
  }
  parlio_ll_rx_set_clock_div(&PARL_IO, &divider);
  parlio_ll_rx_update_config(&PARL_IO);
  s_s31_parlio_rate_hz = capture_rate_hz;
  ESP_LOGI("iq_capture", "retimed persistent PARLIO RX clock to %" PRIu32
                         " Hz (actual %" PRIu32 " Hz)",
           capture_rate_hz, actual_rate_hz);
  return true;
}

static void s31_gpio_diag_prepare(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  if (expert != S31_GPIO_DIAG_OVERRIDE &&
      expert != S31_PARLIO_NATIVE_IQ_OVERRIDE &&
      expert != S31_PARLIO_HOST_IQ_OVERRIDE &&
      expert != S31_HP_TCM_PROBE_OVERRIDE &&
      !s31_continuous_real_if_selected(config) &&
      !s31_parlio_native_packed_iq_selected(config)) {
    return;
  }
  uint32_t data_width =
      (s31_parlio_host_real_selected(config) ||
       s31_parlio_usb_iq4_selected(config))
          ? 8u
          : 16u;
  uint32_t capture_rate_hz = s31_continuous_real_if_selected(config)
                                 ? S31_CONTINUOUS_INPUT_RATE_HZ
                                 : s31_parlio_capture_rate_hz(config);
  bool swapped_pins = s31_parlio_native_packed_iq_selected(config) &&
                      !s31_parlio_usb_iq4_selected(config);
  if (s_s31_parlio_unit != NULL &&
      s_s31_parlio_rate_hz != capture_rate_hz &&
      !s31_gpio_diag_retime(capture_rate_hz)) {
    s31_gpio_diag_release();
  }
  if (s_s31_parlio_unit != NULL) {
    s31_gpio_diag_stop();
    bool geometry_changed = s_s31_parlio_data_width != data_width ||
                            s_s31_parlio_swapped_pins != swapped_pins;
    if (geometry_changed) {
      /* rx_bus_wid_sel is shadowed into the active packing datapath. Merely
       * writing it while stopped works before the first transaction, but an
       * active 16-lane transaction followed by an 8-lane transaction retained
       * the old packing cadence. A core-clock reset only moved the error from
       * 4 to 8 MSa/s. Mirror the driver's fake-EOF recovery instead: reset the
       * complete peripheral register domain while RX/TX and GDMA are idle,
       * restore its retained configuration, then apply the new geometry. No
       * driver allocation or software object is disturbed.
       */
      parl_io_dev_t saved_regs = PARL_IO;
      parlio_ll_reset_register(0);
      memcpy((void *)&PARL_IO, &saved_regs, sizeof(saved_regs));
      parlio_ll_rx_update_config(&PARL_IO);
    }
    for (uint32_t bit = 0u; bit < 16u; ++bit) {
      if (s_s31_diag_gpio[bit] < 0) {
        continue;
      }
      uint32_t signal = s31_gpio_diag_output_signal(config, bit);
      esp_rom_gpio_connect_out_signal(s_s31_diag_gpio[bit], signal, false,
                                      false);
    }
    /* Data width is only immutable in the public driver API. The S31
     * hardware can update it while disabled, and the soft-delimiter driver
     * does not use cfg.data_width when receiving. Keep one persistent 16-lane
     * unit and reconnect its matrix inputs instead of reallocating GDMA and
     * interrupt objects from the fragmented post-network internal heap. */
    for (uint32_t bit = 0u; bit < 16u; ++bit) {
      uint32_t pin_index = swapped_pins ? bit ^ 8u : bit;
      if (s_s31_diag_gpio[pin_index] >= 0) {
        esp_rom_gpio_connect_in_signal(
            s_s31_diag_gpio[pin_index],
            PARLIO_RX_DATA0_PAD_IN_IDX + bit, false);
      }
    }
    parlio_ll_rx_set_bus_width(&PARL_IO, data_width);
    parlio_ll_rx_update_config(&PARL_IO);
    s_s31_parlio_delimiter = s31_parlio_usb_iq4_selected(config)
                                 ? s_s31_parlio_delimiter_half
                                 : s_s31_parlio_delimiter_full;
    s_s31_parlio_data_width = data_width;
    s_s31_parlio_swapped_pins = swapped_pins;
    return;
  }
  uint64_t pin_mask = 0u;
  for (uint32_t bit = 0u; bit < 16u; ++bit) {
    if (s_s31_diag_gpio[bit] >= 0) {
      pin_mask |= 1ull << s_s31_diag_gpio[bit];
    }
  }
  uint64_t reserved_mask = 0u;
  for (uint32_t pin = 0u; pin < SOC_GPIO_PIN_COUNT; ++pin) {
    if (esp_gpio_is_reserved(1ull << pin)) {
      reserved_mask |= 1ull << pin;
    }
  }
  if ((reserved_mask & pin_mask) != 0u) {
    return;
  }
  gpio_config_t gpio_cfg = {
      .pin_bit_mask = pin_mask,
      .mode = GPIO_MODE_INPUT_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
  for (uint32_t bit = 0u; bit < data_width; ++bit) {
    if (s_s31_diag_gpio[bit] < 0) {
      continue;
    }
    uint32_t signal = s31_gpio_diag_output_signal(config, bit);
    esp_rom_gpio_connect_out_signal(s_s31_diag_gpio[bit], signal, false,
                                    false);
  }

  if (s_s31_parlio_unit != NULL) {
    return;
  }
  /* Keep the high-rate circular acquisition ring off external PSRAM in the
   * Ethernet build. At 40 MB/s, simultaneous PARLIO writes and GMAC reads of
   * PSRAM can trip the S31 GDMA fabric; completed internal nodes are copied to
   * the much deeper PSRAM stream ring by the producer instead. */
#if CONFIG_ESP_SDR_TRANSPORT_USB && CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  /* Both high-speed transports leave too little contiguous internal RAM for
   * this ring. Keep one early, stable PSRAM allocation across RX/TX restarts:
   * moving the live GDMA target around the external-memory heap produced
   * boot- and post-TX-dependent loss at 16 MSa/s. */
  s_s31_parlio_dma_buffer = s_s31_parlio_dma_storage;
#elif CONFIG_ESP_SDR_TRANSPORT_USB && !CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  uint32_t parlio_caps =
      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT;
#else
  uint32_t parlio_caps = s31_tx_replay_selected(config)
                             ? MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA |
                                   MALLOC_CAP_8BIT
                             : MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA |
                                   MALLOC_CAP_8BIT;
#endif
#if !(CONFIG_ESP_SDR_TRANSPORT_USB && CONFIG_ESP_SDR_TRANSPORT_ETHERNET)
  s_s31_parlio_dma_buffer = heap_caps_aligned_calloc(
      64u, 1u, S31_PARLIO_DMA_BYTES, parlio_caps);
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  if (s_s31_parlio_dma_buffer == NULL) {
    ESP_LOGW("iq_capture",
             "internal PARLIO DMA ring unavailable; falling back to PSRAM");
    s_s31_parlio_dma_buffer = heap_caps_aligned_calloc(
        64u, 1u, S31_PARLIO_DMA_BYTES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  }
#endif
#endif
  ESP_ERROR_CHECK(s_s31_parlio_dma_buffer != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  parlio_rx_unit_config_t rx_config = {
      .trans_queue_depth = 1u,
      .max_recv_size = S31_PARLIO_DMA_BYTES,
      .dma_burst_size = 64u,
      /* Allocate and connect the maximum geometry once. Runtime selection
       * below narrows the disabled hardware to eight lanes for USB IQC4. */
      .data_width = 16u,
      .clk_src = PARLIO_CLK_SRC_DEFAULT,
      .ext_clk_freq_hz = 0u,
      .exp_clk_freq_hz = capture_rate_hz,
      .clk_in_gpio_num = GPIO_NUM_NC,
      .clk_out_gpio_num = GPIO_NUM_NC,
      .valid_gpio_num = GPIO_NUM_NC,
      .flags = {
          .clk_gate_en = false,
      },
  };
  for (uint32_t bit = 0u; bit < 16u; ++bit) {
    rx_config.data_gpio_nums[bit] = (gpio_num_t)s_s31_diag_gpio[bit];
  }
  /* Release the boot-time contiguous DMA reservation immediately before the
   * driver's descriptor allocation.  Unlike constructing the PARLIO unit at
   * boot, this preserves the first host-requested sample-rate geometry. */
  heap_caps_free(s_s31_parlio_internal_dma_reserve);
  s_s31_parlio_internal_dma_reserve = NULL;
  esp_err_t error = parlio_new_rx_unit(&rx_config, &s_s31_parlio_unit);
  if (error != ESP_OK) {
    ESP_LOGE("iq_capture", "PARLIO RX allocation failed: %s",
             esp_err_to_name(error));
    return;
  }
  s_s31_parlio_data_width = 16u;
  s_s31_parlio_rate_hz = capture_rate_hz;
  s_s31_parlio_swapped_pins = false;
  parlio_rx_soft_delimiter_config_t delimiter_config = {
      .sample_edge = PARLIO_SAMPLE_EDGE_POS,
      .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
      .eof_data_len = S31_PARLIO_BATCH_BYTES,
      .timeout_ticks = 0u,
  };
  error = parlio_new_rx_soft_delimiter(
      &delimiter_config, &s_s31_parlio_delimiter_full);
  if (error != ESP_OK) {
    ESP_LOGE("iq_capture", "PARLIO delimiter allocation failed: %s",
             esp_err_to_name(error));
    s31_gpio_diag_release();
    return;
  }
  delimiter_config.eof_data_len = S31_PARLIO_BATCH_BYTES / 2u;
  error = parlio_new_rx_soft_delimiter(
      &delimiter_config, &s_s31_parlio_delimiter_half);
  if (error != ESP_OK) {
    ESP_LOGE("iq_capture", "PARLIO half delimiter allocation failed: %s",
             esp_err_to_name(error));
    s31_gpio_diag_release();
    return;
  }
  s_s31_parlio_delimiter = s_s31_parlio_delimiter_full;
  parlio_rx_event_callbacks_t callbacks = {
      .on_partial_receive = s31_parlio_partial_receive,
  };
  error = parlio_rx_unit_register_event_callbacks(
      s_s31_parlio_unit, &callbacks, NULL);
  if (error != ESP_OK) {
    ESP_LOGE("iq_capture", "PARLIO callback setup failed: %s",
             esp_err_to_name(error));
    s31_gpio_diag_release();
    return;
  }
  /* Apply the requested lane order, width and delimiter to the retained
   * maximum-width unit before its first receive transaction. */
  s31_gpio_diag_prepare(config);
}

static void s31_gpio_diag_start(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  if ((expert != S31_GPIO_DIAG_OVERRIDE &&
       expert != S31_PARLIO_NATIVE_IQ_OVERRIDE &&
       expert != S31_PARLIO_HOST_IQ_OVERRIDE &&
       expert != S31_HP_TCM_PROBE_OVERRIDE &&
       !s31_continuous_real_if_selected(config) &&
       !s31_parlio_native_packed_iq_selected(config)) ||
      s_s31_parlio_unit == NULL || s_s31_parlio_running) {
    return;
  }
  s_s31_parlio_event_head = 0u;
  s_s31_parlio_event_tail = 0u;
  s_s31_parlio_callback_overruns = 0u;
  s_s31_parlio_discontinuities = 0u;
  s_s31_parlio_pending_event = (s31_parlio_event_t){0};
  s_s31_parlio_pending_offset = 0u;
  ESP_ERROR_CHECK(parlio_rx_unit_enable(s_s31_parlio_unit, true));
  /* A width update issued while the retained unit's peripheral clock is
   * disabled is not reliably latched on S31. Reassert it after enable but
   * before mounting the DMA transaction; no samples can be in flight yet. */
  parlio_ll_rx_set_bus_width(&PARL_IO, s_s31_parlio_data_width);
  parlio_ll_rx_update_config(&PARL_IO);
  ESP_ERROR_CHECK(parlio_rx_soft_delimiter_start_stop(
      s_s31_parlio_unit, s_s31_parlio_delimiter, true));
  parlio_receive_config_t receive_config = {
      .delimiter = s_s31_parlio_delimiter,
      .flags.partial_rx_en = true,
  };
  ESP_ERROR_CHECK(parlio_rx_unit_receive(
      s_s31_parlio_unit, s_s31_parlio_dma_buffer, S31_PARLIO_DMA_BYTES,
      &receive_config));
  s_s31_parlio_running = true;
}

static void s31_gpio_diag_stop(void) {
  if (!s_s31_parlio_running) {
    return;
  }
  (void)parlio_rx_soft_delimiter_start_stop(
      s_s31_parlio_unit, s_s31_parlio_delimiter, false);
  ESP_ERROR_CHECK(parlio_rx_unit_disable(s_s31_parlio_unit));
  s_s31_parlio_running = false;
}

/* Width and clock are immutable properties of a PARLIO unit. Full receiver
 * reconfiguration must destroy it, otherwise a later 32 MHz request silently
 * keeps the clock selected by the first stream after boot. */
static void s31_gpio_diag_release(void) {
  s31_gpio_diag_stop();
  s_s31_parlio_delimiter = NULL;
  if (s_s31_parlio_delimiter_half != NULL) {
    ESP_ERROR_CHECK(
        parlio_del_rx_delimiter(s_s31_parlio_delimiter_half));
    s_s31_parlio_delimiter_half = NULL;
  }
  if (s_s31_parlio_delimiter_full != NULL) {
    ESP_ERROR_CHECK(
        parlio_del_rx_delimiter(s_s31_parlio_delimiter_full));
    s_s31_parlio_delimiter_full = NULL;
  }
  if (s_s31_parlio_unit != NULL) {
    ESP_ERROR_CHECK(parlio_del_rx_unit(s_s31_parlio_unit));
    s_s31_parlio_unit = NULL;
  }
  s_s31_parlio_data_width = 0u;
  s_s31_parlio_rate_hz = 0u;
  s_s31_parlio_swapped_pins = false;
#if !(CONFIG_ESP_SDR_TRANSPORT_USB && CONFIG_ESP_SDR_TRANSPORT_ETHERNET)
  heap_caps_free(s_s31_parlio_dma_buffer);
#endif
  s_s31_parlio_dma_buffer = NULL;
  for (uint32_t bit = 0u; bit < 16u; ++bit) {
    if (s_s31_diag_gpio[bit] >= 0) {
      ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t)s_s31_diag_gpio[bit]));
    }
  }
}

static void engine_enable(void) {
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  if (s31_tx_replay_selected(&s_config)) {
    uint32_t upload_words;
    taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
    upload_words = s_s31_tx_replay_upload_words;
    taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
    /* Long host replay runs on the dedicated producer core. The control core
     * remains available to drain USB/Ethernet while Wi-Fi tasks and modem
     * interrupt routes are quiesced for the ownership interval. */
    const uint32_t tx_step = (uint32_t)s_config.tx.tx_tone0_step;
    const bool digital_txdc_probe =
        (tx_step & 15u) == 11u &&
        (tx_step & S31_TX_REPLAY_ADDRESS_PROBE_SELECTOR_M) != 0u;
    const uint32_t tx_rate_code = (tx_step >> 4u) & 0x0fu;
    const bool digital_txdc_upload =
        (tx_step & 15u) == 3u &&
        (tx_rate_code >= 9u && tx_rate_code <= 15u);
    const bool long_host_replay =
        ((((tx_step & 15u) == 3u) &&
          (upload_words > S31_TX_REPLAY_SIZE_M || digital_txdc_upload)) ||
         digital_txdc_probe) &&
        s_producer_task_handle != NULL;
    if (long_host_replay) {
      /* Core 0 owns USB, Ethernet, and Wi-Fi interrupt routing. Move the
       * cycle-exact updater to the already pinned high-priority producer on
       * core 1, leaving the transport core schedulable during RF replay. */
      (void)ulTaskNotifyTake(pdTRUE, 0u);
      s_s31_tx_worker_config = s_config;
      s_s31_tx_worker_requester = xTaskGetCurrentTaskHandle();
      s_s31_tx_worker_pending = true;
      const bool wifi_was_active = wifi_tx_rx_pause_for_replay();
      xTaskNotifyGive(s_producer_task_handle);
      (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      wifi_tx_rx_resume_after_replay(wifi_was_active);
      /* The worker stops RF while core 0's Wi-Fi/PHY environment is paused.
       * Recover only after resume; doing the sleep/wake on core 1 first lets
       * resume restore stale pre-TX state over the recovered diagnostic path.
       * This must happen before a finite Soapy burst returns because RX can
       * be activated next without any intervening configuration change. */
      modem_recover_rx_after_tx_replay();
    } else {
      s31_tx_replay_enable(&s_config);
    }
    return;
  }
#endif
  modem_enter_debug_mode();
#if CONFIG_IDF_TARGET_ESP32S31
  capture_config_t engine_config = s31_engine_config(&s_config);
  s31_gpio_diag_prepare(&s_config);
  s31_poison_dump_window();
  /* PHY/I2C configuration has already completed with the dump switch closed.
   * S31 only latches the register-only reset and dump setup while it is open. */
  esp_ipc_isr_stall_other_cpu();
  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                   S31_ADCTRIG_TCM_DUMP_CTRL);
  __asm__ __volatile__("fence" ::: "memory");
#endif
  adctrig_prepare(DUMP_BANK_WORDS, &engine_config); /* sample_count = one bank */
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
  reg32p_set_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                  MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
#if CONFIG_IDF_TARGET_ESP32S31
  uint32_t setup_dwell_sel =
      (s_config.iq_engine.adc_source_sel & S31_ADC_SOURCE_PULSE_DWELL_M) >>
      S31_ADC_SOURCE_PULSE_DWELL_S;
  uint32_t setup_dwell = 64u << setup_dwell_sel;
  for (volatile uint32_t i = 0u; i < setup_dwell; ++i) {
    __asm__ __volatile__("nop");
  }
  if ((s_config.iq_engine.adc_source_sel & S31_ADC_SOURCE_TCM_ALWAYS_ON) ==
      0u) {
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  }
  __asm__ __volatile__("fence" ::: "memory");
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  esp_ipc_isr_release_other_cpu();
#endif
}

static void engine_disable(void) {
#if CONFIG_IDF_TARGET_ESP32S31
  s_s31_tx_replay_burst_complete = false;
  /* Preserve PARLIO's internal stash/DMA allocations across ordinary RX/TX
   * transitions. prepare() releases them only if immutable geometry changes;
   * repeated delete/new cycles fragment the small internal heap. */
  s31_gpio_diag_stop();
#endif
  reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                    MODEM_WIFI_DUMP_CTRL_ENABLE_BIT |
                        MODEM_WIFI_DUMP_CTRL_CONTINUOUS_TRIGGER_GATE_BIT);
  reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                    MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
#endif
  __asm__ __volatile__("fence" ::: "memory");
}

/* ---- triggers ---- */

static inline bool trigger_interval_select(const trigger_plan_t *plan,
                                           uint32_t source_chunk_index) {
  if (plan->interval_chunks == 0u || plan->interval_duration_chunks == 0u) {
    return true;
  }
  if (source_chunk_index < plan->interval_offset_chunks) {
    return false;
  }
  uint32_t position = (source_chunk_index - plan->interval_offset_chunks) %
                      plan->interval_chunks;
  return position < plan->interval_duration_chunks;
}

static uint32_t IRAM_ATTR trigger_interval_next_selected(
    const trigger_plan_t *plan, uint32_t source_chunk_index) {
  if (plan->interval_chunks == 0u || plan->interval_duration_chunks == 0u) {
    return source_chunk_index;
  }
  if (source_chunk_index < plan->interval_offset_chunks) {
    return plan->interval_offset_chunks;
  }

  uint32_t since_offset = source_chunk_index - plan->interval_offset_chunks;
  uint32_t position = since_offset % plan->interval_chunks;
  if (position < plan->interval_duration_chunks) {
    return source_chunk_index;
  }
  return source_chunk_index + (plan->interval_chunks - position);
}

static uint32_t IRAM_ATTR trigger_interval_selected_count(
    const trigger_plan_t *plan, uint32_t first_chunk, uint32_t end_chunk) {
  if (end_chunk <= first_chunk) {
    return 0u;
  }
  if (plan->interval_chunks == 0u || plan->interval_duration_chunks == 0u) {
    return end_chunk - first_chunk;
  }
  uint32_t duration = plan->interval_duration_chunks;
  if (duration > plan->interval_chunks) {
    duration = plan->interval_chunks;
  }

  uint64_t before_first = 0u;
  uint64_t before_end = 0u;
  if (first_chunk > plan->interval_offset_chunks) {
    uint32_t rel = first_chunk - plan->interval_offset_chunks;
    before_first = (uint64_t)(rel / plan->interval_chunks) * duration;
    uint32_t rem = rel % plan->interval_chunks;
    before_first += rem < duration ? rem : duration;
  }
  if (end_chunk > plan->interval_offset_chunks) {
    uint32_t rel = end_chunk - plan->interval_offset_chunks;
    before_end = (uint64_t)(rel / plan->interval_chunks) * duration;
    uint32_t rem = rel % plan->interval_chunks;
    before_end += rem < duration ? rem : duration;
  }
  uint64_t count = before_end - before_first;
  return count > UINT32_MAX ? UINT32_MAX : (uint32_t)count;
}

static bool IRAM_ATTR trigger_interval_prev_selected(const trigger_plan_t *plan,
                                                     uint32_t end_chunk,
                                                     uint32_t *selected_chunk) {
  if (end_chunk == 0u) {
    return false;
  }
  if (plan->interval_chunks == 0u || plan->interval_duration_chunks == 0u) {
    *selected_chunk = end_chunk - 1u;
    return true;
  }
  uint32_t duration = plan->interval_duration_chunks;
  if (duration > plan->interval_chunks) {
    duration = plan->interval_chunks;
  }
  if (end_chunk <= plan->interval_offset_chunks) {
    return false;
  }
  uint32_t rel_end = end_chunk - plan->interval_offset_chunks;
  uint32_t period_index = (rel_end - 1u) / plan->interval_chunks;
  uint32_t period_start =
      plan->interval_offset_chunks + period_index * plan->interval_chunks;
  uint32_t selected_end = period_start + duration;
  if (selected_end > end_chunk) {
    selected_end = end_chunk;
  }
  if (selected_end > period_start) {
    *selected_chunk = selected_end - 1u;
    return true;
  }
  if (period_index == 0u) {
    return false;
  }
  period_start -= plan->interval_chunks;
  *selected_chunk = period_start + duration - 1u;
  return true;
}

static trigger_plan_t build_trigger_plan(const capture_config_t *config) {
  trigger_plan_t plan = {
      .mode = config->trigger.trigger_mode,
      .interval_chunks = config->trigger.trigger_config[0], /* now in CHUNKS */
      .interval_duration_chunks = config->trigger.trigger_config[2],
      .checks_per_chunk = 1u,
  };
  if (plan.interval_chunks == 0u) {
    plan.interval_chunks = 1u;
  }
  plan.interval_offset_chunks =
      config->trigger.trigger_config[1] % plan.interval_chunks;

  if (config->trigger.trigger_mode == IQ_TRIGGER_MODE_AGC) {
    plan.checks_per_chunk = config->trigger.trigger_config[0];
    plan.pre_chunks = config->trigger.trigger_config[1];
    plan.post_chunks = config->trigger.trigger_config[2];
    plan.max_gain = config->trigger.trigger_config[3];
    plan.state_mask =
        config->trigger.trigger_config[4] & IQ_AGC_TRIGGER_STATE_MASK_ALL;
    uint32_t match_mode = config->trigger.trigger_config[5];
    plan.use_fsm_match = match_mode != IQ_AGC_TRIGGER_MATCH_MAX_GAIN;
    plan.use_max_gain_match = match_mode != IQ_AGC_TRIGGER_MATCH_FSM;
  } else if (config->trigger.trigger_mode == IQ_TRIGGER_MODE_POWER) {
    plan.checks_per_chunk = config->trigger.trigger_config[0];
    plan.pre_chunks = config->trigger.trigger_config[1];
    plan.post_chunks = config->trigger.trigger_config[2];
    plan.threshold = config->trigger.trigger_config[3];
    plan.dc_shift = config->trigger.trigger_config[4];
  }
  if (plan.checks_per_chunk == 0u) {
    plan.checks_per_chunk = 1u;
  }
  if (plan.checks_per_chunk > IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK) {
    plan.checks_per_chunk = IQ_AGC_TRIGGER_MAX_CHECKS_PER_CHUNK;
  }
  uint32_t sample_stride = IQ_CHUNK_SAMPLE_WORDS / plan.checks_per_chunk;
  if (sample_stride == 0u) {
    sample_stride = 1u;
  }
  for (uint32_t check = 0u; check < plan.checks_per_chunk; ++check) {
    plan.sample_offsets[check] = (uint16_t)(check * sample_stride);
  }
  return plan;
}

/* Scan one chunk's samples; return true if it triggers (AGC / power). */
static bool IRAM_ATTR chunk_triggers(const trigger_plan_t *plan,
                                     const uint32_t *words, int32_t *sum_i,
                                     int32_t *sum_q, uint32_t *sum_n,
                                     int32_t dc_i, int32_t dc_q,
                                     bool dc_valid) {
  bool found = false;
  for (uint32_t check = 0u; check < plan->checks_per_chunk; ++check) {
    uint32_t raw = words[plan->sample_offsets[check]];
    if (plan->mode == IQ_TRIGGER_MODE_AGC) {
      uint32_t agc_state = raw >> 28;
      uint32_t rx_gain = (raw >> 20) & 0xffu;
      bool fsm = (plan->state_mask & (1u << agc_state)) != 0u;
      bool maxg = rx_gain <= plan->max_gain;
      if ((!plan->use_fsm_match || fsm) &&
          (!plan->use_max_gain_match || maxg)) {
        found = true;
      }
    } else { /* power */
      int32_t i = sign_extend_10_u32(raw);
      int32_t q = sign_extend_10_u32(raw >> 10);
      *sum_i += i;
      *sum_q += q;
      ++*sum_n;
      uint32_t mag = abs_i32_u32(i - dc_i) + abs_i32_u32(q - dc_q);
      if (dc_valid && mag >= plan->threshold) {
        found = true;
      }
    }
  }
  return found;
}

static void IRAM_ATTR expand_pre_post(uint8_t *mask, uint32_t n, uint32_t pre,
                                      uint32_t post) {
  uint32_t last_trig = UINT32_MAX;
  for (uint32_t i = 0u; i < n; ++i) {
    if (mask[i] == CHUNK_STREAM_TRIGGER) {
      last_trig = i;
      continue;
    }
    if (last_trig != UINT32_MAX && i - last_trig <= post &&
        mask[i] == CHUNK_STREAM_SKIP) {
      mask[i] = CHUNK_STREAM_POST;
    }
  }
  uint32_t next_trig = UINT32_MAX;
  for (uint32_t j = n; j > 0u; --j) {
    uint32_t i = j - 1u;
    if (mask[i] == CHUNK_STREAM_TRIGGER) {
      next_trig = i;
      continue;
    }
    if (next_trig != UINT32_MAX && next_trig - i <= pre &&
        mask[i] == CHUNK_STREAM_SKIP) {
      mask[i] = CHUNK_STREAM_PRE;
    }
  }
}

/* ---- producer: turn a frozen bank's [lo,hi) into aligned chunks ---- */

/* DCOC tap and feed-phase strides are odd and therefore coprime to the
 * power-of-two output chunk.  Rotating 32 well-spread taps gives the EMA an
 * unbiased traversal of periodic waveforms without adding a full-chunk scan
 * to the already-unavoidable S31 writer-off handoff. */
#define DCOC_TAP_STRIDE_WORDS 113u
#define DCOC_PHASE_STEP_WORDS 239u
static uint32_t s_dcoc_sample_phase;

static void IRAM_ATTR dcoc_feed_chunk(const volatile uint32_t *words) {
  sdr_agc_service_dcoc();
  if (!dcoc_feed_due()) {
    return;
  }
  int32_t sum_i = 0, sum_q = 0;
  _Static_assert((IQ_CHUNK_SAMPLE_WORDS & (IQ_CHUNK_SAMPLE_WORDS - 1u)) == 0u,
                 "DCOC phase wrap requires a power-of-two chunk");
  for (uint32_t j = 0u; j < DCOC_SAMPLES_PER_FEED; ++j) {
    uint32_t sample =
        (s_dcoc_sample_phase + j * DCOC_TAP_STRIDE_WORDS) &
        (IQ_CHUNK_SAMPLE_WORDS - 1u);
    uint32_t word = words[sample];
    sum_i += sign_extend_10_u32(word);
    sum_q += sign_extend_10_u32(word >> 10);
  }
  s_dcoc_sample_phase =
      (s_dcoc_sample_phase + DCOC_PHASE_STEP_WORDS) &
      (IQ_CHUNK_SAMPLE_WORDS - 1u);
  dcoc_feed(sum_i, sum_q);
}

static void IRAM_ATTR fill_and_push_chunk(const capture_config_t *config,
                                          const volatile uint32_t *words,
                                          uint32_t source_chunk,
                                          uint32_t sample_rate_hz,
                                          uint32_t rx_gain, uint32_t agc_state,
                                          stream_frame_t *slot) {
  uint32_t late_misses = 0u;
  uint32_t write_ptr = s_dbg_lo;
  uint32_t producer_write_ptr = adc_dump_store_addr();
#if CONFIG_IDF_TARGET_ESP32S31
  if (s31_pulse_tcm_before_copy(config)) {
    late_misses = s_s31_copy_ctrl_diag;
    write_ptr = s_s31_copy_mode_before_diag;
    producer_write_ptr =
        S31_PULSE_META_MARKER | ((s_s31_copy_physical_chunk & 0xffu) << 16) |
        (dump_ring_mod(
             adc_dump_store_addr_from_raw(s_s31_copy_mode_after_diag)) &
         0xffffu);
  }
#endif
  iq_chunk_report_meta_t meta = {
      .source_chunk_index = source_chunk,
      .adc_decimation = adc_decimation_value(config),
      .sample_rate_hz = sample_rate_hz,
      .center_freq_mhz = modem_rf_freq_hz() / HZ_PER_MHZ,
      .rx_gain = rx_gain,
      .agc_state = agc_state,
      .software_agc_active = sdr_agc_active(),
      .agc_robust_peak = sdr_agc_last_peak(),
      .agc_gain_changes = sdr_agc_gain_changes(),
      .dropped_chunks = s_stream_dropped_chunks,
      .bank_timer_late_misses = late_misses,
      .bank_timer_write_ptr = write_ptr,
      .producer_wake_write_ptr = producer_write_ptr,
  };
  if (stream_output_int8()) {
    if ((config->rx_filter.rx_filter_override >= 2u &&
         config->rx_filter.rx_filter_override < 8u) ||
        config->rx_filter.rx_filter_override == 28u ||
        config->rx_filter.rx_filter_override == 29u ||
        config->rx_filter.rx_filter_override == 30u ||
        config->rx_filter.rx_filter_override == 31u ||
        config->rx_filter.rx_filter_override == 32u ||
        config->rx_filter.rx_filter_override == 33u ||
        config->rx_filter.rx_filter_override == 34u ||
        config->rx_filter.rx_filter_override == 35u ||
        config->rx_filter.rx_filter_override == 36u ||
        config->rx_filter.rx_filter_override == 37u ||
        config->rx_filter.rx_filter_override == 14u ||
        config->rx_filter.rx_filter_override == 15u) {
      stream_ring_fill_bt_bytes_chunk_int8(slot, &meta, words);
    } else {
      stream_ring_fill_iq_chunk_int8(slot, &meta, words);
    }
  } else {
    stream_ring_fill_iq_chunk(slot, &meta, words);
  }
  dcoc_feed_chunk(words);
}

#if CONFIG_IDF_TARGET_ESP32S31
static void IRAM_ATTR fill_and_push_chunk_strided(
    const capture_config_t *config, uint32_t source_chunk,
    uint32_t sample_rate_hz, uint32_t rx_gain, uint32_t agc_state,
    stream_frame_t *slot, uint32_t start_word, uint32_t stride_words) {
  iq_chunk_report_meta_t meta = {
      .source_chunk_index = source_chunk,
      .adc_decimation = adc_decimation_value(config),
      .sample_rate_hz = sample_rate_hz,
      .center_freq_mhz = modem_rf_freq_hz() / HZ_PER_MHZ,
      .rx_gain = rx_gain,
      .agc_state = agc_state,
      .software_agc_active = sdr_agc_active(),
      .agc_robust_peak = sdr_agc_last_peak(),
      .agc_gain_changes = sdr_agc_gain_changes(),
      .dropped_chunks = s_stream_dropped_chunks,
      .bank_timer_late_misses = s_s31_copy_ctrl_diag,
      .bank_timer_write_ptr = s_s31_copy_mode_before_diag,
      .producer_wake_write_ptr =
          S31_PULSE_META_MARKER |
          (((start_word / IQ_CHUNK_SAMPLE_WORDS) & 0xffu) << 16) |
          (dump_ring_mod(adc_dump_store_addr_from_raw(
               s_s31_copy_mode_after_diag)) &
           0xffffu),
  };
  if (stream_output_int8()) {
    if ((config->rx_filter.rx_filter_override >= 2u &&
         config->rx_filter.rx_filter_override < 8u) ||
        config->rx_filter.rx_filter_override == 28u ||
        config->rx_filter.rx_filter_override == 29u ||
        config->rx_filter.rx_filter_override == 30u ||
        config->rx_filter.rx_filter_override == 31u ||
        config->rx_filter.rx_filter_override == 32u ||
        config->rx_filter.rx_filter_override == 33u ||
        config->rx_filter.rx_filter_override == 34u ||
        config->rx_filter.rx_filter_override == 35u ||
        config->rx_filter.rx_filter_override == 36u ||
        config->rx_filter.rx_filter_override == 37u ||
        config->rx_filter.rx_filter_override == 14u ||
        config->rx_filter.rx_filter_override == 15u) {
      stream_ring_fill_bt_bytes_chunk_strided_int8(
          slot, &meta, DUMP_BASE, start_word, stride_words, DUMP_BANK_WORDS);
    } else {
      stream_ring_fill_iq_chunk_strided_int8(
          slot, &meta, DUMP_BASE, start_word, stride_words, DUMP_BANK_WORDS);
    }
  } else {
    stream_ring_fill_iq_chunk_strided(slot, &meta, DUMP_BASE, start_word,
                                      stride_words, DUMP_BANK_WORDS);
  }
  /* DCOC servo feed: retrace the strided ring walk the fill above used
   * (slot->iq is packed, so the copied samples can't be pointed at). */
  sdr_agc_service_dcoc();
  if (dcoc_feed_due()) {
    uint32_t source = dump_ring_mod(
        start_word + s_dcoc_sample_phase * stride_words);
    uint32_t step = DCOC_TAP_STRIDE_WORDS * stride_words;
    int32_t sum_i = 0, sum_q = 0;
    for (uint32_t j = 0u; j < DCOC_SAMPLES_PER_FEED; ++j) {
      uint32_t word = DUMP_BASE[source];
      sum_i += sign_extend_10_u32(word);
      sum_q += sign_extend_10_u32(word >> 10);
      source += step;
      while (source >= DUMP_BANK_WORDS) {
        source -= DUMP_BANK_WORDS;
      }
    }
    s_dcoc_sample_phase =
        (s_dcoc_sample_phase + DCOC_PHASE_STEP_WORDS) &
        (IQ_CHUNK_SAMPLE_WORDS - 1u);
    dcoc_feed(sum_i, sum_q);
  }
}
#endif

static void stream_state_reset(void) {
  stream_ring_reset();
  s_stream_dropped_chunks = 0u;
  s_source_chunk_index = 0u;
  s_emit_chunk = 0u;
  s_s31_ring_started = false;
#if CONFIG_IDF_TARGET_ESP32S31
  s_s31_diag_abs_write = 0u;
  s_s31_diag_c_hi = 0u;
  s_s31_diag_emit_chunk = 0u;
  s_s31_diag_ring_started = 0u;
  s_s31_abs_write_accum = 0u;
  s_s31_abs_track_us = 0;
  s_s31_stage_valid = false;
  s_s31_stage_source_chunk = 0u;
  s_s31_stage_prev_end_raw = 0u;
  s_s31_stage_have_prev_end = false;
  s_s31_micro_gate_advances = 0u;
  s_s31_micro_gate_max_cycles = 0u;
  s_s31_micro_gate_batches = 0u;
  s_s31_cursor_min_backlog = UINT32_MAX;
  s_s31_cursor_max_backlog = 0u;
  s_s31_cursor_batches = 0u;
  s_s31_cursor_last_writer_cycle = 0u;
  s_s31_cursor_last_backlog = 0u;
  s_s31_cursor_have_writer_cycle = false;
  s_s31_cursor_overruns = 0u;
  s_s31_probe_source_chunk = 0u;
  s_s31_probe_end_cycle = 0u;
  s_s31_probe_have_end_cycle = false;
  s_s31_probe_logged_selector = UINT32_MAX;
#endif
  s_power_trigger_dc_valid = false;
  s_power_trigger_dc_i_q16 = 0;
  s_power_trigger_dc_q_q16 = 0;
  s_isr_prev_sa = adc_dump_store_addr();
  s_isr_wrap_count = 0u;
}

static void wait_stream_pipeline_idle(void) {
  for (uint32_t i = 0u;
       i < 100u && (s_stream_write_active || s_producer_active); ++i) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

#if CONFIG_IDF_TARGET_ESP32S31
#define S31_RING_GUARD_WORDS (12u * IQ_CHUNK_SAMPLE_WORDS)
#define S31_LIVE_COPY_GUARD_WORDS 256u
#define S31_RING_MAX_CHUNKS_PER_POLL 16u
#define S31_POLL_CLOSE_US 20u
#define S31_POLL_WAKE_MARGIN_US 30000u
#define S31_STAGE_WORDS (14u * IQ_CHUNK_SAMPLE_WORDS)
#define S31_STAGE_CHUNKS (S31_STAGE_WORDS / IQ_CHUNK_SAMPLE_WORDS)
#define S31_GPIO_DIAG_CHUNKS 2u
/* The live writer wraps at the programmed, chunk-aligned aperture. */
static volatile uint32_t *const S31_STAGE_BASE =
    (volatile uint32_t *)S31_MAC_DUMP_SRAM_GUARD_START;
static gdma_channel_handle_t s_s31_tcm_dma_tx[2];
static gdma_channel_handle_t s_s31_tcm_dma_rx[2];
static gdma_link_list_handle_t s_s31_tcm_dma_tx_links[2];
static gdma_link_list_handle_t s_s31_tcm_dma_rx_links[2];
static size_t s_s31_tcm_dma_int_align[2];
static size_t s_s31_tcm_dma_ext_align[2];
static int s_s31_tcm_dma_channel[2] = {-1, -1};
static gptimer_handle_t s_s31_tcm_gate_watchdog;
static RTC_NOINIT_ATTR volatile bool s_s31_tcm_gate_watchdog_fired;
/* The modem owns the complete TCM fabric while its dump gate is open; even
 * stores to the nominal guard can stall.  Live diagnostic-bus sampling must
 * therefore land in ordinary HP SRAM. */
static RTC_NOINIT_ATTR __attribute__((aligned(64))) uint32_t
    s_s31_gpio_diag_stage[S31_GPIO_DIAG_CHUNKS * IQ_CHUNK_SAMPLE_WORDS];
extern void s31_pie_memcpy_aligned(void *dst, const void *src,
                                   size_t byte_count);
extern uint32_t s31_pie_microgate_copy8(void *dst, const void *src,
                                       volatile uint32_t *gate_reg,
                                       uint32_t gate_mask);
extern uint32_t s31_pie_microgate_copy32(void *dst, const void *src,
                                        volatile uint32_t *gate_reg,
                                        uint32_t gate_mask);
_Static_assert(S31_STAGE_WORDS * sizeof(uint32_t) <=
                   MAC_DUMP_SRAM_BANK_BYTES,
               "pipelined staging area must fit the 64 KiB S31 guard");

static esp_err_t s31_tcm_dma_prepare(uint32_t backend, void *destination,
                                     size_t byte_count) {
  if (s_s31_tcm_dma_tx[backend] == NULL) {
    gdma_channel_alloc_config_t alloc_config = {0};
    esp_err_t err = backend == 0u
                        ? gdma_new_axi_channel(&alloc_config,
                                               &s_s31_tcm_dma_tx[backend],
                                               &s_s31_tcm_dma_rx[backend])
                        : gdma_new_ahb_channel(&alloc_config,
                                               &s_s31_tcm_dma_tx[backend],
                                               &s_s31_tcm_dma_rx[backend]);
    if (err != ESP_OK) {
      return err;
    }
    gdma_strategy_config_t strategy = {
        .owner_check = true,
        .auto_update_desc = true,
        .eof_till_data_popped = true,
    };
    ESP_RETURN_ON_ERROR(gdma_apply_strategy(s_s31_tcm_dma_tx[backend],
                                             &strategy),
                        "iq_capture", "TCM DMA TX strategy");
    ESP_RETURN_ON_ERROR(gdma_apply_strategy(s_s31_tcm_dma_rx[backend],
                                             &strategy),
                        "iq_capture", "TCM DMA RX strategy");
    gdma_transfer_config_t transfer = {
        .max_data_burst_size = 16u,
        .access_ext_mem = true,
    };
    ESP_RETURN_ON_ERROR(gdma_config_transfer(s_s31_tcm_dma_tx[backend],
                                              &transfer),
                        "iq_capture", "TCM DMA TX transfer");
    ESP_RETURN_ON_ERROR(gdma_config_transfer(s_s31_tcm_dma_rx[backend],
                                              &transfer),
                        "iq_capture", "TCM DMA RX transfer");
    gdma_trigger_t trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_M2M, 0);
    uint32_t free_mask = 0u;
    ESP_RETURN_ON_ERROR(gdma_get_free_m2m_trig_id_mask(
                            s_s31_tcm_dma_tx[backend], &free_mask),
                        "iq_capture", "TCM DMA trigger mask");
    if (free_mask == 0u) {
      return ESP_ERR_NOT_FOUND;
    }
    trigger.instance_id = __builtin_ctz(free_mask);
    ESP_RETURN_ON_ERROR(gdma_connect(s_s31_tcm_dma_tx[backend], trigger),
                        "iq_capture", "TCM DMA TX connect");
    ESP_RETURN_ON_ERROR(gdma_connect(s_s31_tcm_dma_rx[backend], trigger),
                        "iq_capture", "TCM DMA RX connect");
    gdma_channel_alignment_info_t alignment = {0};
    ESP_RETURN_ON_ERROR(gdma_get_channel_alignment_constraints(
                            s_s31_tcm_dma_tx[backend], &alignment),
                        "iq_capture", "TCM DMA alignment");
    s_s31_tcm_dma_int_align[backend] = alignment.int_mem_alignment;
    s_s31_tcm_dma_ext_align[backend] = alignment.ext_enc_mem_alignment;
    ESP_RETURN_ON_ERROR(gdma_get_channel_id(s_s31_tcm_dma_rx[backend],
                                             &s_s31_tcm_dma_channel[backend]),
                        "iq_capture", "TCM DMA channel ID");
    gdma_link_list_config_t link_config = {
        .num_items = 4u,
        .item_alignment = backend == 0u ? 8u : 4u,
        .flags = {
            .items_in_ext_mem = false,
            .check_owner = true,
        },
    };
    ESP_RETURN_ON_ERROR(gdma_new_link_list(
                            &link_config, &s_s31_tcm_dma_tx_links[backend]),
                        "iq_capture", "TCM DMA TX links");
    ESP_RETURN_ON_ERROR(gdma_new_link_list(
                            &link_config, &s_s31_tcm_dma_rx_links[backend]),
                        "iq_capture", "TCM DMA RX links");
  }

  ESP_RETURN_ON_ERROR(gdma_reset(s_s31_tcm_dma_tx[backend]), "iq_capture",
                      "TCM DMA TX reset");
  ESP_RETURN_ON_ERROR(gdma_reset(s_s31_tcm_dma_rx[backend]), "iq_capture",
                      "TCM DMA RX reset");
  gdma_buffer_mount_config_t tx_buffer = {
      .buffer = (void *)DUMP_BASE,
      .buffer_alignment = s_s31_tcm_dma_int_align[backend],
      .length = byte_count,
      .flags = {
          .mark_eof = true,
          .mark_final = GDMA_FINAL_LINK_TO_NULL,
          .bypass_buffer_addr_align_check = true,
      },
  };
  gdma_buffer_mount_config_t rx_buffer = {
      .buffer = destination,
      .buffer_alignment = s_s31_tcm_dma_int_align[backend],
      .length = byte_count,
      .flags = {
          .mark_final = GDMA_FINAL_LINK_TO_NULL,
          .bypass_buffer_addr_align_check = true,
      },
  };
  ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(
                          s_s31_tcm_dma_tx_links[backend], 0, &tx_buffer, 1,
                          NULL),
                      "iq_capture", "TCM DMA mount TX");
  ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(
                          s_s31_tcm_dma_rx_links[backend], 0, &rx_buffer, 1,
                          NULL),
                      "iq_capture", "TCM DMA mount RX");
  return ESP_OK;
}

static bool IRAM_ATTR s31_tcm_gate_watchdog_cb(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *event,
    void *user_data) {
  (void)timer;
  (void)event;
  (void)user_data;
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  __asm__ __volatile__("fence rw, rw" ::: "memory");
  s_s31_tcm_gate_watchdog_fired = true;
  return false;
}

/* Called by app_main on core 0 so the watchdog interrupt can recover core 1
 * even when a DMA start transaction stalls that core's bus interface. */
static void s31_tcm_gate_watchdog_init(void) {
  if (s_s31_tcm_gate_watchdog != NULL) {
    return;
  }
  gptimer_config_t config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000u,
      .intr_priority = 3,
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&config, &s_s31_tcm_gate_watchdog));
  gptimer_event_callbacks_t callbacks = {
      .on_alarm = s31_tcm_gate_watchdog_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(
      s_s31_tcm_gate_watchdog, &callbacks, NULL));
  ESP_ERROR_CHECK(gptimer_enable(s_s31_tcm_gate_watchdog));
}

typedef struct {
  uint32_t source_chunk;
  uint32_t start_raw;
  uint32_t end_raw;
} s31_stage_batch_diag_t;

static void s31_prepare_stage_frames(const capture_config_t *config,
                                     stream_frame_t **slots, uint32_t n,
                                     const s31_stage_batch_diag_t *diag) {
  uint32_t sample_cycles = adc_dump_sample_cycles(config);
  uint32_t sample_rate_hz =
      sample_cycles != 0u ? ADC_DUMP_CLOCK_HZ / sample_cycles : 0u;
  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
  uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                       MODEM_WIFI_AGC_AGCRD3_STATE_S;
  uint32_t ctrl = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
  for (uint32_t frame = 0u; frame < n; ++frame) {
    uint32_t batch = frame / S31_STAGE_CHUNKS;
    uint32_t source_chunk =
        diag[batch].source_chunk + (frame % S31_STAGE_CHUNKS);
    iq_chunk_report_meta_t meta = {
        .source_chunk_index = source_chunk,
        .adc_decimation = adc_decimation_value(config),
        .sample_rate_hz = sample_rate_hz,
        .center_freq_mhz = modem_rf_freq_hz() / HZ_PER_MHZ,
        .rx_gain = rx_gain,
        .agc_state = agc_state,
        .software_agc_active = sdr_agc_active(),
        .agc_robust_peak = sdr_agc_last_peak(),
        .agc_gain_changes = sdr_agc_gain_changes(),
        .dropped_chunks = s_stream_dropped_chunks,
        .bank_timer_late_misses = ctrl,
        .bank_timer_write_ptr = diag[batch].start_raw,
        .producer_wake_write_ptr =
            S31_PULSE_META_MARKER |
            dump_ring_mod(diag[batch].end_raw),
    };
    stream_ring_prepare_iq_chunk(slots[frame], &meta);
  }
}

#if CONFIG_ESP_SDR_TRANSPORT_USB
/* All static IQ fields and IQU transport tickets are prepared before the TCM
 * switch opens. Core 1 has write-through PSRAM attributes from startup, so
 * these measured boundary fields and CRCs become DMA-visible without a cache
 * operation. This routine deliberately contains no RTOS, cache, or USB-stack
 * calls and is safe while core 0 is parked. */
static void __attribute__((noinline)) s31_finalize_usb_stage_batch(
    stream_frame_t **slots, uint32_t n,
    const s31_stage_batch_diag_t *diag) {
  for (uint32_t frame = 0u; frame < n; ++frame) {
    uint32_t batch = frame / S31_STAGE_CHUNKS;
    uint32_t source_chunk =
        diag[batch].source_chunk + (frame % S31_STAGE_CHUNKS);
    iq_chunk_t *iq = &slots[frame]->iq;
    iq->source_chunk_index = source_chunk;
    iq->chunk_counter = source_chunk;
    iq->dropped_chunks = s_stream_dropped_chunks;
    iq->bank_timer_write_ptr = diag[batch].start_raw;
    iq->producer_wake_write_ptr =
        S31_PULSE_META_MARKER |
        dump_ring_mod(diag[batch].end_raw);

    uint8_t *packet = (uint8_t *)slots[frame] - sizeof(iq_udp_header_t);
    iq_udp_header_t *header = (iq_udp_header_t *)packet;
    header->frame_sequence = iq->sequence;
    header->source_chunk_index = source_chunk;
    header->frame_crc32 = iq->crc32;
    header->firmware_dropped_chunks = s_stream_dropped_chunks;
    memcpy(header->frame_magic, iq->magic, sizeof(header->frame_magic));
    header->header_crc32 = esp_rom_crc32_le(
        0u, packet, offsetof(iq_udp_header_t, header_crc32));
  }
  __asm__ __volatile__("fence rw, rw" ::: "memory");
}

static bool s31_modem_diag_probe_enabled(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  return (expert >= S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST &&
          expert <= S31_MODEM_DIAG_PROBE_OVERRIDE_LAST) ||
         (expert >= S31_DATADUMP_MMIO_PROBE_OVERRIDE_FIRST &&
          expert <= S31_DATADUMP_MMIO_PROBE_OVERRIDE_LAST) ||
         expert == S31_LP_CORE_PROBE_OVERRIDE ||
         expert == S31_HP_TCM_PROBE_OVERRIDE ||
         expert == S31_SEAMLESS_MICRO_GATE_OVERRIDE ||
         expert == S31_GPIO_DIAG_OVERRIDE ||
         expert == S31_PARLIO_NATIVE_IQ_OVERRIDE ||
         expert == S31_PARLIO_HOST_IQ_OVERRIDE;
}

/* Diagnostic proof-of-path for the live modem bus.  Sampling into the lower
 * reserved TCM guard avoids PSRAM latency and, unlike the ADC dump aperture,
 * requires no TCM ownership switch.  The later TCM-to-PSRAM copy deliberately
 * leaves an honestly reported inter-burst gap; once a valid IQ selector is
 * found, the next step is to attach this bus to a continuous DMA sink. */
static void s31_process_modem_diag_probe(const capture_config_t *config) {
  uint32_t expert = config->rx_filter.rx_filter_override;
  const bool lp_core_probe = expert == S31_LP_CORE_PROBE_OVERRIDE;
  const bool hp_tcm_probe = expert == S31_HP_TCM_PROBE_OVERRIDE;
  const bool seamless_micro_gate =
      expert == S31_SEAMLESS_MICRO_GATE_OVERRIDE;
  const bool gpio_diag = expert == S31_GPIO_DIAG_OVERRIDE;
  const uint32_t capture_chunks =
      (gpio_diag || seamless_micro_gate) ? S31_GPIO_DIAG_CHUNKS
                                         : S31_STAGE_CHUNKS;
  const uint32_t capture_words = capture_chunks * IQ_CHUNK_SAMPLE_WORDS;
  stream_frame_t *slots[S31_STAGE_CHUNKS];
  if (!iq_usb_direct_reserve(capture_chunks, slots)) {
    return;
  }

  uint32_t sample_cycles = adc_dump_sample_cycles(config);
  uint32_t sample_rate_hz = sample_cycles != 0u
                                ? ADC_DUMP_CLOCK_HZ / sample_cycles
                                : 0u;
  if (sample_rate_hz == 0u) {
    iq_usb_direct_abort();
    return;
  }
  uint32_t cycles_per_sample =
      (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u) / sample_rate_hz;
  if (cycles_per_sample == 0u) {
    cycles_per_sample = 1u;
  }

  uint32_t start_cycle = esp_cpu_get_cycle_count();
  if (s_s31_probe_have_end_cycle) {
    uint32_t idle_cycles = start_cycle - s_s31_probe_end_cycle;
    uint32_t missed_samples = idle_cycles > cycles_per_sample
                                  ? idle_cycles / cycles_per_sample - 1u
                                  : 0u;
    uint32_t missed_chunks =
        (missed_samples + IQ_CHUNK_SAMPLE_WORDS - 1u) /
        IQ_CHUNK_SAMPLE_WORDS;
    s_stream_dropped_chunks += missed_chunks;
    s_s31_probe_source_chunk += missed_chunks;
  }

  uint32_t selector = config->rx_filter.rx_filter_mode & 31u;
  s31_stage_batch_diag_t diag = {
      .source_chunk = s_s31_probe_source_chunk,
      .start_raw = selector,
      .end_raw = 0u,
  };
  s31_prepare_stage_frames(config, slots, capture_chunks, &diag);

  const bool datadump_mmio =
      expert >= S31_DATADUMP_MMIO_PROBE_OVERRIDE_FIRST &&
      expert <= S31_DATADUMP_MMIO_PROBE_OVERRIDE_LAST;
  const bool swap = !datadump_mmio && !lp_core_probe && !hp_tcm_probe &&
                    !gpio_diag &&
                    !seamless_micro_gate &&
                    ((expert - S31_MODEM_DIAG_PROBE_OVERRIDE_FIRST) & 1u) !=
                        0u;
  uint32_t probe_offset = datadump_mmio
                              ? 0x20u +
                                    (expert -
                                     S31_DATADUMP_MMIO_PROBE_OVERRIDE_FIRST) *
                                        0x80u +
                                    selector * sizeof(uint32_t)
                              : 0u;
  uintptr_t probe_addr = DR_REG_MODEM_DATADUMP_BASE + probe_offset;
  uint32_t deadline = esp_cpu_get_cycle_count();
  uint32_t first_sample_cycle = 0u;
  uint32_t last_sample_cycle = 0u;
  uint32_t previous = UINT32_MAX;
  uint32_t changes = 0u;
  uint32_t raw_min = UINT32_MAX;
  uint32_t raw_max = 0u;
  uint32_t late_samples = 0u;
  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t status = ((agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M) << 20u) |
                    (((agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                      MODEM_WIFI_AGC_AGCRD3_STATE_S)
                     << 28u);
  uint32_t diag_fix_read =
      reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG);
  uint32_t diag_enable_read = reg32_read_addr(HP_SYSTEM_MODEM_DIAG_EN_REG);

  if (seamless_micro_gate) {
    /* Decisive bus-master experiment. Keep the destination in RTC SRAM and
     * use no DMA interrupt, so a blocked TCM read cannot deadlock PSRAM or an
     * ISR. Wait until this source range is known to have been written, start
     * the preconfigured DMA while modem ownership remains uninterrupted, and
     * sample the raw completion bit before returning ownership to the CPUs. */
    const uint32_t backend = 1u; /* AHB DMA keeps the PSRAM fabric independent. */
    memset((void *)DUMP_BASE, 0xa5, DUMP_BANK_WORDS * sizeof(uint32_t));
    memset(s_s31_gpio_diag_stage, 0x5a,
           capture_words * sizeof(uint32_t));
    esp_err_t prepare_err = s31_tcm_dma_prepare(
        backend, s_s31_gpio_diag_stage,
        capture_words * sizeof(uint32_t));
    if (prepare_err != ESP_OK) {
      ESP_LOGE("iq_capture", "TCM DMA backend %" PRIu32
                             " prepare failed: %s",
               backend, esp_err_to_name(prepare_err));
      iq_usb_direct_abort();
      return;
    }
    const int dma_channel = s_s31_tcm_dma_channel[backend];
    AHB_DMA.in_intr[dma_channel].clr.val = UINT32_MAX;
    AHB_DMA.out_intr[dma_channel].clr.val = UINT32_MAX;
    s_s31_tcm_gate_watchdog_fired = false;
    gptimer_alarm_config_t watchdog_alarm = {
        .alarm_count = 10000u,
    };
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_s31_tcm_gate_watchdog, 0u));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_s31_tcm_gate_watchdog,
                                              &watchdog_alarm));
    ESP_ERROR_CHECK(gptimer_start(s_s31_tcm_gate_watchdog));
    uint32_t gate_mask = S31_ADCTRIG_TCM_DUMP_CTRL;
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, gate_mask);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    uint32_t last_ptr = dump_ring_mod(adc_dump_store_addr_raw());
    uint32_t writer_abs = 0u;
    first_sample_cycle = esp_cpu_get_cycle_count();
    while (writer_abs < capture_words + 512u &&
           !s_s31_tcm_gate_watchdog_fired) {
      uint32_t ptr = dump_ring_mod(adc_dump_store_addr_raw());
      uint32_t advance = dump_ring_delta(ptr, last_ptr);
      if (advance != 0u) {
        writer_abs += advance;
        last_ptr = ptr;
      }
    }
    esp_err_t rx_start = gdma_start(
        s_s31_tcm_dma_rx[backend],
        gdma_link_get_head_addr(s_s31_tcm_dma_rx_links[backend]));
    esp_err_t tx_start = gdma_start(
        s_s31_tcm_dma_tx[backend],
        gdma_link_get_head_addr(s_s31_tcm_dma_tx_links[backend]));
    bool dma_done_open = false;
    uint32_t dma_done_writer_abs = 0u;
    while (writer_abs < DUMP_BANK_WORDS + 1024u &&
           !s_s31_tcm_gate_watchdog_fired) {
      uint32_t ptr = dump_ring_mod(adc_dump_store_addr_raw());
      uint32_t advance = dump_ring_delta(ptr, last_ptr);
      if (advance != 0u) {
        writer_abs += advance;
        last_ptr = ptr;
      }
      uint32_t raw = AHB_DMA.in_intr[dma_channel].raw.val;
      if (!s_s31_tcm_gate_watchdog_fired && !dma_done_open &&
          (raw & BIT(1)) != 0u) {
        dma_done_open = true;
        dma_done_writer_abs = writer_abs;
      }
    }
    last_sample_cycle = esp_cpu_get_cycle_count();
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
    (void)gptimer_stop(s_s31_tcm_gate_watchdog);
    uint32_t post_raw = AHB_DMA.in_intr[dma_channel].raw.val;
    (void)gdma_stop(s_s31_tcm_dma_tx[backend]);
    (void)gdma_stop(s_s31_tcm_dma_rx[backend]);
    for (uint32_t word = 0u; word < capture_words; ++word) {
      uint32_t raw = s_s31_gpio_diag_stage[word];
      if (raw != 0xa5a5a5a5u && raw != 0x5a5a5a5au) {
        ++changes;
      }
      if (raw < raw_min) {
        raw_min = raw;
      }
      if (raw > raw_max) {
        raw_max = raw;
      }
    }
    late_samples = dma_done_open ? 0u : 1u;
    diag.start_raw = (backend << 31u) |
                     ((uint32_t)(rx_start == ESP_OK) << 30u) |
                     ((uint32_t)(tx_start == ESP_OK) << 29u) |
                     ((uint32_t)s_s31_tcm_gate_watchdog_fired << 28u) |
                     (dma_done_writer_abs & 0x0fffffffu);
    diag.end_raw = ((post_raw & 0xffu) << 24u) |
                   (changes & 0x00ffffffu);
  } else if (lp_core_probe) {
    /* Keep the modem writer enabled for the complete burst. Census the raw LP
     * probe at the target sample cadence; do not assume the load-result group
     * carries a software-generated valid bit until the hardware sweep proves
     * it. */
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                     S31_ADCTRIG_TCM_DUMP_CTRL);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    deadline = esp_cpu_get_cycle_count();
    uint32_t sample = 0u;
    for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
      volatile uint32_t *dst = (volatile uint32_t *)(
          (uintptr_t)slots[frame] + offsetof(iq_chunk_t, samples));
      for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS;
           ++word, ++sample) {
        uint32_t now;
        do {
          now = esp_cpu_get_cycle_count();
        } while ((int32_t)(now - deadline) < 0);
        uint32_t raw = reg32_read_addr(LP_SYSTEM_REG_LP_PROBE_OUT_REG);
        if (sample == 0u) {
          first_sample_cycle = now;
        } else if (now - deadline >= cycles_per_sample * 2u) {
          ++late_samples;
        }
        dst[word] = status | (raw & 0x000fffffu);
        if (raw != previous) {
          ++changes;
        }
        previous = raw;
        if (raw < raw_min) {
          raw_min = raw;
        }
        if (raw > raw_max) {
          raw_max = raw;
        }
        last_sample_cycle = now;
        deadline += cycles_per_sample;
      }
    }
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
    esp_ipc_isr_release_other_cpu();
  } else if (hp_tcm_probe) {
    esp_ipc_isr_stall_other_cpu();
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                     S31_ADCTRIG_TCM_DUMP_CTRL);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    deadline = esp_cpu_get_cycle_count();
    uint32_t sample = 0u;
    for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
      volatile uint32_t *dst = (volatile uint32_t *)(
          (uintptr_t)slots[frame] + offsetof(iq_chunk_t, samples));
      for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS;
           ++word, ++sample) {
        uint32_t now;
        do {
          now = esp_cpu_get_cycle_count();
        } while ((int32_t)(now - deadline) < 0);
        if (sample == 0u) {
          first_sample_cycle = now;
        } else if (now - deadline >= cycles_per_sample) {
          ++late_samples;
        }
        uint32_t raw = reg32_read_addr(HP_SYSTEM_PROBE_OUT_REG);
        dst[word] = raw;
        if (raw != previous) {
          ++changes;
        }
        previous = raw;
        if (raw < raw_min) {
          raw_min = raw;
        }
        if (raw > raw_max) {
          raw_max = raw;
        }
        last_sample_cycle = now;
        deadline += cycles_per_sample;
      }
    }
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
    esp_ipc_isr_release_other_cpu();
  } else if (gpio_diag) {
    /* MODEM_DIAG_EN substitutes the modem's selected low 16-bit slice onto
     * HP_PROBE_TOP_OUT[15:0]. Loop those signals through GPIO pads which do
     * not overlap the module memory, console, or RGMII groups. */
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    deadline = esp_cpu_get_cycle_count();
    for (uint32_t sample = 0u; sample < capture_words; ++sample) {
      uint32_t now;
      do {
        now = esp_cpu_get_cycle_count();
      } while ((int32_t)(now - deadline) < 0);
      if (sample == 0u) {
        first_sample_cycle = now;
      } else if (now - deadline >= cycles_per_sample) {
        ++late_samples;
      }
      uint32_t raw = (GPIO.in.val >> 8u) & 0xffffu;
      int32_t i = (int8_t)(raw >> 8u);
      int32_t q = (int8_t)raw;
      s_s31_gpio_diag_stage[sample] =
          status | ((((uint32_t)(i << 2)) & 0x3ffu) << 10u) |
          (((uint32_t)(q << 2)) & 0x3ffu);
      if (raw != previous) {
        ++changes;
      }
      previous = raw;
      if (raw < raw_min) {
        raw_min = raw;
      }
      if (raw > raw_max) {
        raw_max = raw;
      }
      last_sample_cycle = now;
      deadline += cycles_per_sample;
    }
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  } else {
    taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
    for (uint32_t sample = 0u; sample < capture_words; ++sample) {
    uint32_t now;
    do {
      now = esp_cpu_get_cycle_count();
    } while ((int32_t)(now - deadline) < 0);
    if (sample == 0u) {
      first_sample_cycle = now;
    } else if (now - deadline >= cycles_per_sample) {
      ++late_samples;
    }
    uint32_t raw = datadump_mmio
                       ? reg32_read_addr(probe_addr)
                       : reg32_read_addr(HP_SYSTEM_PROBE_OUT_REG) & 0xffffu;
    if (datadump_mmio) {
      /* Preserve the peripheral word verbatim.  A real sample port should
       * already use the dump engine's native Q[9:0]/I[19:10] packing. */
      S31_STAGE_BASE[sample] = raw;
    } else {
      uint32_t low = raw & 0xffu;
      uint32_t high = raw >> 8u;
      int32_t i = (int8_t)(swap ? low : high);
      int32_t q = (int8_t)(swap ? high : low);
      S31_STAGE_BASE[sample] = status |
                               (((uint32_t)i & 0x3ffu) << 10u) |
                               ((uint32_t)q & 0x3ffu);
    }
    if (raw != previous) {
      ++changes;
    }
    previous = raw;
    if (raw < raw_min) {
      raw_min = raw;
    }
    if (raw > raw_max) {
      raw_max = raw;
    }
    last_sample_cycle = now;
    deadline += cycles_per_sample;
    }
    taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  }
  s_s31_probe_end_cycle = last_sample_cycle;
  s_s31_probe_have_end_cycle = true;

  if (seamless_micro_gate) {
    for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
      volatile uint32_t *dst = (volatile uint32_t *)(
          (uintptr_t)slots[frame] + offsetof(iq_chunk_t, samples));
      uint32_t stage_off = frame * IQ_CHUNK_SAMPLE_WORDS;
      for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS; ++word) {
        dst[word] = s_s31_gpio_diag_stage[stage_off + word];
      }
    }
  } else if (!lp_core_probe && !hp_tcm_probe) {
    for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
      volatile uint32_t *dst = (volatile uint32_t *)(
          (uintptr_t)slots[frame] + offsetof(iq_chunk_t, samples));
      uint32_t stage_off = frame * IQ_CHUNK_SAMPLE_WORDS;
      for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS; ++word) {
        dst[word] = gpio_diag ? s_s31_gpio_diag_stage[stage_off + word]
                              : S31_STAGE_BASE[stage_off + word];
      }
    }
  }
  s31_finalize_usb_stage_batch(slots, capture_chunks, &diag);
  uint32_t actual_cycles =
      last_sample_cycle > first_sample_cycle
          ? (last_sample_cycle - first_sample_cycle) /
                (capture_words - 1u)
          : 0u;
  for (uint32_t frame = 0u; frame < capture_chunks; ++frame) {
    slots[frame]->iq.bank_timer_late_misses =
        gpio_diag ? diag_fix_read
                  : (seamless_micro_gate
                         ? diag.end_raw
                         : slots[frame]->iq.bank_timer_late_misses);
    slots[frame]->iq.bank_timer_write_ptr =
        gpio_diag ? diag_enable_read
                  : (seamless_micro_gate ? diag.start_raw : selector);
    slots[frame]->iq.producer_wake_write_ptr =
        0xd1000000u | (actual_cycles & 0xffffu);
  }
  if (!iq_usb_direct_commit(slots, capture_chunks)) {
    s_stream_dropped_chunks += capture_chunks;
    return;
  }

  uint32_t log_key = selector | (expert << 8u);
  /* Formatted logging can lazily allocate a libc mutex.  The USB direct slab
   * intentionally leaves very little internal heap, so never invoke it from
   * the GPIO diagnostic path after the stream buffers have been reserved. */
  if (!gpio_diag && s_s31_probe_logged_selector != log_key) {
    ESP_LOGI("iq_capture",
             "%s selector=%" PRIu32 " addr=%08" PRIxPTR
             " swap=%u raw=%08" PRIx32 "..%08" PRIx32
             " changes=%" PRIu32 "/%u cycles=%" PRIu32
             " late=%" PRIu32 " regs=%08" PRIx32 "/%08" PRIx32
             "/%08" PRIx32 "/%08" PRIx32 "/%08" PRIx32,
             seamless_micro_gate
                 ? "seamless_micro_gate"
                 : (lp_core_probe
                 ? "lp_core_probe"
                 : (hp_tcm_probe
                        ? "hp_tcm_probe"
                        : (gpio_diag
                               ? "gpio_diag"
                               : (datadump_mmio ? "datadump_mmio"
                                                : "modem_diag")))),
             selector,
             lp_core_probe ? (uintptr_t)LP_SYSTEM_REG_LP_PROBE_OUT_REG
                           : (gpio_diag ? (uintptr_t)&GPIO.in.val
                           : (datadump_mmio
                                  ? probe_addr
                                  : (uintptr_t)HP_SYSTEM_PROBE_OUT_REG)),
             swap ? 1u : 0u, raw_min, raw_max, changes,
             capture_words, actual_cycles, late_samples,
             reg32_read_addr(MODEM_WIDGETS_CLK_CONF_REG),
             reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG),
             reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG),
             reg32_read_addr(HP_SYSTEM_MODEM_DIAG_EN_REG),
             reg32p_read(&MODEM_WIFI_BB.BB_DIAG0));
    s_s31_probe_logged_selector = log_key;
  }
  s_s31_probe_source_chunk += capture_chunks;
  s_emit_chunk = s_s31_probe_source_chunk;
  s_source_chunk_index = s_emit_chunk;
}

#endif

/* PARLIO samples the live modem diagnostic bus into one circular GDMA
 * transaction: 16-bit native I/Q at 16 MHz on Ethernet, or the 8-bit real
 * lane at 4 MHz on USB.  There is deliberately no transaction boundary in
 * this path: descriptor callbacks only publish completed pieces of the same
 * permanently mounted ring. */
static bool s31_parlio_next_event(s31_parlio_event_t *event) {
  while (s_s31_parlio_running && iq_network_stream_armed()) {
    uint32_t head = __atomic_load_n(&s_s31_parlio_event_head,
                                    __ATOMIC_ACQUIRE);
    uint32_t backlog = head - s_s31_parlio_event_tail;
    if (backlog >= S31_PARLIO_SAFE_NODE_BACKLOG) {
      /* DMA is one descriptor away from reusing the oldest completed node.
       * Discard the complete backlog rather than race an overwrite. */
      s_s31_parlio_event_tail = head;
      ++s_s31_parlio_callback_overruns;
      ++s_s31_parlio_discontinuities;
      return false;
    }
    if (backlog != 0u) {
      uint32_t sequence = s_s31_parlio_event_tail;
      s31_parlio_event_t value =
          s_s31_parlio_events[sequence % S31_PARLIO_EVENT_RING_SIZE];
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      if (value.sequence != sequence || value.data == NULL ||
          value.bytes == 0u) {
        s_s31_parlio_event_tail = head;
        ++s_s31_parlio_discontinuities;
        return false;
      }
      /* The PARLIO ISR invalidates this node on the CPU that services GDMA,
       * but the producer task can run on the other S31 core.  Invalidate the
       * completed node again in the consumer context before memcpy.  Without
       * this, that core can retain individual 64-byte lines from the previous
       * lap of the circular buffer: coherent-tone tests then show phase jumps
       * every 32 complex samples and byte-identical frames exactly one
       * 14-frame DMA-ring lap apart.  PARLIO's 4092-byte nodes are not cache
       * aligned, while M2C msync deliberately rejects its UNALIGNED flag, so
       * cover the node with complete cache lines.  The receive buffer itself
       * is 64-byte aligned and padded to a complete line. */
      size_t cache_line = esp_cache_get_line_size_by_addr(value.data);
      if (cache_line != 0u) {
        uintptr_t begin = (uintptr_t)value.data & ~(cache_line - 1u);
        uintptr_t end = ((uintptr_t)value.data + value.bytes + cache_line - 1u) &
                        ~(cache_line - 1u);
        (void)esp_cache_msync((void *)begin, end - begin,
                              ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      }
      /* Keep tail on this node until the producer has copied every byte. DMA
       * ownership must not be released merely because a pointer was handed
       * out: a preemption or a batch boundary could otherwise let GDMA wrap
       * and overwrite the node while it is still being read. */
      *event = value;
      return true;
    }
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
  }
  return false;
}

static void s31_parlio_release_pending_event(void) {
  if (s_s31_parlio_pending_event.data == NULL ||
      s_s31_parlio_pending_offset < s_s31_parlio_pending_event.bytes) {
    return;
  }
  s_s31_parlio_event_tail = s_s31_parlio_pending_event.sequence + 1u;
  s_s31_parlio_pending_event = (s31_parlio_event_t){0};
  s_s31_parlio_pending_offset = 0u;
}

static void s31_process_parlio_diag(const capture_config_t *config) {
  /* Expert sweeps can request a finite number of raw chunks through the
   * ordinary interval-duration field.  Stop producing at that exact batch
   * boundary so the Ethernet queue drains and control remains recoverable
   * even when an experimental rate exceeds the current transport ceiling. */
  uint32_t chunk_limit = config->trigger.trigger_config[1];
  if (chunk_limit != 0u && s_s31_probe_source_chunk >= chunk_limit) {
    s31_gpio_diag_stop();
    return;
  }
  s31_parlio_release_pending_event();
  if (s_s31_parlio_pending_offset < s_s31_parlio_pending_event.bytes) {
    uint32_t head = __atomic_load_n(&s_s31_parlio_event_head,
                                    __ATOMIC_ACQUIRE);
    if (head - s_s31_parlio_pending_event.sequence >=
        S31_PARLIO_SAFE_NODE_BACKLOG) {
      s_s31_parlio_pending_event = (s31_parlio_event_t){0};
      s_s31_parlio_pending_offset = 0u;
      s_s31_parlio_event_tail = head;
      ++s_s31_parlio_callback_overruns;
      ++s_s31_parlio_discontinuities;
    }
  }
  const bool host_real = s31_parlio_host_real_selected(config);
  const bool usb_iq4 = s31_parlio_usb_iq4_selected(config);
  const uint32_t desired_width = (host_real || usb_iq4) ? 8u : 16u;
  if (s_s31_parlio_unit != NULL &&
      s_s31_parlio_data_width != desired_width) {
    /* STREAM_START is the first point at which the transport owner and wire
     * format are known. Reconfigure the retained maximum-width unit while it
     * is stopped; rebuilding its GDMA objects after network startup is not
     * reliable in the small fragmented internal heap. */
    s31_gpio_diag_prepare(config);
  }
  const uint32_t output_chunks =
      host_real ? S31_CONTINUOUS_OUTPUT_CHUNKS : S31_PARLIO_BATCH_CHUNKS;
  const uint32_t frame_payload_bytes =
      host_real ? 4u * IQ_CHUNK_SAMPLE_WORDS
                : (usb_iq4 ? IQ_CHUNK_SAMPLE_WORDS
                           : 2u * IQ_CHUNK_SAMPLE_WORDS);
  stream_frame_t *slots[S31_PARLIO_BATCH_CHUNKS];
  if (s_s31_parlio_unit == NULL) {
    return;
  }
  /* Native USB can carry the already compact PARLIO IQ bytes straight from
   * the producer into DWC2's coherent slab.  The ordinary stream ring would
   * add a second PSRAM-to-PSRAM copy on core 0 and is the measured 16 MSa/s
   * bottleneck. Ethernet retains its zero-copy stream-ring path. */
  const bool direct_usb_compact =
      !host_real && iq_network_stream_owner() == IQ_STREAM_OWNER_USB &&
      (iq_usb_stream_format() == IQ_USB_FORMAT_INT8 || usb_iq4);
  const bool slots_reserved =
      direct_usb_compact
          ? (usb_iq4 ? iq_usb_direct_reserve_iq4(output_chunks, slots)
                     : iq_usb_direct_reserve_iq8(output_chunks, slots))
          : stream_ring_reserve_slots(output_chunks, slots);
  if (!slots_reserved) {
    return;
  }
  if (!s_s31_parlio_running) {
    s31_gpio_diag_start(config);
  }
  if (!s_s31_parlio_running) {
    if (direct_usb_compact) {
      iq_usb_direct_abort();
    } else {
      stream_ring_release_reserved(output_chunks);
    }
    return;
  }

  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
  uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                       MODEM_WIFI_AGC_AGCRD3_STATE_S;
  for (uint32_t frame = 0u; frame < output_chunks; ++frame) {
    iq_chunk_report_meta_t meta = {
        .source_chunk_index = s_s31_probe_source_chunk + frame,
        .adc_decimation = adc_decimation_value(config),
        .sample_rate_hz = s31_parlio_capture_rate_hz(config),
        .center_freq_mhz = config->radio.rf_freq_hz / HZ_PER_MHZ,
        .rx_gain = rx_gain,
        .agc_state = agc_state,
        .software_agc_active = sdr_agc_active(),
        .agc_robust_peak = sdr_agc_last_peak(),
        .agc_gain_changes = sdr_agc_gain_changes(),
        .dropped_chunks = s_stream_dropped_chunks,
        .bank_timer_late_misses = s_s31_parlio_callback_overruns,
        .bank_timer_write_ptr = s_s31_parlio_discontinuities,
        .producer_wake_write_ptr = 0u,
    };
    stream_ring_prepare_iq_chunk(slots[frame], &meta);
    if (host_real) {
      memcpy(slots[frame]->iq.magic, STREAM_FRAME_MAGIC_REAL8, 4u);
    } else if (usb_iq4) {
      memcpy(slots[frame]->iq.magic, STREAM_FRAME_MAGIC_IQ4, 4u);
    } else {
      slots[frame]->iq.magic[3] = '8';
    }
  }

  uint32_t batch_byte = 0u;
  const uint32_t batch_bytes = output_chunks * frame_payload_bytes;
  while (batch_byte < batch_bytes) {
    if (s_s31_parlio_pending_offset >=
        s_s31_parlio_pending_event.bytes) {
      if (!s31_parlio_next_event(&s_s31_parlio_pending_event)) {
        if (direct_usb_compact) {
          iq_usb_direct_abort();
        } else {
          stream_ring_release_reserved(output_chunks);
        }
        if (!iq_network_stream_armed()) {
          return;
        }
        s_stream_dropped_chunks += output_chunks;
        s_s31_probe_source_chunk += output_chunks;
        return;
      }
      s_s31_parlio_pending_offset = 0u;
    }
    uint32_t available = s_s31_parlio_pending_event.bytes -
                         s_s31_parlio_pending_offset;
    uint32_t event_take = batch_bytes - batch_byte;
    if (event_take > available) {
      event_take = available;
    }
    uint32_t copied = 0u;
    while (copied < event_take) {
      uint32_t frame = batch_byte / frame_payload_bytes;
      uint32_t byte = batch_byte % frame_payload_bytes;
      uint32_t take = event_take - copied;
      if (take > frame_payload_bytes - byte) {
        take = frame_payload_bytes - byte;
      }
      uint8_t *payload =
          (uint8_t *)slots[frame] + offsetof(iq_chunk_t, samples);
      memcpy(payload + byte, s_s31_parlio_pending_event.data +
                                 s_s31_parlio_pending_offset + copied,
             take);
      copied += take;
      batch_byte += take;
    }
    s_s31_parlio_pending_offset += event_take;
    s31_parlio_release_pending_event();
  }

  uint32_t backlog =
      __atomic_load_n(&s_s31_parlio_event_head, __ATOMIC_ACQUIRE) -
      s_s31_parlio_event_tail;
  for (uint32_t frame = 0u; frame < output_chunks; ++frame) {
    uint32_t *wire_crc = (uint32_t *)(
        (uint8_t *)slots[frame] + offsetof(iq_chunk_t, samples) +
        frame_payload_bytes);
    *wire_crc = 0u;
    slots[frame]->iq.dropped_chunks = s_stream_dropped_chunks;
    slots[frame]->iq.bank_timer_late_misses = s_s31_parlio_callback_overruns;
    slots[frame]->iq.bank_timer_write_ptr = s_s31_parlio_discontinuities;
    slots[frame]->iq.producer_wake_write_ptr = backlog;
  }
  if (direct_usb_compact) {
    if (!iq_usb_direct_commit(slots, output_chunks)) {
      s_stream_dropped_chunks += output_chunks;
    }
  } else {
    stream_ring_commit_reserved(output_chunks);
  }
  s_s31_probe_source_chunk += output_chunks;
  s_emit_chunk = s_s31_probe_source_chunk;
  s_source_chunk_index = s_emit_chunk;
}

static uint32_t IRAM_ATTR s31_ring_guard_words(const trigger_plan_t *plan) {
  if (plan->mode == IQ_TRIGGER_MODE_INTERVAL && plan->interval_chunks == 1u &&
      plan->interval_duration_chunks != 0u) {
    return S31_LIVE_COPY_GUARD_WORDS;
  }
  return S31_RING_GUARD_WORDS;
}

static bool s31_pulse_tcm_before_copy(const capture_config_t *config) {
  return (config->iq_engine.adc_source_sel &
          S31_ADC_SOURCE_PULSE_TCM_BEFORE_COPY) != 0u;
}

static uint32_t s31_pulse_tcm_dwell(const capture_config_t *config) {
  if ((config->iq_engine.adc_source_sel & S31_ADC_SOURCE_LONG_GATE_DWELL) !=
      0u) {
    return 8u * DUMP_BANK_WORDS;
  }
  uint32_t hardware_decimation_field =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_HW_DECIM_M) >>
      S31_ADC_SOURCE_HW_DECIM_S;
  if (hardware_decimation_field == 7u) {
    uint32_t dwell = 32768u;
    if ((config->iq_engine.adc_source_sel & S31_ADC_SOURCE_FIELD7_DWELL_X2) !=
        0u) {
      dwell *= 2u;
    }
    if ((config->iq_engine.adc_source_sel & S31_ADC_SOURCE_FIELD7_DWELL_X4) !=
        0u) {
      dwell *= 4u;
    }
    return dwell;
  }
  uint32_t dwell_sel =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_PULSE_DWELL_M) >>
      S31_ADC_SOURCE_PULSE_DWELL_S;
  /* Amortize each settled snapshot over a safe output batch. The encoded
   * 1,024-cycle dwell leaves stale regions. A 2,048-cycle dwell substantially
   * improves OFDM coherence through decimation 8, while decimations 9 and 10
   * use a conservative 1,280-cycle dwell to preserve every source chunk.
   * Explicit diagnostic source values retain their encoded power-of-two
   * dwell. */
  if (config->iq_engine.adc_source_sel == ADC_DUMP_SOURCE_CONTINUOUS_RX_IQ) {
    return s31_software_decimation(config) <= 8u ? 2048u : 1280u;
  }
  return 64u << dwell_sel;
}

static uint32_t s31_pulse_tcm_mask(const capture_config_t *config) {
  uint32_t mask =
      (config->iq_engine.adc_source_sel & S31_ADC_SOURCE_TCM_MASK_M) >>
      S31_ADC_SOURCE_TCM_MASK_S;
  return (mask == 0u ? 0xffu : mask) << 24;
}

static int32_t s31_live_copy_offset_chunks(const capture_config_t *config) {
  (void)config;
  return 0;
}

static uint32_t s31_output_chunk_words(const capture_config_t *config) {
  return IQ_CHUNK_SAMPLE_WORDS * s31_software_decimation(config);
}

static void IRAM_ATTR s31_msync_ring_span(uint32_t start_word,
                                          uint32_t word_count) {
  uint32_t off = dump_ring_mod(start_word);
  uint32_t first = DUMP_BANK_WORDS - off;
  if (first > word_count) {
    first = word_count;
  }
  (void)esp_cache_msync((void *)&DUMP_BASE[off], first * sizeof(uint32_t),
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  if (word_count > first) {
    (void)esp_cache_msync((void *)DUMP_BASE,
                          (word_count - first) * sizeof(uint32_t),
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  }
}

static void s31_pulse_tcm_dump_gate(const capture_config_t *config) {
  if ((config->iq_engine.adc_source_sel & S31_ADC_SOURCE_TCM_ALWAYS_ON) != 0u) {
    return;
  }
  /* TCM dump access is visible to both HP cores.  Park the peer in the IPC
   * high-priority ISR and mask local interrupts so neither scheduler can touch
   * TCM while the modem gate owns it. */
  esp_ipc_isr_stall_other_cpu();
  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG,
                   s31_pulse_tcm_mask(config));
  __asm__ __volatile__("fence" ::: "memory");
  uint32_t dwell = s31_pulse_tcm_dwell(config);
  for (volatile uint32_t i = 0u; i < dwell; ++i) {
    __asm__ __volatile__("nop");
  }
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  __asm__ __volatile__("fence" ::: "memory");
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  esp_ipc_isr_release_other_cpu();
}

static bool s31_pipelined_stage_enabled(const capture_config_t *config) {
  return (config->iq_engine.adc_source_sel &
          S31_ADC_SOURCE_PIPELINED_STAGE) != 0u;
}

/* Copy two already-completed dump words during ownership slots synchronized
 * to a writer-pointer edge.  At 4 MS/s the 300 MHz HP core has 75 cycles
 * between samples.  If each closed interval is shorter than that and begins
 * immediately after an edge, the modem never reaches a write while its TCM
 * route is disconnected.  Pointer advance is measured across every slot;
 * Pluto phase continuity remains the independent end-to-end oracle. */
static void IRAM_ATTR s31_micro_gate_capture(
    volatile uint32_t *dst, uint32_t start_raw, uint32_t word_count,
    uint32_t gate_mask, uint32_t *pointer_advances,
    uint32_t *max_gate_cycles) {
  const uint32_t burst_words = 2u;
  uint32_t advances = 0u;
  uint32_t max_cycles = 0u;

  for (uint32_t copied = 0u; copied < word_count; copied += burst_words) {
    uint32_t source = start_raw + copied;
    if (source >= DUMP_BANK_WORDS) {
      source -= DUMP_BANK_WORDS;
    }
    volatile uint32_t *src = &DUMP_BASE[source];
    uint32_t target = copied + burst_words;
    uint32_t edge_raw;
    do {
      edge_raw = adc_dump_store_addr_raw();
      uint32_t ready = edge_raw >= start_raw
                           ? edge_raw - start_raw
                           : DUMP_BANK_WORDS - start_raw + edge_raw;
      if (ready >= target) {
        break;
      }
    } while (true);

    /* Even when a backlog exists, wait for a fresh edge so the gate closes at
     * the beginning rather than the end of a sample interval. */
    uint32_t before_raw;
    do {
      before_raw = adc_dump_store_addr_raw();
    } while (before_raw == edge_raw);

    uint32_t close_cycle = esp_cpu_get_cycle_count();
    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
    __asm__ __volatile__("fence rw, rw" ::: "memory");

    /* start_raw and the ring length are two-word aligned, so a burst never
     * straddles the physical end of the aperture. */
    uint32_t v0 = src[0];
    uint32_t v1 = src[1];
    dst[copied + 0u] = v0;
    dst[copied + 1u] = v1;

    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, gate_mask);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    uint32_t gate_cycles = esp_cpu_get_cycle_count() - close_cycle;
    if (gate_cycles > max_cycles) {
      max_cycles = gate_cycles;
    }
    uint32_t after_raw = adc_dump_store_addr_raw();
    advances += after_raw >= before_raw
                    ? after_raw - before_raw
                    : DUMP_BANK_WORDS - before_raw + after_raw;
  }

  *pointer_advances = advances;
  *max_gate_cycles = max_cycles;
}

/* The normal S31 snapshot path writes every frame directly to PSRAM after the
 * dump switch closes.  At 8 MS/s that keeps the writer disconnected for about
 * seven chunks per 16-chunk batch.  This diagnostic backend pipelines through
 * the reserved 64 KiB TCM guard instead:
 *
 *   writer open:  move the previous staging block to PSRAM while acquiring;
 *   writer closed: copy the new 64 KiB block TCM-to-TCM;
 *
 * The entire transaction executes from flash with a PSRAM producer stack and
 * both HP cores quiesced.  Volatile word loops deliberately prevent GCC from
 * substituting an internal-RAM memcpy while the fabric switch is open. */
static uint32_t __attribute__((noinline)) s31_stage_capture_transaction(
    const capture_config_t *config, stream_frame_t **slots,
    uint32_t requested_frames, s31_stage_batch_diag_t *diag) {
  uint32_t requested_batches = requested_frames / S31_STAGE_CHUNKS;
  if (requested_batches == 0u) {
    return 0u;
  }

  uint32_t gate_mask = s31_pulse_tcm_mask(config);
  const bool micro_gate =
      config->rx_filter.rx_filter_override == S31_MICRO_GATE_OVERRIDE;
  const bool ring_cursor =
      config->rx_filter.rx_filter_override == S31_RING_CURSOR_OVERRIDE;
  bool stage_valid = s_s31_stage_valid;
  uint32_t stage_source_chunk = s_s31_stage_source_chunk;
  uint32_t prev_end_raw = s_s31_stage_prev_end_raw;
  bool have_prev_end = s_s31_stage_have_prev_end;
  uint32_t produced_batches = 0u;

  /* The live dump aperture remaps TCM as observed by both HP cores.  Keep the
   * peer parked for every transport; letting Ethernet/lwIP run on core 0 while
   * the gate is open causes arbitrary ROM/lwIP load faults.  The Ethernet
   * caller deliberately limits this to one 14-frame acquisition window so
   * the GMAC interrupt blackout remains bounded. */
  esp_ipc_isr_stall_other_cpu();
  taskENTER_CRITICAL(&s_tcm_dump_gate_mux);
  while (produced_batches < requested_batches) {
    uint32_t output_batch = produced_batches;
    uint32_t output_source_chunk = stage_source_chunk;

    reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, gate_mask);
    __asm__ __volatile__("fence" ::: "memory");
    uint32_t start_raw;
    uint32_t writer_raw;
    if (ring_cursor && have_prev_end) {
      /* Keep consuming immediately after the preceding block. The writer
       * pointer proves availability; it must never redefine the consumer. */
      start_raw = prev_end_raw;
      writer_raw = dump_ring_mod(adc_dump_store_addr_raw());
    } else {
      /* The initial cursor is aligned so both pieces of a possible PIE ring
       * wrap remain valid operands. */
      do {
        writer_raw = dump_ring_mod(adc_dump_store_addr_raw());
        start_raw = writer_raw;
      } while ((start_raw & 3u) != 0u);
    }

    if (ring_cursor) {
      if (!have_prev_end) {
        stage_source_chunk = 0u;
      }
    } else {
      uint32_t gap_words =
          have_prev_end ? dump_ring_delta(start_raw, prev_end_raw) : 0u;
      uint32_t gap_chunks =
          (gap_words + IQ_CHUNK_SAMPLE_WORDS - 1u) / IQ_CHUNK_SAMPLE_WORDS;
      if (have_prev_end) {
        s_stream_dropped_chunks += gap_chunks;
        stage_source_chunk += S31_STAGE_CHUNKS + gap_chunks;
      } else {
        stage_source_chunk = 0u;
      }
    }

    if (stage_valid) {
      diag[output_batch].source_chunk = output_source_chunk;
      diag[output_batch].start_raw = prev_end_raw;
      diag[output_batch].end_raw = start_raw;
#if CONFIG_ESP_SDR_TRANSPORT_USB
      /* Header stores target the narrow write-through USB slab. Hide them
       * behind acquisition: start_raw was already sampled, so the writer's
       * progress during metadata preparation remains part of this block. */
      s31_prepare_stage_frames(
          config, &slots[output_batch * S31_STAGE_CHUNKS],
          S31_STAGE_CHUNKS, &diag[output_batch]);
#endif
      for (uint32_t chunk = 0u; chunk < S31_STAGE_CHUNKS; ++chunk) {
        stream_frame_t *slot =
            slots[output_batch * S31_STAGE_CHUNKS + chunk];
        volatile uint32_t *dst = (volatile uint32_t *)(
            (uintptr_t)slot + offsetof(iq_chunk_t, samples));
        uint32_t stage_off = chunk * IQ_CHUNK_SAMPLE_WORDS;
        for (uint32_t word = 0u; word < IQ_CHUNK_SAMPLE_WORDS; ++word) {
          dst[word] = S31_STAGE_BASE[stage_off + word];
        }
      }
#if CONFIG_ESP_SDR_TRANSPORT_USB
      s31_finalize_usb_stage_batch(
          &slots[output_batch * S31_STAGE_CHUNKS], S31_STAGE_CHUNKS,
          &diag[output_batch]);
#endif
    }

    uint32_t end_raw;
    if (micro_gate) {
      uint32_t pointer_advances;
      uint32_t max_gate_cycles;
      s31_micro_gate_capture(S31_STAGE_BASE, start_raw, S31_STAGE_WORDS,
                             gate_mask, &pointer_advances, &max_gate_cycles);
      end_raw = dump_ring_mod(start_raw + S31_STAGE_WORDS);
      s_s31_micro_gate_advances += pointer_advances;
      if (max_gate_cycles > s_s31_micro_gate_max_cycles) {
        s_s31_micro_gate_max_cycles = max_gate_cycles;
      }
      ++s_s31_micro_gate_batches;
      /* Runtime and transport are not yet safe under writer ownership.  Close
       * once per transaction here; the next iteration measures this remaining
       * coarse service interval separately as its normal inter-stage gap. */
      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
      __asm__ __volatile__("fence rw, rw" ::: "memory");
    } else if (ring_cursor) {
      uint32_t sample_cycles = adc_dump_sample_cycles(config);
      uint32_t cpu_cycles_per_sample =
          sample_cycles * CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ /
          (ADC_DUMP_CLOCK_HZ / 1000000u);
      if (cpu_cycles_per_sample == 0u) {
        cpu_cycles_per_sample = 1u;
      }

      /* A modulo pointer cannot distinguish empty from producer-lapped.
       * Independently bound writer progress with the CPU cycle counter. Any
       * lap is reported and resynchronized, and must remain zero in sustained
       * operation. */
      uint32_t writer_cycle = esp_cpu_get_cycle_count();
      if (s_s31_cursor_have_writer_cycle) {
        uint32_t elapsed_samples =
            (writer_cycle - s_s31_cursor_last_writer_cycle) /
            cpu_cycles_per_sample;
        if (s_s31_cursor_last_backlog + elapsed_samples >= DUMP_BANK_WORDS) {
          ++s_s31_cursor_overruns;
          uint32_t lost_words =
              s_s31_cursor_last_backlog + elapsed_samples -
              (DUMP_BANK_WORDS - 1u);
          uint32_t lost_chunks =
              (lost_words + IQ_CHUNK_SAMPLE_WORDS - 1u) /
              IQ_CHUNK_SAMPLE_WORDS;
          s_stream_dropped_chunks += lost_chunks;
          stage_source_chunk += lost_chunks;
          do {
            writer_raw = dump_ring_mod(adc_dump_store_addr_raw());
            start_raw = writer_raw;
          } while ((start_raw & 3u) != 0u);
        }
      }

      do {
        writer_raw = dump_ring_mod(adc_dump_store_addr_raw());
      } while (dump_ring_delta(writer_raw, start_raw) < S31_STAGE_WORDS);

      uint32_t end_cursor = dump_ring_mod(start_raw + S31_STAGE_WORDS);
      uint32_t backlog = dump_ring_delta(writer_raw, end_cursor);
      if (backlog < s_s31_cursor_min_backlog) {
        s_s31_cursor_min_backlog = backlog;
      }
      if (backlog > s_s31_cursor_max_backlog) {
        s_s31_cursor_max_backlog = backlog;
      }
      ++s_s31_cursor_batches;
      s_s31_cursor_last_writer_cycle = esp_cpu_get_cycle_count();
      s_s31_cursor_last_backlog = backlog;
      s_s31_cursor_have_writer_cycle = true;
      end_raw = end_cursor;

      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
      __asm__ __volatile__("fence" ::: "memory");
    } else {
      do {
        end_raw = dump_ring_mod(adc_dump_store_addr_raw());
      } while (dump_ring_delta(end_raw, start_raw) < S31_STAGE_WORDS);

      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
      __asm__ __volatile__("fence" ::: "memory");
    }

    if (stage_valid) {
      ++produced_batches;
    }

    uint32_t start_word = dump_ring_mod(start_raw);
    /* The live aperture and staging guard are uncached internal TCM.  Cache
     * synchronization here only lengthens the writer-off interval; the dump
     * fabric handoff plus the full memory fence above makes the completed
     * snapshot directly visible to PIE. */
    if (!micro_gate) {
      uint32_t first_words = DUMP_BANK_WORDS - start_word;
      if (first_words > S31_STAGE_WORDS) {
        first_words = S31_STAGE_WORDS;
      }
      s31_pie_memcpy_aligned((void *)(uintptr_t)S31_STAGE_BASE,
                             (const void *)(uintptr_t)&DUMP_BASE[start_word],
                             first_words * sizeof(uint32_t));
      if (first_words < S31_STAGE_WORDS) {
        s31_pie_memcpy_aligned(
            (void *)(uintptr_t)&S31_STAGE_BASE[first_words],
            (const void *)(uintptr_t)DUMP_BASE,
            (S31_STAGE_WORDS - first_words) * sizeof(uint32_t));
      }
    }
    stage_valid = true;
    if (ring_cursor && have_prev_end) {
      stage_source_chunk += S31_STAGE_CHUNKS;
    }
    have_prev_end = true;
    prev_end_raw = end_raw;
  }
  taskEXIT_CRITICAL(&s_tcm_dump_gate_mux);
  esp_ipc_isr_release_other_cpu();

  s_s31_stage_valid = stage_valid;
  s_s31_stage_source_chunk = stage_source_chunk;
  s_s31_stage_prev_end_raw = prev_end_raw;
  s_s31_stage_have_prev_end = have_prev_end;
  return produced_batches * S31_STAGE_CHUNKS;
}

static void IRAM_ATTR process_s31_pipelined_stage(
    const capture_config_t *config) {
#if CONFIG_ESP_SDR_TRANSPORT_USB
  /* One 14-frame transfer lasts less than the following 14-frame acquisition
   * at the sustainable full-precision USB rate. Ping-pong whole transfers;
   * asking for two at once prevents the completion ISR from rearming DWC2
   * while core 0 is parked and creates a multi-millisecond writer-off gap.
   * Run several transactions per producer turn so scheduler/config-service
   * latency is paid only once per group. */
  for (uint32_t round = 0u; round < 4u; ++round) {
    uint32_t n = S31_STAGE_CHUNKS;
    stream_frame_t *slots[S31_STAGE_CHUNKS];
    if (!iq_usb_direct_reserve(n, slots)) {
      return;
    }
    s31_stage_batch_diag_t diag[1u];
    uint32_t produced =
        s31_stage_capture_transaction(config, slots, n, diag);
    if (produced != n) {
      iq_usb_direct_abort();
      return;
    }
    if (config->rx_filter.rx_filter_override == S31_MICRO_GATE_OVERRIDE &&
        (s_s31_micro_gate_batches & 63u) == 0u) {
      ESP_LOGI("iq_capture",
               "micro_gate batches=%" PRIu32 " pointer_advances=%" PRIu32
               " max_closed_cycles=%" PRIu32,
               s_s31_micro_gate_batches, s_s31_micro_gate_advances,
               s_s31_micro_gate_max_cycles);
    }
    if (config->rx_filter.rx_filter_override == S31_RING_CURSOR_OVERRIDE &&
        (s_s31_cursor_batches & 63u) == 0u) {
      ESP_LOGI("iq_capture",
               "ring_cursor batches=%" PRIu32 " backlog=%" PRIu32
               "..%" PRIu32 " overruns=%" PRIu32,
               s_s31_cursor_batches, s_s31_cursor_min_backlog,
               s_s31_cursor_max_backlog, s_s31_cursor_overruns);
    }

    /* Feed every transaction so the DCO servo converges at its intended
     * cadence. Its internal 4-ms limiter makes non-update calls inexpensive. */
    const volatile uint32_t *servo_words =
        (const volatile uint32_t *)((uintptr_t)slots[produced - 1u] +
                                    offsetof(iq_chunk_t, samples));
    dcoc_feed_chunk(servo_words);
    if (!iq_usb_direct_commit(slots, n)) {
      s_stream_dropped_chunks += n;
      return;
    }
    s_emit_chunk = diag[0].source_chunk + S31_STAGE_CHUNKS;
    s_source_chunk_index = s_emit_chunk;
    if (!iq_network_stream_armed() || config_apply_in_progress()) {
      return;
    }
  }
  return;
#else
  uint32_t available = stream_ring_available_slots();
  /* Keep the peer core and Ethernet ISR parked for only one capture window.
   * Longer transactions starve the GMAC/lwIP control path even when IQC8
   * reduces the downstream wire rate. */
  uint32_t n = available > S31_STAGE_CHUNKS ? S31_STAGE_CHUNKS : available;
  n -= n % S31_STAGE_CHUNKS;
  if (n == 0u) {
    return;
  }

  stream_frame_t *slots[IQ_STREAM_RING_CHUNKS];
  if (!stream_ring_reserve_slots(n, slots)) {
    return;
  }
  s31_stage_batch_diag_t diag[IQ_STREAM_RING_CHUNKS / S31_STAGE_CHUNKS];
  uint32_t produced =
      s31_stage_capture_transaction(config, slots, n, diag);
  if (produced == 0u) {
    stream_ring_release_reserved(n);
    return;
  }
  if (produced < n) {
    stream_ring_release_reserved(n - produced);
    n = produced;
  }

  /* The staged path bypasses fill_and_push_chunk(), which normally samples
   * each emitted chunk for the slow DCOC/AGC servos.  Service it only after
   * the TCM fabric has been returned to the CPUs: dcoc_feed() may read the
   * timer and occasionally rewrite/re-latch gain RAM, neither of which is
   * safe while the writer-owned aperture and interrupt gate are active.
   * One representative completed chunk per transaction is sufficient; the
   * servo has its own 4-ms rate limiter. */
  const volatile uint32_t *servo_words =
      (const volatile uint32_t *)((uintptr_t)slots[produced - 1u] +
                                  offsetof(iq_chunk_t, samples));
  dcoc_feed_chunk(servo_words);

  s31_prepare_stage_frames(config, slots, n, diag);
  stream_ring_commit_reserved(n);
  s_emit_chunk = diag[(n - 1u) / S31_STAGE_CHUNKS].source_chunk +
                 S31_STAGE_CHUNKS;
  s_source_chunk_index = s_emit_chunk;
  /* A full-duty producer otherwise re-enters the dual-core TCM stall before
   * lwIP can drain control traffic. Give core 0 one scheduler tick between
   * snapshots; the next source index honestly accounts for this interval. */
  vTaskDelay(pdMS_TO_TICKS(1));
#endif
}

static const uint32_t *IRAM_ATTR s31_chunk_source(
    const capture_config_t *config, uint32_t c, uint64_t abs_write,
    bool refresh_dump_window) {
  uint32_t decimation = s31_software_decimation(config);
  uint32_t output_words = s31_output_chunk_words(config);
  uint32_t chunks_per_window = DUMP_BANK_WORDS / IQ_CHUNK_SAMPLE_WORDS;
  uint64_t start_abs = (uint64_t)c * output_words;
  uint32_t start_word = (uint32_t)(start_abs % DUMP_BANK_WORDS);
  uint32_t physical_chunk =
      (start_word / IQ_CHUNK_SAMPLE_WORDS) % chunks_per_window;
  uint64_t abs_chunk = abs_write / output_words;
  int32_t physical_delta =
      (int32_t)physical_chunk - (int32_t)(abs_chunk % chunks_per_window);
  if (physical_delta > 0) {
    physical_delta -= (int32_t)chunks_per_window;
  }
  int64_t source_abs = (int64_t)abs_chunk + physical_delta;
  uint32_t source_chunk = source_abs > 0 ? (uint32_t)source_abs : 0u;
  s_s31_copy_ctrl_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
  s_s31_copy_mode_before_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE);
  if (refresh_dump_window && s31_pulse_tcm_before_copy(config)) {
    s31_pulse_tcm_dump_gate(config);
  }
  s_s31_copy_mode_after_diag = adc_dump_store_addr_raw();
  s_s31_copy_physical_chunk = physical_chunk;
  s_s31_copy_source_chunk = source_chunk;
  if (decimation == 1u) {
    uint32_t off = dump_ring_mod(start_word);
    (void)esp_cache_msync((void *)&DUMP_BASE[off],
                          IQ_CHUNK_SAMPLE_WORDS * sizeof(uint32_t),
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    return (const uint32_t *)&DUMP_BASE[off];
  }
  s31_msync_ring_span(start_word, output_words);
  uint32_t source_index = dump_ring_mod(start_word);
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; ++i) {
    s_s31_decim_buf[i] = DUMP_BASE[source_index];
    source_index += decimation;
    if (source_index >= DUMP_BANK_WORDS)
      source_index -= DUMP_BANK_WORDS;
  }
  return s_s31_decim_buf;
}

static const uint32_t *IRAM_ATTR
s31_exact_chunk_source(const capture_config_t *config, uint32_t source_chunk) {
  uint32_t decimation = s31_software_decimation(config);
  uint32_t output_words = s31_output_chunk_words(config);
  uint32_t chunks_per_window = DUMP_BANK_WORDS / IQ_CHUNK_SAMPLE_WORDS;
  uint64_t start_abs = (uint64_t)source_chunk * output_words;
  uint32_t start_word = (uint32_t)(start_abs % DUMP_BANK_WORDS);
  uint32_t physical_chunk =
      (start_word / IQ_CHUNK_SAMPLE_WORDS) % chunks_per_window;
  s_s31_copy_ctrl_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
  s_s31_copy_mode_before_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE);
  if (s31_pulse_tcm_before_copy(config)) {
    s31_pulse_tcm_dump_gate(config);
  }
  s_s31_copy_mode_after_diag = adc_dump_store_addr_raw();
  s_s31_copy_physical_chunk = physical_chunk;
  s_s31_copy_source_chunk = source_chunk;
  if (decimation == 1u) {
    uint32_t off = dump_ring_mod(start_word);
    (void)esp_cache_msync((void *)&DUMP_BASE[off],
                          IQ_CHUNK_SAMPLE_WORDS * sizeof(uint32_t),
                          ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    return (const uint32_t *)&DUMP_BASE[off];
  }
  s31_msync_ring_span(start_word, output_words);
  uint32_t source_index = dump_ring_mod(start_word);
  for (uint32_t i = 0u; i < IQ_CHUNK_SAMPLE_WORDS; ++i) {
    s_s31_decim_buf[i] = DUMP_BASE[source_index];
    source_index += decimation;
    if (source_index >= DUMP_BANK_WORDS)
      source_index -= DUMP_BANK_WORDS;
  }
  return s_s31_decim_buf;
}

static void IRAM_ATTR s31_chunk_copy_done(uint32_t c) { (void)c; }

static uint64_t IRAM_ATTR
s31_current_abs_write(const capture_config_t *config) {
  int64_t now_us = esp_timer_get_time();
  uint32_t raw_sa = adc_dump_store_addr_raw();
  uint32_t sa = dump_ring_mod(adc_dump_store_addr_from_raw(raw_sa));
  uint32_t delta_mod = dump_ring_delta(sa, s_isr_prev_sa);
  uint32_t delta = delta_mod;
  uint32_t sample_cycles = adc_dump_write_sample_cycles(config);
  if (s_s31_abs_track_us != 0 && sample_cycles != 0u) {
    uint64_t elapsed_us = (uint64_t)(now_us - s_s31_abs_track_us);
    uint64_t expected_delta = (elapsed_us * (uint64_t)ADC_DUMP_CLOCK_HZ) /
                              ((uint64_t)sample_cycles * 1000000ull);
    if (expected_delta > (DUMP_BANK_WORDS / 2u)) {
      uint64_t wraps = 0u;
      if (expected_delta > delta_mod) {
        wraps = (expected_delta - delta_mod + (DUMP_BANK_WORDS / 2u)) /
                DUMP_BANK_WORDS;
      }
      uint64_t delta64 = (uint64_t)delta_mod + wraps * DUMP_BANK_WORDS;
      delta = delta64 > UINT32_MAX ? UINT32_MAX : (uint32_t)delta64;
    }
  }
  s_s31_abs_write_accum += delta;
  s_isr_wrap_count = (uint32_t)(s_s31_abs_write_accum / DUMP_BANK_WORDS);
  s_isr_prev_sa = sa;
  s_s31_abs_track_us = now_us;
  s_dbg_lo = raw_sa;
  uint64_t abs_write = s_s31_abs_write_accum;
  s_s31_diag_abs_write = abs_write;
  return abs_write;
}

static bool IRAM_ATTR s31_wait_for_abs_write(const capture_config_t *config,
                                             uint64_t target_abs,
                                             uint64_t *abs_write) {
  int64_t wait_start_us = esp_timer_get_time();
  uint32_t sample_cycles = adc_dump_write_sample_cycles(config);
  while (*abs_write < target_abs) {
    uint64_t remaining_words64 = target_abs - *abs_write;
    uint32_t remaining_words = remaining_words64 > UINT32_MAX
                                   ? UINT32_MAX
                                   : (uint32_t)remaining_words64;
    uint32_t remaining_us =
        sample_cycles != 0u
            ? (remaining_words * sample_cycles) /
                  (ADC_DUMP_CLOCK_HZ / 1000000u)
            : 0u;
    if (remaining_us > S31_POLL_WAKE_MARGIN_US + 1000u) {
      vTaskDelay(
          pdMS_TO_TICKS((remaining_us - S31_POLL_WAKE_MARGIN_US) / 1000u));
    } else if (remaining_us > 80u) {
      esp_rom_delay_us(remaining_us - S31_POLL_CLOSE_US);
    } else {
      __asm__ __volatile__("nop");
    }
    *abs_write = s31_current_abs_write(config);
    if ((uint32_t)(esp_timer_get_time() - wait_start_us) > 100000u) {
      return false;
    }
  }
  return true;
}

static void IRAM_ATTR process_s31_live_pulse_interval(
    const capture_config_t *config, const trigger_plan_t *plan,
    uint64_t abs_write) {
  uint32_t output_words = s31_output_chunk_words(config);
  if (plan->interval_chunks == 1u &&
      plan->interval_duration_chunks != 0u) {
    if (s31_pipelined_stage_enabled(config) &&
        s31_software_decimation(config) == 1u && !stream_output_int8()) {
      process_s31_pipelined_stage(config);
      return;
    }
    if (abs_write <= S31_LIVE_COPY_GUARD_WORDS) {
      return;
    }
    uint32_t ready_chunk = (uint32_t)(
        (abs_write - S31_LIVE_COPY_GUARD_WORDS) / output_words);
    if (!s_s31_ring_started) {
      s_emit_chunk = ready_chunk > 0u ? ready_chunk - 1u : 0u;
      s_s31_ring_started = true;
    }
    uint64_t low_abs = abs_write > DUMP_BANK_WORDS
                           ? abs_write - DUMP_BANK_WORDS + output_words
                           : 0u;
    uint32_t low_chunk =
        (uint32_t)((low_abs + output_words - 1u) / output_words);
    if (s_emit_chunk < low_chunk) {
      s_stream_dropped_chunks += low_chunk - s_emit_chunk;
      s_emit_chunk = low_chunk;
    }
    if (ready_chunk <= s_emit_chunk) {
      s_source_chunk_index = s_emit_chunk;
      return;
    }
    uint32_t n = ready_chunk - s_emit_chunk;
    uint32_t safe_chunks =
        (DUMP_BANK_WORDS - S31_LIVE_COPY_GUARD_WORDS) / output_words;
    uint32_t desired_batch = safe_chunks > 3u ? safe_chunks - 3u : 1u;
    if (n < desired_batch) {
      s_source_chunk_index = s_emit_chunk;
      return;
    }
    if (n > S31_RING_MAX_CHUNKS_PER_POLL) {
      n = S31_RING_MAX_CHUNKS_PER_POLL;
    }
    uint32_t available = stream_ring_available_slots();
    if (n > available) {
      n = available;
    }
    if (n == 0u) {
      return;
    }
    stream_frame_t *slots[S31_RING_MAX_CHUNKS_PER_POLL];
    if (!stream_ring_reserve_slots(n, slots)) {
      return;
    }
    uint32_t sample_cycles = adc_dump_sample_cycles(config);
    uint32_t sample_rate_hz =
        sample_cycles != 0u ? ADC_DUMP_CLOCK_HZ / sample_cycles : 0u;
    uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
    uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
    uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                         MODEM_WIFI_AGC_AGCRD3_STATE_S;
    s31_pulse_tcm_dump_gate(config);
    s_s31_copy_ctrl_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
    s_s31_copy_mode_before_diag = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE);
    s_s31_copy_mode_after_diag = adc_dump_store_addr_raw();
    uint32_t decimation = s31_software_decimation(config);
    for (uint32_t i = 0u; i < n; ++i) {
      uint32_t source_chunk = s_emit_chunk + i;
      uint32_t start_word = (uint32_t)(
          ((uint64_t)source_chunk * output_words) % DUMP_BANK_WORDS);
      fill_and_push_chunk_strided(config, source_chunk, sample_rate_hz,
                                  rx_gain, agc_state, slots[i], start_word,
                                  decimation);
    }
    stream_ring_commit_reserved(n);
    s_emit_chunk += n;
    s_source_chunk_index = s_emit_chunk;
    return;
  }
  uint32_t abs_chunk = (uint32_t)(abs_write / output_words);
  if (!s_s31_ring_started) {
    s_emit_chunk = trigger_interval_next_selected(plan, abs_chunk);
    s_s31_ring_started = true;
  }
  if (abs_chunk < s_emit_chunk) {
    return;
  }

  int32_t offset = s31_live_copy_offset_chunks(config);
  int64_t copy_chunk_i64 = (int64_t)s_emit_chunk - offset;
  if (copy_chunk_i64 < 0) {
    copy_chunk_i64 = 0;
  }
  uint32_t copy_chunk = (uint32_t)copy_chunk_i64;
  uint64_t target_ready_abs =
      ((uint64_t)copy_chunk + 1u) * output_words +
      S31_LIVE_COPY_GUARD_WORDS;
  if (!s31_wait_for_abs_write(config, target_ready_abs, &abs_write)) {
    ++s_stream_dropped_chunks;
    s_emit_chunk = trigger_interval_next_selected(plan, abs_chunk + 1u);
    s_source_chunk_index = s_emit_chunk;
    return;
  }

  uint64_t stale_words =
      abs_write - ((uint64_t)s_emit_chunk * output_words);
  if (stale_words >= DUMP_BANK_WORDS - output_words) {
    uint32_t new_emit_chunk = abs_write / output_words;
    s_stream_dropped_chunks +=
        trigger_interval_selected_count(plan, s_emit_chunk, new_emit_chunk);
    s_emit_chunk = trigger_interval_next_selected(plan, new_emit_chunk);
    s_source_chunk_index = s_emit_chunk;
    return;
  }

  stream_frame_t *slots[1];
  if (!stream_ring_reserve_slots(1u, slots)) {
    ++s_stream_dropped_chunks;
    s_emit_chunk = trigger_interval_next_selected(plan, abs_chunk + 1u);
    s_source_chunk_index = s_emit_chunk;
    return;
  }

  uint32_t sample_cycles = adc_dump_sample_cycles(config);
  uint32_t sample_rate_hz =
      sample_cycles != 0u ? ADC_DUMP_CLOCK_HZ / sample_cycles : 0u;
  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
  uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                       MODEM_WIFI_AGC_AGCRD3_STATE_S;
  uint32_t logical_chunk = s_emit_chunk;
  const uint32_t *words =
      s31_chunk_source(config, copy_chunk, abs_write, true);
  fill_and_push_chunk(config, words, logical_chunk, sample_rate_hz, rx_gain,
                      agc_state, slots[0]);
  stream_ring_commit_reserved(1u);

  s_emit_chunk = trigger_interval_next_selected(plan, logical_chunk + 1u);
  s_source_chunk_index = s_emit_chunk;
}

static void IRAM_ATTR process_s31_dump_ring(const capture_config_t *config,
                                            const trigger_plan_t *plan) {
  if (s31_continuous_real_if_selected(config)) {
    /* USB transports the 4 MSa/s signed ADC stream losslessly and leaves the
     * Fs/4 analytic conversion to Soapy. This removes the firmware FIR from
     * the acquisition critical path while retaining 2 MSa/s complex output. */
    s31_process_parlio_diag(config);
    return;
  }
  if (s31_parlio_native_packed_iq_selected(config) ||
      config->rx_filter.rx_filter_override == S31_GPIO_DIAG_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_PARLIO_NATIVE_IQ_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_PARLIO_HOST_IQ_OVERRIDE ||
      config->rx_filter.rx_filter_override == S31_HP_TCM_PROBE_OVERRIDE) {
    s31_process_parlio_diag(config);
    return;
  }
#if CONFIG_ESP_SDR_TRANSPORT_USB
  if (s31_modem_diag_probe_enabled(config)) {
    s31_process_modem_diag_probe(config);
    return;
  }
#endif
  uint64_t abs_write = s31_current_abs_write(config);
  uint32_t guard_words = s31_ring_guard_words(plan);
  s_s31_diag_emit_chunk = s_emit_chunk;
  s_s31_diag_ring_started = s_s31_ring_started ? 1u : 0u;
  if (s31_pulse_tcm_before_copy(config) &&
      plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
    process_s31_live_pulse_interval(config, plan, abs_write);
    return;
  }
  if (abs_write <= guard_words) {
    s_s31_diag_c_hi = 0u;
    return;
  }
  uint32_t output_words = s31_output_chunk_words(config);
  uint64_t hi = abs_write - guard_words;
  uint32_t c_hi = (uint32_t)(hi / output_words);
  s_s31_diag_c_hi = c_hi;
  if (!s_s31_ring_started) {
    s_emit_chunk = c_hi > 0u ? c_hi - 1u : 0u;
    if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
      s_emit_chunk = trigger_interval_next_selected(plan, s_emit_chunk);
    }
    s_s31_ring_started = true;
    s_s31_diag_ring_started = 1u;
    s_s31_diag_emit_chunk = s_emit_chunk;
  }
  uint64_t emit_abs = (uint64_t)s_emit_chunk * output_words;
  if (hi > emit_abs && (hi - emit_abs) > (DUMP_BANK_WORDS - guard_words)) {
    uint64_t new_emit_abs = hi - (DUMP_BANK_WORDS / 2u);
    uint32_t new_emit_chunk = (uint32_t)(new_emit_abs / output_words);
    if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
      s_stream_dropped_chunks +=
          trigger_interval_selected_count(plan, s_emit_chunk, new_emit_chunk);
    } else {
      s_stream_dropped_chunks += new_emit_chunk - s_emit_chunk;
    }
    s_emit_chunk = new_emit_chunk;
    if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
      s_emit_chunk = trigger_interval_next_selected(plan, s_emit_chunk);
    }
    s_s31_diag_emit_chunk = s_emit_chunk;
  }
  if (c_hi <= s_emit_chunk) {
    return;
  }

  uint32_t sample_cycles = adc_dump_sample_cycles(config);
  uint32_t sample_rate_hz =
      sample_cycles != 0u ? ADC_DUMP_CLOCK_HZ / sample_cycles : 0u;
  uint32_t agc_rd3 = reg32p_read(&MODEM_WIFI_AGC.AGCRD3);
  uint32_t rx_gain = agc_rd3 & MODEM_WIFI_AGC_AGCRD3_RX_GAIN_M;
  uint32_t agc_state = (agc_rd3 & MODEM_WIFI_AGC_AGCRD3_STATE_M) >>
                       MODEM_WIFI_AGC_AGCRD3_STATE_S;

  if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
    if (plan->interval_chunks == 1u && plan->interval_duration_chunks != 0u) {
      uint64_t low_abs = abs_write > DUMP_BANK_WORDS
                             ? abs_write - DUMP_BANK_WORDS + output_words
                             : 0u;
      uint32_t low_chunk =
          (uint32_t)((low_abs + output_words - 1u) / output_words);
      if (s_emit_chunk < low_chunk) {
        s_stream_dropped_chunks += low_chunk - s_emit_chunk;
        s_emit_chunk = low_chunk;
      }
      if (s_emit_chunk >= c_hi) {
        s_source_chunk_index = s_emit_chunk;
        return;
      }
      uint32_t n = c_hi - s_emit_chunk;
      if (n > S31_RING_MAX_CHUNKS_PER_POLL) {
        n = S31_RING_MAX_CHUNKS_PER_POLL;
      }
      uint32_t available = stream_ring_available_slots();
      if (n > available) {
        n = available;
      }
      if (n == 0u) {
        s_source_chunk_index = s_emit_chunk;
        return;
      }
      stream_frame_t *slots[S31_RING_MAX_CHUNKS_PER_POLL];
      if (!stream_ring_reserve_slots(n, slots)) {
        return;
      }
      for (uint32_t i = 0u; i < n; ++i) {
        uint32_t source_chunk = s_emit_chunk + i;
        const uint32_t *words = s31_exact_chunk_source(config, source_chunk);
        fill_and_push_chunk(config, words, source_chunk, sample_rate_hz,
                            rx_gain, agc_state, slots[i]);
        s31_chunk_copy_done(source_chunk);
      }
      stream_ring_commit_reserved(n);
      s_emit_chunk += n;
      s_source_chunk_index = s_emit_chunk;
      s_s31_diag_emit_chunk = s_emit_chunk;
      return;
    }
    uint32_t latest_selected;
    if (!trigger_interval_prev_selected(plan, c_hi, &latest_selected)) {
      return;
    }
    uint64_t low_abs = abs_write > DUMP_BANK_WORDS
                           ? abs_write - DUMP_BANK_WORDS + output_words
                           : 0u;
    uint32_t low_chunk =
        (uint32_t)((low_abs + output_words - 1u) / output_words);
    if (latest_selected < low_chunk) {
      s_stream_dropped_chunks +=
          trigger_interval_selected_count(plan, s_emit_chunk, low_chunk);
      s_emit_chunk = trigger_interval_next_selected(plan, low_chunk);
      s_source_chunk_index = s_emit_chunk;
      s_s31_diag_emit_chunk = s_emit_chunk;
      return;
    }
    if (latest_selected < s_emit_chunk) {
      s_source_chunk_index = s_emit_chunk;
      s_s31_diag_emit_chunk = s_emit_chunk;
      return;
    }
    uint32_t older_selected =
        trigger_interval_selected_count(plan, s_emit_chunk, latest_selected);
    if (older_selected != 0u) {
      s_stream_dropped_chunks += older_selected;
    }
    stream_frame_t *slots[1];
    if (!stream_ring_reserve_slots(1u, slots)) {
      ++s_stream_dropped_chunks;
      return;
    }
    const uint32_t *words = s31_exact_chunk_source(config, latest_selected);
    fill_and_push_chunk(config, words, latest_selected, sample_rate_hz, rx_gain,
                        agc_state, slots[0]);
    s31_chunk_copy_done(latest_selected);
    stream_ring_commit_reserved(1u);
    s_emit_chunk = trigger_interval_next_selected(plan, latest_selected + 1u);
    s_source_chunk_index = s_emit_chunk;
    s_s31_diag_emit_chunk = s_emit_chunk;
    return;
  }

  uint32_t n = c_hi - s_emit_chunk;
  if (n > S31_RING_MAX_CHUNKS_PER_POLL) {
    n = S31_RING_MAX_CHUNKS_PER_POLL;
  }

  int32_t dc_i = s_power_trigger_dc_i_q16 >> 16;
  int32_t dc_q = s_power_trigger_dc_q_q16 >> 16;
  bool dc_valid = s_power_trigger_dc_valid;
  int32_t sum_i = 0, sum_q = 0;
  uint32_t sum_n = 0u;
  for (uint32_t k = 0u; k < n; ++k) {
    uint32_t c = s_emit_chunk + k;
    if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
      s_chunk_mark[k] = trigger_interval_select(plan, c) ? CHUNK_STREAM_TRIGGER
                                                         : CHUNK_STREAM_SKIP;
    } else {
      const uint32_t *words =
          s31_chunk_source(config, c, abs_write, true);
      s_chunk_mark[k] = chunk_triggers(plan, words, &sum_i, &sum_q, &sum_n,
                                       dc_i, dc_q, dc_valid)
                            ? CHUNK_STREAM_TRIGGER
                            : CHUNK_STREAM_SKIP;
    }
  }
  if (plan->mode != IQ_TRIGGER_MODE_INTERVAL) {
    expand_pre_post(s_chunk_mark, n, plan->pre_chunks, plan->post_chunks);
    if (plan->mode == IQ_TRIGGER_MODE_POWER && sum_n != 0u) {
      int32_t mean_i = (sum_i / (int32_t)sum_n) * 65536;
      int32_t mean_q = (sum_q / (int32_t)sum_n) * 65536;
      if (!s_power_trigger_dc_valid) {
        s_power_trigger_dc_i_q16 = mean_i;
        s_power_trigger_dc_q_q16 = mean_q;
        s_power_trigger_dc_valid = true;
      } else {
        s_power_trigger_dc_i_q16 +=
            (mean_i - s_power_trigger_dc_i_q16) >> plan->dc_shift;
        s_power_trigger_dc_q_q16 +=
            (mean_q - s_power_trigger_dc_q_q16) >> plan->dc_shift;
      }
    }
  }

  uint32_t selected = 0u;
  for (uint32_t k = 0u; k < n; ++k) {
    if (s_chunk_mark[k] != CHUNK_STREAM_SKIP) {
      ++selected;
    }
  }
  uint32_t available = stream_ring_available_slots();
  uint32_t reserve = selected < available ? selected : available;
  if (reserve < selected) {
    s_stream_dropped_chunks += selected - reserve;
  }
  stream_frame_t *slots[IQ_STREAM_RING_CHUNKS];
  if (reserve != 0u && !stream_ring_reserve_slots(reserve, slots)) {
    s_stream_dropped_chunks += reserve;
    reserve = 0u;
  }
  uint32_t filled = 0u;
  for (uint32_t k = 0u; k < n && filled < reserve; ++k) {
    if (s_chunk_mark[k] == CHUNK_STREAM_SKIP) {
      continue;
    }
    uint32_t c = s_emit_chunk + k;
    const uint32_t *words = s31_chunk_source(config, c, abs_write, true);
    uint32_t source_chunk = s_s31_copy_source_chunk;
    fill_and_push_chunk(config, words, source_chunk, sample_rate_hz, rx_gain,
                        agc_state, slots[filled]);
    s31_chunk_copy_done(c);
    ++filled;
  }
  if (filled != 0u) {
    stream_ring_commit_reserved(filled);
  }
  if (reserve > filled) {
    stream_ring_release_reserved(reserve - filled);
  }
  s_emit_chunk += n;
  s_source_chunk_index = s_emit_chunk;
  s_s31_diag_emit_chunk = s_emit_chunk;
}

static void s31_wait_until_next_ring_window(const capture_config_t *config,
                                            const trigger_plan_t *plan) {
#if CONFIG_ESP_SDR_TRANSPORT_USB
  if (s31_modem_diag_probe_enabled(config)) {
    taskYIELD();
    return;
  }
#endif
  if (s31_pulse_tcm_before_copy(config) &&
      plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
    if (s31_pipelined_stage_enabled(config) &&
        s31_software_decimation(config) == 1u && !stream_output_int8()) {
      /* The staged transaction already waited for the writer to acquire its
       * complete next block. s_s31_diag_abs_write predates that transaction,
       * so using it below would sleep for the same block a second time. */
      taskYIELD();
      return;
    }
    uint64_t abs_write = s_s31_diag_abs_write;
    uint32_t next_chunk = s_emit_chunk;
    uint32_t output_words = s31_output_chunk_words(config);
    int32_t offset = s31_live_copy_offset_chunks(config);
    int64_t copy_chunk = (int64_t)next_chunk - offset;
    if (copy_chunk < 0) {
      copy_chunk = 0;
    }
    uint64_t ready_abs =
        ((uint64_t)copy_chunk + 1u) * output_words +
        S31_LIVE_COPY_GUARD_WORDS;
    if (ready_abs <= abs_write) {
      taskYIELD();
      return;
    }
    uint64_t remaining_words64 = ready_abs - abs_write;
    uint32_t remaining_words = remaining_words64 > UINT32_MAX
                                   ? UINT32_MAX
                                   : (uint32_t)remaining_words64;
    uint32_t sample_cycles = adc_dump_write_sample_cycles(config);
    uint32_t remaining_us =
        sample_cycles != 0u
            ? (remaining_words * sample_cycles) /
                  (ADC_DUMP_CLOCK_HZ / 1000000u)
            : 0u;
    if (remaining_us > S31_POLL_WAKE_MARGIN_US + 1000u) {
      vTaskDelay(
          pdMS_TO_TICKS((remaining_us - S31_POLL_WAKE_MARGIN_US) / 1000u));
    } else if (remaining_us > S31_POLL_CLOSE_US) {
      esp_rom_delay_us(remaining_us - S31_POLL_CLOSE_US);
    } else {
      esp_rom_delay_us(S31_POLL_CLOSE_US);
    }
    return;
  }
  uint64_t abs_write = s_s31_diag_abs_write;
  uint32_t next_chunk = s_emit_chunk;
  uint32_t output_words = s31_output_chunk_words(config);
  if (plan->mode == IQ_TRIGGER_MODE_INTERVAL) {
    next_chunk = trigger_interval_next_selected(plan, next_chunk);
  }
  uint64_t ready_abs = ((uint64_t)next_chunk + 1u) * output_words +
                       s31_ring_guard_words(plan);
  if (ready_abs <= abs_write) {
    taskYIELD();
    return;
  }
  uint64_t remaining_words64 = ready_abs - abs_write;
  uint32_t remaining_words = remaining_words64 > UINT32_MAX
                                 ? UINT32_MAX
                                 : (uint32_t)remaining_words64;
  uint32_t sample_cycles = adc_dump_write_sample_cycles(config);
  uint32_t remaining_us =
      sample_cycles != 0u
          ? (remaining_words * sample_cycles) /
                (ADC_DUMP_CLOCK_HZ / 1000000u)
          : 0u;
  if (remaining_us > S31_POLL_WAKE_MARGIN_US + 1000u) {
    vTaskDelay(pdMS_TO_TICKS((remaining_us - S31_POLL_WAKE_MARGIN_US) / 1000u));
  } else if (remaining_us > S31_POLL_CLOSE_US) {
    esp_rom_delay_us(remaining_us - S31_POLL_CLOSE_US);
    taskYIELD();
  } else {
    esp_rom_delay_us(S31_POLL_CLOSE_US);
    taskYIELD();
  }
}
#endif

static void IRAM_ATTR producer_task(void *arg) {
  (void)arg;
  volatile uint32_t stack_probe = 0u;
  ESP_LOGI("iq_capture", "producer stack %p (%s)", &stack_probe,
           esp_ptr_external_ram((const void *)&stack_probe) ? "PSRAM" : "TCM");
  ESP_LOGI("iq_capture", "producer running on core %d", xPortGetCoreID());
#if CONFIG_IDF_TARGET_ESP32S31
  /* Trigger FreeRTOS's lazy PIE-context setup while scheduling and interrupts
   * are still available. The staged transaction can then use PIE safely with
   * both cores quiesced. */
  s31_pie_memcpy_aligned((void *)(uintptr_t)S31_STAGE_BASE,
                         (const void *)(uintptr_t)S31_STAGE_BASE, 64u);
#endif
  while (true) {
#if CONFIG_IDF_TARGET_ESP32S31
    if (s_s31_tx_worker_pending) {
      const capture_config_t tx_config = s_s31_tx_worker_config;
      TaskHandle_t requester = s_s31_tx_worker_requester;
      s_s31_tx_worker_mode = true;
      s31_tx_replay_enable(&tx_config);
      s_s31_tx_worker_mode = false;
      s_s31_tx_worker_pending = false;
      if (requester != NULL) {
        xTaskNotifyGive(requester);
      }
      continue;
    }
#endif
    if (!iq_network_stream_armed()) {
      vTaskDelay(pdMS_TO_TICKS(IQ_TASK_IDLE_DELAY_MS));
      continue;
    }
    if (config_apply_in_progress()) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
#if CONFIG_IDF_TARGET_ESP32S31
    capture_config_t config;
    trigger_plan_t plan;
    taskENTER_CRITICAL(&s_config_mux);
    config = s_config;
    plan = s_trigger_plan;
    taskEXIT_CRITICAL(&s_config_mux);
    if (!capture_engine_running(&config)) {
      engine_enable();
      stream_state_reset();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    if (s31_tx_replay_selected(&config)) {
      vTaskDelay(pdMS_TO_TICKS(IQ_TASK_IDLE_DELAY_MS));
      continue;
    }
    (void)ulTaskNotifyTake(pdTRUE, 0u);
    s_producer_active = true;
    process_s31_dump_ring(&config, &plan);
    s_producer_active = false;
    if (!s31_continuous_real_if_selected(&config) &&
        !s31_parlio_native_packed_iq_selected(&config)) {
      s31_wait_until_next_ring_window(&config, &plan);
    }
#endif
  }
}

/* ---- config apply ---- */

static capture_config_t active_config_snapshot(void) {
  capture_config_t config;
  taskENTER_CRITICAL(&s_config_mux);
  config = s_config;
  taskEXIT_CRITICAL(&s_config_mux);
  return config;
}

static capture_config_t sanitize_capture_config(capture_config_t config) {
#if CONFIG_IDF_TARGET_ESP32S31
  if (config.loopback.loopback != 0u && config.tx.tx_tone_enable != 0u) {
    config.radio.rf_freq_hz = RF_FREQ_DEFAULT_HZ;
  }
  if (config.radio.rf_freq_hz < RF_FREQ_MIN_HZ)
    config.radio.rf_freq_hz = RF_FREQ_MIN_HZ;
  if (config.radio.rf_freq_hz > RF_FREQ_MAX_HZ)
    config.radio.rf_freq_hz = RF_FREQ_MAX_HZ;
  /* phy_set_freq() accepts a signed kHz remainder; make HTTP readback reflect
   * the actual hardware resolution rather than preserving unattainable Hz. */
  config.radio.rf_freq_hz =
      ((config.radio.rf_freq_hz + 500u) / 1000u) * 1000u;
  config.bandwidth.bw_mhz = 20u;
  config.bandwidth.second_chan = SECOND_CHAN_NONE;
#endif
  if (config.gain.tx_gain > TX_GAIN_MAX)
    config.gain.tx_gain = TX_GAIN_MAX;
  config.dc_offset.automatic = config.dc_offset.automatic != 0u;
  if (config.rx_filter.rx_filter_override == 0u &&
      config.rx_filter.filter_bw_mhz != RX_FILTER_BW_OPEN) {
    if (config.rx_filter.filter_bw_mhz < RX_FILTER_BW_MIN_MHZ)
      config.rx_filter.filter_bw_mhz = RX_FILTER_BW_MIN_MHZ;
    if (config.rx_filter.filter_bw_mhz > RX_FILTER_BW_MAX_MHZ)
      config.rx_filter.filter_bw_mhz = RX_FILTER_BW_MAX_MHZ;
  }
  if (config.rx_filter.rx_filter_dcap > 63u)
    config.rx_filter.rx_filter_dcap = 63u;
  if (config.rx_filter.rx_filter_override != S31_GPIO_DIAG_OVERRIDE &&
      config.rx_filter.rx_filter_override != S31_PARLIO_NATIVE_IQ_OVERRIDE &&
      config.rx_filter.rx_filter_override != S31_PARLIO_HOST_IQ_OVERRIDE &&
      config.rx_filter.rx_filter_override != S31_HP_TCM_PROBE_OVERRIDE &&
      config.rx_filter.rx_filter_mode > 32u)
    config.rx_filter.rx_filter_mode = 32u;
  return config;
}

static void queue_config_apply(const capture_config_t *config) {
  capture_config_t sanitized = sanitize_capture_config(*config);
  trigger_plan_t plan = build_trigger_plan(&sanitized);
  taskENTER_CRITICAL(&s_config_mux);
  s_pending_config = sanitized;
  s_pending_trigger_plan = plan;
  s_config_apply_pending = true;
  taskEXIT_CRITICAL(&s_config_mux);
}

static bool consume_pending_config_for_apply(void) {
  bool pending;
  int64_t now_us = esp_timer_get_time();
  taskENTER_CRITICAL(&s_config_mux);
  pending = s_config_apply_enabled && s_config_apply_pending &&
            !s_config_apply_in_progress;
  if (pending) {
    s_config = s_pending_config;
    s_trigger_plan = s_pending_trigger_plan;
    s_config_apply_pending = false;
    s_config_apply_in_progress = true;
    s_config_apply_started_us = now_us;
  }
  taskEXIT_CRITICAL(&s_config_mux);
  return pending;
}

static void finish_config_apply(void) {
  taskENTER_CRITICAL(&s_config_mux);
  s_config_apply_in_progress = false;
  s_config_apply_started_us = 0;
  taskEXIT_CRITICAL(&s_config_mux);
}

static bool config_apply_in_progress(void) {
  bool in_progress;
  taskENTER_CRITICAL(&s_config_mux);
  in_progress = s_config_apply_pending || s_config_apply_in_progress;
  taskEXIT_CRITICAL(&s_config_mux);
  return in_progress;
}

static void recover_stale_config_apply(void) {
  int64_t now_us = esp_timer_get_time();
  taskENTER_CRITICAL(&s_config_mux);
  if (s_config_apply_in_progress && s_config_apply_started_us != 0 &&
      now_us - s_config_apply_started_us > 500000) {
    s_config_apply_in_progress = false;
    s_config_apply_started_us = 0;
  }
  taskEXIT_CRITICAL(&s_config_mux);
}

enum {
  CONFIG_STAGE_PACKET_ACCEPTED = 0u,
  CONFIG_STAGE_AFTER_STREAM_ARM = 8u,
  CONFIG_STAGE_AFTER_QUEUE = 9u,
  CONFIG_STAGE_APPLY_START = 1u,
  CONFIG_STAGE_AFTER_WIFI = 2u,
  CONFIG_STAGE_BEFORE_MODEM_APPLY = 3u,
  CONFIG_STAGE_AFTER_MODEM_APPLY = 4u,
  CONFIG_STAGE_BEFORE_RESTART = 6u,
  CONFIG_STAGE_AFTER_RESTART = 7u,
  CONFIG_STAGE_RESTART_ENTRY = 20u,
  CONFIG_STAGE_AFTER_ENGINE_ENABLE = 21u,
  CONFIG_STAGE_BEFORE_GAIN_TABLE = 22u,
  CONFIG_STAGE_AFTER_GAIN_TABLE = 23u,
  CONFIG_STAGE_AFTER_LIVE_GAIN = 24u,
  CONFIG_STAGE_AFTER_STREAM_STATE = 25u,
};

static void push_config_stage_report(const capture_config_t *config,
                                     uint32_t stage) {
  config_report_t report = {
      .magic = {'C', 'F', 'G', '1'},
      .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
      .stage = stage,
      .rx_gain = config->gain.rx_gain,
      .loopback = config->loopback.loopback,
      .tx_tone_enable = config->tx.tx_tone_enable,
      .adc_source_sel = config->iq_engine.adc_source_sel,
      .expert_gain_word0 = config->gain.expert_gain_word0,
      .expert_gain_word1 = config->gain.expert_gain_word1,
      .ctrl = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL),
      .mode = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE),
      .trigger_interval_chunks = config->trigger.trigger_config[0],
      .trigger_duration_chunks = config->trigger.trigger_config[2],
#if CONFIG_IDF_TARGET_ESP32S31
      .hp_tcm_dump_ctrl = reg32_read_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG),
#else
      .hp_tcm_dump_ctrl = 0u,
#endif
  };
  (void)stream_ring_push_config_report(&report);
}

static bool capture_engine_running(const capture_config_t *config) {
#if CONFIG_IDF_TARGET_ESP32S31
  if (s31_tx_replay_selected(config)) {
    return s_s31_tx_replay_burst_complete;
  }
  uint32_t ctrl = reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL);
  if ((ctrl & MODEM_WIFI_DUMP_CTRL_ENABLE_BIT) == 0u) {
    return false;
  }
  if ((ctrl & MODEM_WIFI_DUMP_CTRL_CONTINUOUS_TRIGGER_GATE_BIT) == 0u) {
    return false;
  }
  return true;
#else
  return true;
#endif
}

static bool config_equal_except_gain(const capture_config_t *a,
                                     const capture_config_t *b) {
  return memcmp(&a->stream, &b->stream, sizeof(a->stream)) == 0 &&
         memcmp(&a->radio, &b->radio, sizeof(a->radio)) == 0 &&
         memcmp(&a->bandwidth, &b->bandwidth, sizeof(a->bandwidth)) == 0 &&
         memcmp(&a->loopback, &b->loopback, sizeof(a->loopback)) == 0 &&
         memcmp(&a->tx, &b->tx, sizeof(a->tx)) == 0 &&
         memcmp(&a->iq_engine, &b->iq_engine, sizeof(a->iq_engine)) == 0 &&
         memcmp(&a->trigger, &b->trigger, sizeof(a->trigger)) == 0 &&
         memcmp(&a->rx_filter, &b->rx_filter, sizeof(a->rx_filter)) == 0 &&
         memcmp(&a->wifi_tx, &b->wifi_tx, sizeof(a->wifi_tx)) == 0 &&
         memcmp(&a->dc_offset, &b->dc_offset, sizeof(a->dc_offset)) == 0;
}

#if CONFIG_IDF_TARGET_ESP32S31
static bool config_equal_except_tx(const capture_config_t *a,
                                   const capture_config_t *b) {
  capture_config_t left = *a;
  capture_config_t right = *b;
  memset(&left.tx, 0, sizeof(left.tx));
  memset(&right.tx, 0, sizeof(right.tx));
  return memcmp(&left, &right, sizeof(left)) == 0;
}
#endif

static bool config_equal_except_trigger(const capture_config_t *a,
                                        const capture_config_t *b) {
  return memcmp(&a->stream, &b->stream, sizeof(a->stream)) == 0 &&
         memcmp(&a->radio, &b->radio, sizeof(a->radio)) == 0 &&
         memcmp(&a->gain, &b->gain, sizeof(a->gain)) == 0 &&
         memcmp(&a->bandwidth, &b->bandwidth, sizeof(a->bandwidth)) == 0 &&
         memcmp(&a->loopback, &b->loopback, sizeof(a->loopback)) == 0 &&
         memcmp(&a->tx, &b->tx, sizeof(a->tx)) == 0 &&
         memcmp(&a->iq_engine, &b->iq_engine, sizeof(a->iq_engine)) == 0 &&
         memcmp(&a->rx_filter, &b->rx_filter, sizeof(a->rx_filter)) == 0 &&
         memcmp(&a->wifi_tx, &b->wifi_tx, sizeof(a->wifi_tx)) == 0 &&
         memcmp(&a->dc_offset, &b->dc_offset, sizeof(a->dc_offset)) == 0;
}

#if CONFIG_IDF_TARGET_ESP32S31
static bool config_equal_except_diag_mode(const capture_config_t *a,
                                          const capture_config_t *b) {
  capture_config_t left = *a;
  capture_config_t right = *b;
  left.rx_filter.rx_filter_mode = 0u;
  right.rx_filter.rx_filter_mode = 0u;
  return memcmp(&left, &right, sizeof(left)) == 0;
}
#endif

static void apply_live_gain_only_config(const capture_config_t *config) {
  push_config_stage_report(config, CONFIG_STAGE_APPLY_START);
  wait_stream_pipeline_idle();
  /* The calibrated gain table was populated after engine_enable(). Rewriting
   * gain RAM while the dump engine is active corrupts the RX operating point;
   * a live gain change only needs to select another existing table slot. */
  modem_config_t gain_cfg = modem_config_from_capture(config);
  modem_apply_live_rx_config(&gain_cfg);
  /* The forced-gain operating point changed: re-arm the DCOC servo on the
   * new slot (disarms in AGC/loopback modes). */
  dcoc_arm(&gain_cfg);
  sdr_agc_arm(&gain_cfg);
  push_config_stage_report(config, CONFIG_STAGE_AFTER_LIVE_GAIN);
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
  wait_stream_pipeline_idle();
  stream_state_reset();
  push_config_stage_report(config, CONFIG_STAGE_AFTER_STREAM_STATE);
#endif
  push_config_stage_report(config, CONFIG_STAGE_AFTER_RESTART);
}

static void restart_capture(const capture_config_t *config) {
  push_config_stage_report(config, CONFIG_STAGE_RESTART_ENTRY);
  s_isr_prev_sa = 0u;
  s_isr_wrap_count = 0u;
  engine_enable();
  push_config_stage_report(config, CONFIG_STAGE_AFTER_ENGINE_ENABLE);
#if CONFIG_IDF_TARGET_ESP32S31
  if (s31_tx_replay_selected(config)) {
    stream_state_reset();
    push_config_stage_report(config, CONFIG_STAGE_AFTER_STREAM_STATE);
    return;
  }
#endif
  /* The engine/FE bring-up leaves gain memory stale, so forced slot indices
   * can map to a flat (~2-level) gain. Regenerate the gain ramp
   * before selecting either a manual gain or the initial software-AGC gain.
   * Loopback gains use a separate path. */
  if (config->loopback.loopback == 0u &&
      (config->gain.gain_mode == GAIN_MODE_MANUAL ||
       config->gain.gain_mode == GAIN_MODE_AUTO)) {
    push_config_stage_report(config, CONFIG_STAGE_BEFORE_GAIN_TABLE);
    modem_setup_rx_gain_table();
    push_config_stage_report(config, CONFIG_STAGE_AFTER_GAIN_TABLE);
    if (config->gain.gain_mode == GAIN_MODE_AUTO) {
      modem_prepare_sdr_agc_gain_table();
    } else {
      modem_config_t gain_cfg = modem_config_from_capture(config);
      modem_apply_live_rx_config(&gain_cfg);
      push_config_stage_report(config, CONFIG_STAGE_AFTER_LIVE_GAIN);
    }
  }
  /* Forced-gain modes: (re-)arm the DCOC servo for this operating point (the
   * freshly rebuilt table only carries the coarse boot-time compensation; at
   * high forced gains the leftover LO self-mixing DC clips the ADC). The
   * producer feeds it samples per emitted chunk; it converges within seconds
   * and keeps tracking drift. Arm handles AGC/loopback by disarming. */
  modem_config_t dcoc_cfg = modem_config_from_capture(config);
  dcoc_arm(&dcoc_cfg);
  sdr_agc_arm(&dcoc_cfg);
  wait_stream_pipeline_idle();
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
#endif
  stream_state_reset();
  push_config_stage_report(config, CONFIG_STAGE_AFTER_STREAM_STATE);
}

#if CONFIG_IDF_TARGET_ESP32S31
static void finish_s31_one_shot_config(capture_config_t *config) {
  if (!s31_tx_replay_selected(config) ||
      (((uint32_t)config->tx.tx_tone0_step &
        S31_TX_REPLAY_ONE_SHOT_REQUEST) == 0u) ||
      !s_s31_tx_replay_burst_complete) {
    return;
  }
  /* RF is already cold and all uploaded buffers have been reclaimed. Make
   * the active configuration reflect that finite-burst state immediately;
   * otherwise a later RX-only patch can re-enter replay with no waveform. */
  config->tx.tx_tone_enable = 0u;
  taskENTER_CRITICAL(&s_config_mux);
  s_config.tx.tx_tone_enable = 0u;
  taskEXIT_CRITICAL(&s_config_mux);
}
#endif

static void apply_pending_config(void) {
  static capture_config_t applied;
  static bool have_applied;
  capture_config_t cfg = active_config_snapshot();
#if CONFIG_IDF_TARGET_ESP32S31
  bool force_rearm;
  taskENTER_CRITICAL(&s_config_mux);
  force_rearm = s_s31_tx_replay_force_rearm;
  s_s31_tx_replay_force_rearm = false;
  taskEXIT_CRITICAL(&s_config_mux);
#else
  const bool force_rearm = false;
#endif

  if (!force_rearm && have_applied &&
      memcmp(&applied, &cfg, sizeof(cfg)) == 0 &&
      capture_engine_running(&cfg)) {
    return;
  }
#if CONFIG_IDF_TARGET_ESP32S31
  if (force_rearm && have_applied &&
      (memcmp(&applied, &cfg, sizeof(cfg)) == 0 ||
       config_equal_except_tx(&applied, &cfg)) &&
      s31_tx_replay_selected(&cfg)) {
    /* The RF and modem configuration is already current. Re-run only the
     * replay engine so both the first RX->TX transition and consecutive Soapy
     * bursts avoid a redundant full PHY calibration/restart. */
    push_config_stage_report(&cfg, CONFIG_STAGE_APPLY_START);
    wait_stream_pipeline_idle();
    engine_disable();
    modem_config_t replay_modem_config = modem_config_from_capture(&cfg);
    modem_prepare_tx_replay(&replay_modem_config);
    restart_capture(&cfg);
    push_config_stage_report(&cfg, CONFIG_STAGE_AFTER_RESTART);
    finish_s31_one_shot_config(&cfg);
    applied = cfg;
    return;
  }
  if (!force_rearm && have_applied &&
      (applied.rx_filter.rx_filter_override == S31_GPIO_DIAG_OVERRIDE ||
       applied.rx_filter.rx_filter_override ==
           S31_PARLIO_NATIVE_IQ_OVERRIDE) &&
      applied.rx_filter.rx_filter_override ==
          cfg.rx_filter.rx_filter_override &&
      config_equal_except_diag_mode(&applied, &cfg) &&
      capture_engine_running(&cfg)) {
    wait_stream_pipeline_idle();
    s31_gpio_diag_stop();
    s31_configure_modem_diag(&cfg);
    s31_gpio_diag_prepare(&cfg);
    stream_state_reset();
    applied = cfg;
    return;
  }
  if (!force_rearm && have_applied &&
      config_equal_except_trigger(&applied, &cfg) &&
      capture_engine_running(&cfg)) {
    stream_state_reset();
    applied = cfg;
    return;
  }
  if (!force_rearm && have_applied &&
      config_equal_except_gain(&applied, &cfg) &&
      applied.gain.gain_mode == cfg.gain.gain_mode &&
      capture_engine_running(&cfg)) {
    apply_live_gain_only_config(&cfg);
    applied = cfg;
    have_applied = true;
    return;
  }
#endif
  push_config_stage_report(&cfg, CONFIG_STAGE_APPLY_START);
  wait_stream_pipeline_idle();
  /* Fractional/out-of-band RF programming touches the same closed modem path
   * as continuous IQ capture. Quiesce the dump engine before retuning; the
   * restart below re-arms it with the new configuration. */
  engine_disable();
  wifi_tx_rx_config_t wifi_config = wifi_tx_rx_config_from_capture(&cfg);
  wifi_tx_rx_apply_config(&wifi_config);
  if (wifi_config.stream_packets != 0u || wifi_config.dummy_tx_enable != 0u) {
    wifi_tx_rx_init(stream_ring_push_wifi);
  }
  push_config_stage_report(&cfg, CONFIG_STAGE_AFTER_WIFI);

  modem_prepare_direct_phy_access();
  modem_config_t modem_config = modem_config_from_capture(&cfg);
  push_config_stage_report(&cfg, CONFIG_STAGE_BEFORE_MODEM_APPLY);
#if CONFIG_IDF_TARGET_ESP32S31
  if (s31_tx_replay_selected(&cfg)) {
    /* Calibrate/tune with TX physically disabled. The replay engine enables
     * the calibrated path only after its final deadline check. */
    modem_config_t disabled_tx_config = modem_config;
    disabled_tx_config.tx_tone_enable = 0u;
    modem_apply_rx_config(&disabled_tx_config);
    modem_prepare_tx_replay(&modem_config);
  } else
#endif
  {
    modem_apply_rx_config(&modem_config);
  }
  push_config_stage_report(&cfg, CONFIG_STAGE_AFTER_MODEM_APPLY);
  push_config_stage_report(&cfg, CONFIG_STAGE_BEFORE_RESTART);
  restart_capture(&cfg);
  push_config_stage_report(&cfg, CONFIG_STAGE_AFTER_RESTART);
#if CONFIG_IDF_TARGET_ESP32S31
  finish_s31_one_shot_config(&cfg);
#endif

  applied = cfg;
  have_applied = true;
}

static void service_pending_config(void) {
  recover_stale_config_apply();
  if (consume_pending_config_for_apply()) {
    apply_pending_config();
    finish_config_apply();
  }
}

/* ---- HTTP configuration callbacks and UDP stream ---- */

static void network_get_config(capture_config_t *config) {
  *config = active_config_snapshot();
}

static void network_apply_config(const capture_config_t *config) {
  capture_config_t sanitized = sanitize_capture_config(*config);
  wifi_tx_rx_set_stream_armed(true);
  queue_config_apply(&sanitized);
}

static bool network_set_tx_waveform(const uint32_t *words,
                                    uint32_t word_count) {
  if (words == NULL || word_count == 0u ||
      word_count > TX_BATCH_WORDS_MAX) {
    return false;
  }
  taskENTER_CRITICAL(&s_config_mux);
  bool busy = s_config_apply_pending || s_config_apply_in_progress ||
              s_config.tx.tx_tone_enable != 0u;
  taskEXIT_CRITICAL(&s_config_mux);
  if (busy) {
    return false;
  }
  uint32_t *new_upload = heap_caps_malloc(
      word_count * sizeof(*new_upload),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (new_upload == NULL) {
    new_upload = heap_caps_malloc(
        word_count * sizeof(*new_upload),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (new_upload == NULL) {
    return false;
  }
  memcpy(new_upload, words, word_count * sizeof(*new_upload));
  taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
  uint32_t *old_upload = s_s31_tx_replay_upload;
  s_s31_tx_replay_upload = new_upload;
  s_s31_tx_replay_upload_words = word_count;
  taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
  s31_txdc_release_upload(old_upload);
  return true;
}

static iq_tx_waveform_result_t network_take_tx_waveform(
    uint32_t *words, uint32_t word_count, uint16_t commit_flags,
    uint64_t start_time_ns) {
  if (words == NULL || word_count == 0u ||
      word_count > TX_BATCH_WORDS_MAX) {
    return IQ_TX_WAVEFORM_REJECTED;
  }
  bool autostart = (commit_flags & IQ_TX_UDP_FLAG_AUTOSTART) != 0u;
  bool more = (commit_flags & IQ_TX_UDP_FLAG_MORE) != 0u;
  bool require_continuation =
      (commit_flags & IQ_TX_UDP_FLAG_CONTINUE) != 0u;
  uint32_t rate_code =
      (commit_flags & IQ_TX_UDP_RATE_CODE_M) >> IQ_TX_UDP_RATE_CODE_S;
  bool rate_valid = rate_code <= 3u ||
                    (rate_code >= 7u && rate_code <= 15u);
  bool digital_txdc = autostart &&
                      (rate_code >= 9u && rate_code <= 15u);
  if ((autostart && !rate_valid) ||
      (more && !digital_txdc) ||
      (require_continuation && !digital_txdc) ||
      (!autostart && ((commit_flags & IQ_TX_UDP_RATE_CODE_M) != 0u ||
                      start_time_ns != 0u))) {
    return IQ_TX_WAVEFORM_REJECTED;
  }
  const bool continuation =
      digital_txdc &&
      __atomic_load_n(&s_s31_txdc_stream_active, __ATOMIC_ACQUIRE);
  if (require_continuation && !continuation) {
    /* The host explicitly identified this as a later batch. If the realtime
     * chain ended while its upload was in flight, rejecting ownership here
     * prevents stale IQ from silently starting a new RF application. */
    ESP_LOGW("iq_capture", "TX continuation rejected: RF chain inactive");
    return IQ_TX_WAVEFORM_REJECTED;
  }
  if (continuation &&
      (start_time_ns != 0u ||
       rate_code != __atomic_load_n(&s_s31_txdc_stream_rate_code,
                                    __ATOMIC_ACQUIRE))) {
    ESP_LOGW("iq_capture",
             "TX continuation rejected: timing/rate mismatch (%" PRIu32
             " != %" PRIu32 ")",
             rate_code, s_s31_txdc_stream_rate_code);
    return IQ_TX_WAVEFORM_REJECTED;
  }
  if (continuation) {
    const bool usb_staged_rate =rate_code == 14u || rate_code == 15u;
    const uint32_t queue_limit =
        usb_staged_rate ? 3u
        : (commit_flags & IQ_TX_UDP_FLAG_PACKED16) != 0u
            ? S31_TXDC_CONTINUATION_QUEUE_DEPTH
        : (commit_flags & IQ_TX_UDP_FLAG_PACKED20) != 0u
            ? S31_TXDC_ETHERNET_PACKED20_QUEUE_DEPTH
            : 1u;
    /* Native USB reaches this callback only after its bulk upload completed.
     * Waiting synchronously for the next FIFO slot makes upload time overlap
     * the current RF batch and was the proven 5+ MSa/s producer contract.
     * Ethernet is different: this callback runs in the direct EMAC receive
     * path, where waiting prevents RX descriptor recycling and collapses the
     * transport. Its ACK protocol reports committed=0 and retries the
     * idempotent final datagram instead. */
    if (usb_staged_rate) {
    const int64_t queue_deadline_us = esp_timer_get_time() + 300000;
    while (s31_txdc_continuation_count() >= queue_limit &&
           __atomic_load_n(&s_s31_txdc_stream_active, __ATOMIC_ACQUIRE) &&
           esp_timer_get_time() < queue_deadline_us) {
      uint32_t *reclaim = __atomic_exchange_n(
          &s_s31_txdc_reclaim_upload, NULL, __ATOMIC_ACQ_REL);
      uint32_t *reclaim2 = __atomic_exchange_n(
          &s_s31_txdc_reclaim_upload2, NULL, __ATOMIC_ACQ_REL);
      s31_txdc_release_upload(reclaim);
      s31_txdc_release_upload(reclaim2);
      vTaskDelay(1);
    }
    }
    uint32_t *reclaim =
        __atomic_exchange_n(&s_s31_txdc_reclaim_upload, NULL, __ATOMIC_ACQ_REL);
    uint32_t *reclaim2 = __atomic_exchange_n(&s_s31_txdc_reclaim_upload2, NULL,
                                             __ATOMIC_ACQ_REL);
    s31_txdc_release_upload(reclaim);
    s31_txdc_release_upload(reclaim2);
    if (s31_txdc_continuation_count() >= queue_limit ||
        !__atomic_load_n(&s_s31_txdc_stream_active, __ATOMIC_ACQUIRE)) {
      return __atomic_load_n(&s_s31_txdc_stream_active,
                             __ATOMIC_ACQUIRE)
                 ? IQ_TX_WAVEFORM_RETRY
                 : IQ_TX_WAVEFORM_REJECTED;
    }
  }
  capture_config_t replay_config;
  taskENTER_CRITICAL(&s_config_mux);
  replay_config = s_config;
  bool completed_one_shot =
      replay_config.tx.tx_tone_enable == S31_TX_REPLAY_MODE &&
      ((uint32_t)replay_config.tx.tx_tone0_step &
       S31_TX_REPLAY_ONE_SHOT_REQUEST) != 0u &&
      s_s31_tx_replay_burst_complete;
  bool busy = !continuation &&
              (s_config_apply_pending || s_config_apply_in_progress ||
               (replay_config.tx.tx_tone_enable != 0u &&
                !completed_one_shot));
  taskEXIT_CRITICAL(&s_config_mux);
  if (busy) {
    ESP_LOGW("iq_capture", "TX initial batch rejected: replay busy");
    return IQ_TX_WAVEFORM_RETRY;
  }
  if (autostart && start_time_ns != 0u &&
      (uint64_t)esp_timer_get_time() * 1000u >= start_time_ns) {
    /* The commit is successful, but its RF deadline expired while the host
     * was staging the waveform. Take ownership, suppress configuration/RF
     * work entirely, and publish an unambiguous completed request. */
    s31_txdc_release_upload(words);
    s31_tx_replay_publish_missed(word_count, rate_code, start_time_ns);
    return IQ_TX_WAVEFORM_ADOPTED;
  }

  /* iq_network reserves the maximum UDP batch so it can accept packets in
   * any order.  Do not retain that entire 4 MiB reservation for a short SDR
   * burst: subsequent PARLIO RX allocations would then be displaced into a
   * PSRAM region where sustained GMAC + GDMA traffic is unreliable on S31.
   * heap_caps_realloc() may shrink in place or move the allocation; either is
   * safe because returning true transfers ownership away from iq_network. */
  uint32_t *upload = words;
  /* USB and the current UDP arm contract allocate an exact-size autostart
   * buffer.  Reallocating that PSRAM block on the EMAC RX task adds about
   * 47 ms to every final datagram and forces an otherwise lossless host to
   * retry the last ACK window.  Keep shrinking legacy/manual maximum-size
   * uploads, but transfer exact streaming allocations without heap work. */
  if (!autostart && word_count < TX_BATCH_WORDS_MAX) {
    uint32_t *resized = heap_caps_realloc(
        words, word_count * sizeof(*words),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (resized == NULL) {
      resized = heap_caps_realloc(words, word_count * sizeof(*words),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (resized != NULL) {
      upload = resized;
    }
  }
  if (continuation) {
    uint32_t *reclaim = __atomic_exchange_n(
        &s_s31_txdc_reclaim_upload, NULL, __ATOMIC_ACQ_REL);
    uint32_t *reclaim2 = __atomic_exchange_n(
        &s_s31_txdc_reclaim_upload2, NULL, __ATOMIC_ACQ_REL);
    s31_txdc_release_upload(reclaim);
    s31_txdc_release_upload(reclaim2);
    if (!s31_txdc_continuation_push(upload, word_count, commit_flags)) {
      ESP_LOGW("iq_capture", "TX continuation rejected: FIFO push raced");
      return __atomic_load_n(&s_s31_txdc_stream_active,
                             __ATOMIC_ACQUIRE)
                 ? IQ_TX_WAVEFORM_RETRY
                 : IQ_TX_WAVEFORM_REJECTED;
    }
    ++s_s31_txdc_commit_count;
    s_s31_txdc_flags_trace =
        (s_s31_txdc_flags_trace << 8u) | (commit_flags >> 8u);
    return IQ_TX_WAVEFORM_ADOPTED;
  }
  if (digital_txdc) {
    s31_txdc_continuation_drain();
  }
  taskENTER_CRITICAL(&s_s31_tx_replay_upload_mux);
  uint32_t *old_upload = s_s31_tx_replay_upload;
  s_s31_tx_replay_upload = upload;
  s_s31_tx_replay_upload_words = word_count;
  s_s31_tx_replay_start_time_ns = start_time_ns;
  s_s31_tx_replay_commit_flags = commit_flags;
  if (digital_txdc) {
    s_s31_txdc_stream_rate_code = rate_code;
    s_s31_txdc_stream_active = true;
    s_s31_txdc_commit_count = 1u;
    s_s31_txdc_flags_trace = commit_flags >> 8u;
  }
  taskEXIT_CRITICAL(&s_s31_tx_replay_upload_mux);
  s31_txdc_release_upload(old_upload);
  if (autostart) {
    replay_config.tx.tx_tone_enable = S31_TX_REPLAY_MODE;
    replay_config.tx.tx_tone0_step =
        (int32_t)((rate_code << 4u) | 3u |
                  S31_TX_REPLAY_ONE_SHOT_REQUEST);
    taskENTER_CRITICAL(&s_config_mux);
    s_s31_tx_replay_force_rearm = true;
    taskEXIT_CRITICAL(&s_config_mux);
    queue_config_apply(&replay_config);
  }
  return IQ_TX_WAVEFORM_ADOPTED;
}

static void network_reclaim_tx_waveforms(void) {
  uint32_t *reclaim = __atomic_exchange_n(
      &s_s31_txdc_reclaim_upload, NULL, __ATOMIC_ACQ_REL);
  uint32_t *reclaim2 = __atomic_exchange_n(
      &s_s31_txdc_reclaim_upload2, NULL, __ATOMIC_ACQ_REL);
  s31_txdc_release_upload(reclaim);
  s31_txdc_release_upload(reclaim2);
}

static uint32_t network_dropped_chunks(void) { return s_stream_dropped_chunks; }

static uint32_t network_source_chunk(void) { return s_source_chunk_index; }

static uint32_t network_adc_dump_cfg(void) {
  return reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_CFG);
}

static uint32_t network_adc_dump_mode(void) {
  return reg32p_read(&MODEM_WIFI_DUMP.ADC_DUMP_MODE);
}

static uint32_t network_modem_diag_fix_sel(void) {
  return reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_FIX_SEL_REG);
}

static uint32_t network_modem_diag_exchange(void) {
  return reg32_read_addr(MODEM_WIDGETS_MODEM_DIAG_EXCHANGE_REG);
}

static uint32_t network_bb_diag(void) {
  return reg32p_read(&MODEM_WIFI_BB.BB_DIAG0);
}

static uint32_t network_dcoc_diag(void) { return dcoc_diag(); }
static bool network_dcoc_active(void) { return dcoc_active(); }

static void network_tx_replay_diag(iq_tx_replay_diag_t *diag) {
  *diag = s_s31_tx_replay_diag;
}

static void network_set_capture_armed(bool armed) {
  s_network_capture_armed = armed;
  if (armed) {
    capture_config_t config = active_config_snapshot();
    /* Capture is stopped when UDP is stopped.  Serialize a fresh engine start
     * in the stream task; UDP arming is delayed long enough for this restart
     * and its configuration reports to finish before the producer runs. */
    queue_config_apply(&config);
  } else {
#if CONFIG_IDF_TARGET_ESP32S31
    /* The persistent PARLIO transaction otherwise keeps overwriting its DMA
     * ring while no host owns the stream. A later START then observes roughly
     * the entire 100 ms arm delay as a synthetic source gap. Stop at ownership
     * release; s31_process_parlio_diag() restarts the retained unit on demand. */
    wait_stream_pipeline_idle();
    s31_gpio_diag_stop();
#endif
  }
}

static bool network_config_applying(void) {
  bool applying;
  taskENTER_CRITICAL(&s_config_mux);
  applying = s_config_apply_pending || s_config_apply_in_progress;
  taskEXIT_CRITICAL(&s_config_mux);
  return applying;
}

static void stream_next_frame(void) {
  if (config_apply_in_progress()) {
    vTaskDelay(pdMS_TO_TICKS(1));
    return;
  }
  stream_frame_t *frame = stream_ring_peek();
  if (frame == NULL) {
    /* Wake immediately when the producer commits instead of a fixed sleep:
     * the USB transport's drain rate barely exceeds the full-rate arrival
     * rate, so any fixed sleep here turns into source drops. */
    (void)stream_ring_wait_frames(1u);
    frame = stream_ring_peek();
    if (frame == NULL) {
      return;
    }
  }
  s_stream_write_active = true;
  if (config_apply_in_progress()) {
    s_stream_write_active = false;
    vTaskDelay(pdMS_TO_TICKS(1));
    return;
  }
  size_t frame_size = stream_frame_wire_size(frame);
  if (frame_size == 0u) {
    stream_ring_pop();
    s_stream_write_active = false;
    return;
  }
  bool sent = false;
#if CONFIG_ESP_SDR_TRANSPORT_USB
  if (iq_network_stream_owner() == IQ_STREAM_OWNER_USB) {
    sent = iq_usb_send_frame((const uint8_t *)frame, frame_size);
  }
#endif
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  if (iq_network_stream_owner() == IQ_STREAM_OWNER_ETH) {
  if (iq_network_stream_format() == IQ_USB_FORMAT_INT8 &&
      memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ, 4u) == 0) {
    sent = iq_network_send_frame_int8((const uint8_t *)frame, frame_size);
  } else {
    /* Fill one hardware kick up to the default 12-descriptor ring. IQC8 uses
     * two descriptors and the larger, packet-efficient IQR8 uses three.
     * Non-compact or mixed report traffic remains a single-frame operation. */
    enum { MAX_ETH_BATCH_FRAMES = CONFIG_ETH_DMA_TX_BUFFER_NUM / 2u };
    stream_frame_t *batch_frames[MAX_ETH_BATCH_FRAMES];
    const uint8_t *batch_data[MAX_ETH_BATCH_FRAMES];
    size_t batch_sizes[MAX_ETH_BATCH_FRAMES];
    uint32_t batch_count = 1u;
    if ((memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_IQ8, 4u) == 0 ||
         memcmp(frame->iq.magic, STREAM_FRAME_MAGIC_REAL8, 4u) == 0) &&
        MAX_ETH_BATCH_FRAMES > 1u) {
      uint32_t available =
          stream_ring_peek_batch(batch_frames, MAX_ETH_BATCH_FRAMES);
      batch_count = 0u;
      uint32_t descriptors = 0u;
      const uint32_t descriptor_limit =
          CONFIG_ETH_DMA_TX_BUFFER_NUM > 3u
              ? CONFIG_ETH_DMA_TX_BUFFER_NUM - 3u
              : CONFIG_ETH_DMA_TX_BUFFER_NUM;
      for (uint32_t i = 0u; i < available; ++i) {
        size_t size = stream_frame_wire_size(batch_frames[i]);
        uint32_t fragments =
            (size + IQ_UDP_FRAGMENT_PAYLOAD_BYTES - 1u) /
            IQ_UDP_FRAGMENT_PAYLOAD_BYTES;
        if ((size != IQ8_FRAME_WIRE_BYTES &&
             size != REAL8_FRAME_WIRE_BYTES) ||
            (memcmp(batch_frames[i]->iq.magic, STREAM_FRAME_MAGIC_IQ8, 4u) !=
                 0 &&
             memcmp(batch_frames[i]->iq.magic, STREAM_FRAME_MAGIC_REAL8, 4u) !=
                 0) ||
            descriptors + fragments > descriptor_limit) {
          break;
        }
        batch_data[batch_count] = (const uint8_t *)batch_frames[i];
        batch_sizes[batch_count] = size;
        descriptors += fragments;
        ++batch_count;
      }
    }
    if (batch_count == 1u) {
      sent = iq_network_send_frame((const uint8_t *)frame, frame_size);
    } else {
      sent = iq_network_send_frames(batch_data, batch_sizes, batch_count);
    }
    if (sent || !iq_network_stream_armed()) {
      stream_ring_pop_batch(batch_count);
    } else {
      /* A transient descriptor/socket stall must not turn into a silent RF
       * hole. Leave every frame owned by the ring and retry. A partially sent
       * UDP batch can create duplicates, which the host de-duplicates; it
       * cannot create a missing source chunk. */
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    s_stream_write_active = false;
    return;
  }
  }
#endif
  if (sent || !iq_network_stream_armed()) {
    stream_ring_pop();
  } else {
    /* Preserve the slot across transient USB/Ethernet backpressure. */
    s_stream_write_active = false;
    vTaskDelay(pdMS_TO_TICKS(1));
    return;
  }
#if CONFIG_ESP_SDR_TRANSPORT_USB
  if (iq_network_stream_owner() == IQ_STREAM_OWNER_USB &&
      stream_ring_peek() == NULL) {
    iq_usb_flush();
  }
#endif
  s_stream_write_active = false;
}

void app_main(void) {
#if CONFIG_IDF_TARGET_ESP32S31
  reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
#endif
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("iq_network", ESP_LOG_INFO);
  (void)esp_task_wdt_deinit();
  modem_prepare_direct_phy_access();
  stream_ring_init_storage();
#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_ESP_SDR_TRANSPORT_USB && \
    CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  s_s31_parlio_dma_storage = heap_caps_aligned_calloc(
      64u, 1u, S31_PARLIO_DMA_BYTES,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  ESP_ERROR_CHECK(s_s31_parlio_dma_storage != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  ESP_LOGI("iq_capture", "reserved persistent PARLIO DMA ring at %p",
           s_s31_parlio_dma_storage);
#endif
  init_idf_services();
#if CONFIG_IDF_TARGET_ESP32S31
  ESP_ERROR_CHECK(esp_register_freertos_tick_hook_for_cpu(
      s31_txdc_iwdt_tick_hook, 0));
#endif

  const iq_network_callbacks_t network_callbacks = {
      .get_config = network_get_config,
      .apply_config = network_apply_config,
      .get_firmware_dropped_chunks = network_dropped_chunks,
      .get_source_chunk_index = network_source_chunk,
      .get_adc_dump_cfg = network_adc_dump_cfg,
      .get_adc_dump_mode = network_adc_dump_mode,
      .get_modem_diag_fix_sel = network_modem_diag_fix_sel,
      .get_modem_diag_exchange = network_modem_diag_exchange,
      .get_bb_diag = network_bb_diag,
      .get_dcoc_diag = network_dcoc_diag,
      .get_dcoc_active = network_dcoc_active,
      .is_config_applying = network_config_applying,
      .set_capture_armed = network_set_capture_armed,
      .set_tx_waveform = network_set_tx_waveform,
      .take_tx_waveform = network_take_tx_waveform,
      .reclaim_tx_waveforms = network_reclaim_tx_waveforms,
      .get_tx_replay_diag = network_tx_replay_diag,
  };
#if CONFIG_ESP_SDR_TRANSPORT_ETHERNET
  /* GMAC needs a contiguous pool of DMA-capable internal RAM. Install it
   * before Wi-Fi/PHY capture support consumes and fragments that heap. */
  iq_network_init(&network_callbacks);
#else
  /* The USB control protocol reuses the JSON builders and stream-owner state
   * from iq_network, but must not bring up the Ethernet MAC. */
  iq_control_init(&network_callbacks);
#endif

#if CONFIG_IDF_TARGET_ESP32S31
  /* The combined Ethernet/USB image leaves only a small DMA-capable internal
   * heap after Wi-Fi and TinyUSB start.  PARLIO keeps its acquisition ring in
   * PSRAM, but the IDF driver requires a contiguous internal descriptor list.
   * Hold a small block across stack initialization, then hand that exact hole
   * to the first host-selected PARLIO geometry in prepare(). */
  s_s31_parlio_internal_dma_reserve = heap_caps_aligned_alloc(
      64u, 2048u,
      MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  ESP_ERROR_CHECK(s_s31_parlio_internal_dma_reserve != NULL
                      ? ESP_OK
                      : ESP_ERR_NO_MEM);
#endif

#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_ESP_PHY_ENABLE_CERT_TEST
  init_s31_rftest_services();
#endif

  wifi_tx_rx_config_t wifi_config = wifi_tx_rx_config_from_capture(&s_config);
  ESP_LOGI("iq_capture", "initializing Wi-Fi capture support");
  wifi_tx_rx_init(stream_ring_push_wifi);
  ESP_LOGI("iq_capture", "applying Wi-Fi capture config");
  wifi_tx_rx_apply_config(&wifi_config);

  modem_config_t modem_config = modem_config_from_capture(&s_config);
  ESP_LOGI("iq_capture", "initializing direct modem access");
  ESP_ERROR_CHECK(modem_init(&modem_config));
  ESP_LOGI("iq_capture", "applying direct modem config");
  modem_apply_rx_config(&modem_config);
  ESP_LOGI("iq_capture", "direct modem config ready");
#if !CONFIG_IDF_TARGET_ESP32S31
  modem_calibrate_dco();
#elif !CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF
  ESP_ERROR_CHECK(ulp_lp_core_load_binary(
      s31_tcm_probe_bin_start,
      (size_t)(s31_tcm_probe_bin_end - s31_tcm_probe_bin_start)));
  ulp_lp_core_cfg_t lp_cfg = {
      .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
  };
  ESP_ERROR_CHECK(ulp_lp_core_run(&lp_cfg));
  ESP_LOGI("iq_capture", "LP live-TCM probe reader running");
#endif
#if CONFIG_IDF_TARGET_ESP32S31
  s31_tcm_gate_watchdog_init();
#endif
  s_trigger_plan = build_trigger_plan(&s_config);
  s_config_apply_enabled = true;

#if CONFIG_ESP_SDR_TRANSPORT_USB
  iq_usb_init(&network_callbacks);
#endif
#if CONFIG_IDF_TARGET_ESP32S31
  __atomic_store_n(&s_s31_txdc_stager_done, true, __ATOMIC_RELEASE);
  ESP_ERROR_CHECK(
      xTaskCreatePinnedToCoreWithCaps(
          s31_txdc_tcm_stager_task, "tx_tcm_stage", 4096, NULL, 24,
          &s_s31_txdc_stager_task_handle, 0,
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS
          ? ESP_OK
          : ESP_ERR_NO_MEM);
  const esp_timer_create_args_t txdc_stager_wakeup_args = {
      .callback = s31_txdc_stager_wakeup_isr,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_ISR,
      .name = "tx_stage_wake",
      .skip_unhandled_events = true,
  };
  ESP_ERROR_CHECK(esp_timer_create(&txdc_stager_wakeup_args,
                                   &s_s31_txdc_stager_wakeup_timer));
#endif
  ESP_ERROR_CHECK(
      xTaskCreatePinnedToCoreWithCaps(
          producer_task, "iq_producer", IQ_CHUNK_PRODUCER_TASK_STACK_BYTES,
          NULL, IQ_CHUNK_PRODUCER_TASK_PRIORITY, &s_producer_task_handle, 1,
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS
          ? ESP_OK
          : ESP_ERR_NO_MEM);

#if !CONFIG_IDF_TARGET_ESP32S31
  restart_capture(&s_config);
#endif

  vTaskPrioritySet(NULL, STREAM_TASK_PRIORITY);
  ESP_LOGI("iq_capture", "stream running on core %d", xPortGetCoreID());
  while (true) {
    service_pending_config();
    if (!s_network_capture_armed && capture_engine_running(&s_config)
#if CONFIG_IDF_TARGET_ESP32S31
        && !s31_tx_replay_selected(&s_config)
#endif
    ) {
      reg32p_clear_bits(&MODEM_WIFI_DUMP.ADC_DUMP_CTRL,
                        MODEM_WIFI_DUMP_CTRL_ENABLE_BIT);
      reg32p_clear_bits(&MODEM_WIFI_DUMP.MACTOADCDUMP0,
                        MODEM_WIFI_TOADCDUMP_ENABLE_BIT);
#if CONFIG_IDF_TARGET_ESP32S31
      reg32_write_addr(HP_SYSTEM_TCM_DATA_DUMP_CTRL_REG, 0u);
#endif
      stream_state_reset();
    }
    stream_next_frame();
  }
}
