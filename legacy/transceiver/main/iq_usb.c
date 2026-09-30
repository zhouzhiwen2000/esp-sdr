/* USB transport for control and IQ streaming.
 *
 * Implements a vendor-specific USB device on the ESP32-S31's high-speed
 * USB-OTG port with a minimal custom TinyUSB class driver. The stream path
 * follows the same principles as the Ethernet data plane: staging buffers are
 * handed directly to the endpoint DMA (no class-driver FIFO copies) and
 * transfer completions are handled in the ISR fast path, with a two-slot
 * pipeline so the bus never idles while the stream task refills.
 *
 * See iq_usb.h for the wire protocol.
 */
#include "iq_usb.h"
#include "ringbuffer.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_cache.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "riscv/csr.h"
#include "soc/cnnt_sys_struct.h"
#include "soc/hp_alive_sys_struct.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/usb_dwc_struct.h"
#include "soc/usb_utmi_struct.h"

#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "device/usbd_pvt.h"

#include "cJSON.h"

#define IQ_USB_EP_CTRL_OUT 0x01u
#define IQ_USB_EP_CTRL_IN 0x81u
#define IQ_USB_EP_STREAM_IN 0x82u
#define IQ_USB_EP_TX_OUT 0x02u
#define IQ_USB_STREAM_EP_NUM 2u
/* Keep OUT transactions packet-aligned but short. 4 KiB transactions bound
 * DWC2's PSRAM-fabric occupation while still amortizing TinyUSB scheduling
 * over eight high-speed packets. */
#define IQ_USB_TX_TRANSFER_BYTES (32u * 1024u)

/* Three slots in PSRAM mode (fill one while one is in flight and one is
 * parked); two in the smaller internal-memory fallback. */
#define IQ_USB_SLOT_MAX 3u
#define IQ_USB_MESSAGE_MAX_BYTES                                               \
  (sizeof(iq_udp_header_t) + sizeof(stream_frame_t))
#define IQ_USB_IQ8_MESSAGE_BYTES                                               \
  (sizeof(iq_udp_header_t) + IQ8_FRAME_WIRE_BYTES)
#define IQ_USB_IQ4_MESSAGE_BYTES                                               \
  (sizeof(iq_udp_header_t) + IQ4_FRAME_WIRE_BYTES)
/* Batch several frame messages per USB transfer: per-transfer overhead
 * (endpoint re-arm, completion interrupt) is ~30 us, which caps
 * single-frame transfers well below the full-duty decimation-8 rate.
 * Buffers are allocated on first use so Ethernet's boot-time DMA pool is
 * unaffected, and the batch shrinks if internal memory is scarce. */
#define IQ_USB_BATCH_FRAMES_MAX 6u
#define IQ_USB_DIRECT_BATCH_FRAMES 14u
#define IQ_USB_DIRECT_MAX_SLOTS 2u
#define IQ_USB_DIRECT_MAX_FRAMES                                            \
  (IQ_USB_DIRECT_BATCH_FRAMES * IQ_USB_DIRECT_MAX_SLOTS)
#define IQ_USB_DIRECT_SLAB_BYTES (256u * 1024u)
#define IQ_USB_DIRECT_PMA_ENTRY 6u
#define IQ_USB_SLOT_TIMEOUT_MS 250u

/* Bulk OUT transfer sizes must be a multiple of the endpoint size. */
#define IQ_USB_CTRL_REQUEST_BYTES                                              \
  (((sizeof(iq_usb_ctrl_request_t) + IQ_USB_CTRL_MAX_PAYLOAD + 511u) / 512u) * \
   512u)
#define IQ_USB_CTRL_RESPONSE_BYTES                                             \
  (sizeof(iq_usb_ctrl_response_t) + IQ_USB_CTRL_MAX_PAYLOAD + 4u)

static const char *TAG = "iq_usb";

static iq_network_callbacks_t s_callbacks;
static SemaphoreHandle_t s_slot_sem;
static portMUX_TYPE s_usb_mux = portMUX_INITIALIZER_UNLOCKED;

CFG_TUSB_MEM_SECTION static uint8_t s_ctrl_request[IQ_USB_CTRL_REQUEST_BYTES];
CFG_TUSB_MEM_SECTION static uint8_t s_ctrl_response[IQ_USB_CTRL_RESPONSE_BYTES];
/* One internal frame validates the PIE packer's byte order on first use. The
 * checked fast path subsequently writes straight into the coherent USB slab. */
CFG_TUSB_MEM_SECTION static uint8_t
    s_iq8_scratch[2u * IQ_CHUNK_SAMPLE_WORDS]
    __attribute__((aligned(4)));
extern void s31_pie_pack_iq8(void *dst, const void *src,
                            uint32_t sample_count);
static bool s_pie_iq8_checked;
static bool s_pie_iq8_valid;

static uint8_t *s_stream_slot[IQ_USB_SLOT_MAX]; /* lazily allocated */
static uint8_t *s_stream_slab;
static uint32_t s_slot_count;
static uint32_t s_slot_capacity;
/* Slots in cached PSRAM allow much larger batches than internal SRAM, but
 * need an explicit cache writeback before the endpoint DMA reads them, so
 * the completion ISR must not steal the (unsynced) fill slot. */
static bool s_slots_psram;
static volatile uint32_t s_direct_pma_core_mask;

/* Slot lifecycle, guarded by s_usb_mux: FREE -> fill (stream task appends
 * messages) -> parked in the pending queue -> inflight (endpoint DMA) ->
 * FREE. In internal-memory mode the completion ISR additionally steals the
 * partially filled slot whenever the endpoint would otherwise idle, so
 * batches grow only as far as the arrival rate requires. */
static uint16_t s_slot_bytes[IQ_USB_SLOT_MAX];
static volatile bool s_slot_busy[IQ_USB_SLOT_MAX];
static volatile int8_t s_inflight_slot = -1;
static int8_t s_pending_queue[IQ_USB_SLOT_MAX];
static volatile uint8_t s_pending_head;
static volatile uint8_t s_pending_count;
static volatile int8_t s_fill_slot = -1;
static volatile uint32_t s_fill_bytes;
/* Set while the stream task is copying into the fill slot; the ISR must not
 * submit a half-written batch. */
static volatile bool s_fill_writing;
/* The S31 producer may coalesce two endpoint-sized batches into one TCM
 * ownership transaction. Neither slot is visible to the generic path until
 * commit parks/submits both complete batches. */
static volatile int8_t s_direct_fill_slots[IQ_USB_DIRECT_MAX_SLOTS] = {-1, -1};
static volatile uint8_t s_direct_fill_count;
static volatile uint32_t s_direct_frame_bytes;
static volatile uint32_t s_direct_message_bytes;
static volatile bool s_direct_cache_sync;

/* Discard endpoint data from an earlier stream generation.  This is also
 * used before a new start so a client which did not shut down cleanly cannot
 * inherit queued IQ from the previous owner. */
static bool stream_discard_queued_data(void);

/* Call with s_usb_mux held. */
static inline void pending_push_locked(int8_t slot) {
  s_pending_queue[(s_pending_head + s_pending_count) % IQ_USB_SLOT_MAX] = slot;
  ++s_pending_count;
}

static inline int pending_pop_locked(void) {
  if (s_pending_count == 0u) {
    return -1;
  }
  const int slot = s_pending_queue[s_pending_head];
  s_pending_head = (uint8_t)((s_pending_head + 1u) % IQ_USB_SLOT_MAX);
  --s_pending_count;
  return slot;
}

static uint8_t s_rhport;
static volatile uint8_t s_ep_ctrl_out;
static volatile uint8_t s_ep_ctrl_in;
static volatile uint8_t s_ep_stream_in;
static volatile uint8_t s_ep_tx_out;
static volatile uint32_t s_usb_frames;
static volatile uint32_t s_usb_send_errors;
/* IQ_USB_FORMAT_*: selected per stream via the stream-start payload.
 * The producers emit IQC8 frames directly when int8 is selected. */
static volatile uint32_t s_stream_format;
/* Cycle diagnostics: total stream-task cycles inside send, and the share
 * spent blocked waiting for a free transfer slot. */
static volatile uint64_t s_diag_send_cycles;
static volatile uint64_t s_diag_wait_cycles;
static volatile uint64_t s_diag_pack_cycles;
static volatile uint64_t s_diag_header_cycles;
static volatile uint64_t s_diag_cache_sync_cycles;
static volatile uint64_t s_diag_completion_cycles;
static volatile uint32_t s_diag_submissions;
static volatile uint32_t s_diag_submission_bytes;
static volatile uint32_t s_diag_completions;
static volatile uint32_t s_diag_completion_bytes;
static volatile uint32_t s_diag_max_completion_cycles;
static volatile uint32_t s_slot_submit_cycle[IQ_USB_SLOT_MAX];

/* Host-to-radio upload. DWC2 writes directly into the final aligned PSRAM
 * allocation in endpoint-sized pieces; commit then transfers ownership to
 * the replay engine without a second full-waveform copy. TinyUSB serializes
 * endpoint callbacks in its device task, so control and TX completion state
 * need no additional mutex. */
static uint32_t *s_tx_upload;
static uint32_t s_tx_word_count;
static uint32_t s_tx_received_bytes;
static uint32_t s_tx_armed_bytes;
static uint16_t s_tx_commit_flags;
static uint64_t s_tx_start_time_ns;
static volatile uint32_t s_usb_tx_uploads;
static volatile uint32_t s_usb_tx_bytes;
static volatile uint32_t s_usb_tx_errors;
static volatile uint32_t s_usb_tx_backpressure_retries;
static volatile uint32_t s_usb_tx_commit_rejections;
#define IQ_USB_TX_POOL_COUNT 4u
static uint32_t *s_tx_pool[IQ_USB_TX_POOL_COUNT];
static volatile uint32_t s_tx_pool_busy[IQ_USB_TX_POOL_COUNT];
static bool s_tx_network_suspended;
static size_t s_tx_pool_capacity;
static uint32_t s_tx_last_pool_wait_ms;
static int64_t s_tx_arm_started_us;

/* ---------------- Descriptors ------------------------------------------- */
static const tusb_desc_device_t s_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x303A,
    .idProduct = 0x4531,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

#define IQ_USB_EP_DESC(addr, mps)                                              \
  7u, TUSB_DESC_ENDPOINT, (addr), TUSB_XFER_BULK, U16_TO_U8S_LE(mps), 0u

#define IQ_USB_CONFIG_DESC(mps)                                                \
  TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_CONFIG_DESC_LEN + 9u + 4u * 7u, 0, 500), \
      /* Vendor interface: control pair, RX stream IN, TX stream OUT. */       \
      9u, TUSB_DESC_INTERFACE, 0u, 0u, 4u, TUSB_CLASS_VENDOR_SPECIFIC, 0u,     \
      0u, 4u, IQ_USB_EP_DESC(IQ_USB_EP_CTRL_OUT, mps),                         \
      IQ_USB_EP_DESC(IQ_USB_EP_CTRL_IN, mps),                                  \
      IQ_USB_EP_DESC(IQ_USB_EP_STREAM_IN, mps),                                \
      IQ_USB_EP_DESC(IQ_USB_EP_TX_OUT, mps)

static const uint8_t s_fs_config_desc[] = {IQ_USB_CONFIG_DESC(64u)};
static const uint8_t s_hs_config_desc[] = {IQ_USB_CONFIG_DESC(512u)};

static char s_serial[13] = "000000000000";
static const char *s_string_desc[] = {
    (const char[]){0x09, 0x04}, /* 0: LANGID: en-US */
    "ESPARGOS",                 /* 1: manufacturer */
    "ESP-SDR",                  /* 2: product (used for host discovery) */
    s_serial,                   /* 3: serial (base MAC) */
    "ESP-SDR IQ stream and control", /* 4: interface */
};

/* ---------------- Control channel --------------------------------------- */
static void ctrl_prepare_request_read(void) {
  (void)usbd_edpt_xfer(s_rhport, IQ_USB_EP_CTRL_OUT, s_ctrl_request,
                       IQ_USB_CTRL_REQUEST_BYTES, false);
}

bool iq_usb_tx_recycle(uint32_t *buffer) {
  if (buffer == NULL) {
    return false;
  }
  for (uint32_t i = 0u; i < IQ_USB_TX_POOL_COUNT; ++i) {
    if (s_tx_pool[i] == buffer) {
      /* Pool pointers are immutable from first TX arm until RF teardown.
       * Releasing ownership must not spin on the RX/endpoint mux: DWC2 can
       * hold that lock long enough to perturb the realtime core. */
      __atomic_store_n(&s_tx_pool_busy[i], 0u, __ATOMIC_RELEASE);
      return true;
    }
  }
  return false;
}

void iq_usb_tx_pool_trim(void) {
  uint32_t *retired[IQ_USB_TX_POOL_COUNT] = {0};
  taskENTER_CRITICAL(&s_usb_mux);
  bool all_free = true;
  for (uint32_t i = 0u; i < IQ_USB_TX_POOL_COUNT; ++i) {
    all_free = all_free &&
               __atomic_load_n(&s_tx_pool_busy[i], __ATOMIC_ACQUIRE) == 0u;
  }
  if (all_free) {
    for (uint32_t i = 0u; i < IQ_USB_TX_POOL_COUNT; ++i) {
      retired[i] = s_tx_pool[i];
      s_tx_pool[i] = NULL;
      __atomic_store_n(&s_tx_pool_busy[i], 0u, __ATOMIC_RELAXED);
    }
    s_tx_pool_capacity = 0u;
  }
  taskEXIT_CRITICAL(&s_usb_mux);
  for (uint32_t i = 0u; i < IQ_USB_TX_POOL_COUNT; ++i) {
    heap_caps_free(retired[i]);
  }
}

static bool tx_pool_prepare(size_t allocation_bytes) {
  if (s_tx_pool_capacity >= allocation_bytes) {
    return true;
  }
  if (s_tx_pool_capacity != 0u) {
    return false;
  }
  uint32_t *prepared[IQ_USB_TX_POOL_COUNT] = {0};
  for (uint32_t i = 0u; i < IQ_USB_TX_POOL_COUNT; ++i) {
    prepared[i] = heap_caps_aligned_alloc(
        64u, allocation_bytes,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (prepared[i] == NULL) {
      for (uint32_t j = 0u; j < IQ_USB_TX_POOL_COUNT; ++j) {
        heap_caps_free(prepared[j]);
      }
      return false;
    }
  }
  taskENTER_CRITICAL(&s_usb_mux);
  for (uint32_t i = 0u; i < IQ_USB_TX_POOL_COUNT; ++i) {
    s_tx_pool[i] = prepared[i];
    __atomic_store_n(&s_tx_pool_busy[i], 0u, __ATOMIC_RELAXED);
  }
  s_tx_pool_capacity = allocation_bytes;
  taskEXIT_CRITICAL(&s_usb_mux);
  return true;
}

static uint32_t *tx_pool_claim(size_t allocation_bytes) {
  if (s_tx_pool_capacity >= allocation_bytes) {
    for (uint32_t i = 0u; i < IQ_USB_TX_POOL_COUNT; ++i) {
      uint32_t expected = 0u;
      if (s_tx_pool[i] != NULL &&
          __atomic_compare_exchange_n(&s_tx_pool_busy[i], &expected, 1u,
                                      false, __ATOMIC_ACQ_REL,
                                      __ATOMIC_ACQUIRE)) {
        return s_tx_pool[i];
      }
    }
  }
  return NULL;
}

static void tx_upload_forget(bool free_buffer) {
  if (free_buffer) {
    if (!iq_usb_tx_recycle(s_tx_upload)) {
      heap_caps_free(s_tx_upload);
    }
  }
  s_tx_upload = NULL;
  s_tx_word_count = 0u;
  s_tx_received_bytes = 0u;
  s_tx_armed_bytes = 0u;
  s_tx_commit_flags = 0u;
  s_tx_start_time_ns = 0u;
}

static uint32_t tx_upload_wire_bytes(uint32_t words, uint16_t flags) {
  if ((flags & IQ_TX_USB_FLAG_PACKED20) != 0u) {
    return (words * 5u + 1u) / 2u;
  }
  return words * sizeof(uint32_t);
}

static bool tx_upload_arm_next(void) {
  if (s_tx_upload == NULL || s_ep_tx_out == 0u) {
    return false;
  }
  const uint32_t total_bytes =
      tx_upload_wire_bytes(s_tx_word_count, s_tx_commit_flags);
  if (s_tx_received_bytes >= total_bytes) {
    s_tx_armed_bytes = 0u;
    return true;
  }
  uint32_t bytes = total_bytes - s_tx_received_bytes;
  if (bytes > IQ_USB_TX_TRANSFER_BYTES) {
    bytes = IQ_USB_TX_TRANSFER_BYTES;
  }
  s_tx_armed_bytes = bytes;
  return usbd_edpt_xfer(
      s_rhport, IQ_USB_EP_TX_OUT,
      (uint8_t *)s_tx_upload + s_tx_received_bytes, (uint16_t)bytes, false);
}

static const char *tx_upload_begin(const uint8_t *payload,
                                   uint32_t payload_bytes) {
  if (payload_bytes != 8u && payload_bytes != sizeof(iq_usb_tx_arm_t)) {
    return "USB TX arm payload must be 8 or 16 bytes";
  }
  if (s_tx_upload != NULL ||
      (s_ep_tx_out != 0u && usbd_edpt_busy(s_rhport, IQ_USB_EP_TX_OUT))) {
    return "USB TX upload already active";
  }
  iq_usb_tx_arm_t arm = {0};
  memcpy(&arm, payload, payload_bytes);
  const uint16_t allowed =
      IQ_TX_UDP_FLAG_AUTOSTART | IQ_TX_UDP_RATE_CODE_M |
      IQ_TX_UDP_FLAG_MORE | IQ_TX_UDP_FLAG_CONTINUE |
      IQ_TX_USB_FLAG_PACKED20;
  const uint32_t rate_code =
      (arm.commit_flags & IQ_TX_UDP_RATE_CODE_M) >> IQ_TX_UDP_RATE_CODE_S;
  const bool rate_valid = rate_code <= 3u ||
                          (rate_code >= 7u && rate_code <= 15u);
  if (arm.word_count == 0u || arm.word_count > TX_BATCH_WORDS_MAX ||
      arm.reserved != 0u || (arm.commit_flags & ~allowed) != 0u ||
      ((arm.commit_flags & IQ_TX_UDP_FLAG_AUTOSTART) != 0u && !rate_valid) ||
      ((arm.commit_flags & IQ_TX_USB_FLAG_PACKED20) != 0u &&
       (((arm.commit_flags & IQ_TX_UDP_FLAG_AUTOSTART) == 0u) ||
        (rate_code != 14u && rate_code != 15u))) ||
      ((arm.commit_flags & IQ_TX_UDP_FLAG_AUTOSTART) == 0u &&
       ((arm.commit_flags & (IQ_TX_UDP_RATE_CODE_M |
                             IQ_TX_UDP_FLAG_MORE)) != 0u ||
        arm.start_time_ns != 0u))) {
    return "invalid USB TX arm parameters";
  }
  if ((arm.commit_flags & IQ_TX_UDP_FLAG_AUTOSTART) != 0u &&
      (arm.commit_flags & IQ_TX_UDP_FLAG_CONTINUE) == 0u &&
      !s_tx_network_suspended) {
    iq_network_suspend_for_usb_tx();
    s_tx_network_suspended = true;
  }
  const size_t upload_bytes =
      tx_upload_wire_bytes(arm.word_count, arm.commit_flags);
  const size_t allocation_bytes = (upload_bytes + 63u) & ~(size_t)63u;
  const bool use_pool = tx_pool_prepare(allocation_bytes);
  s_tx_arm_started_us = esp_timer_get_time();
  uint32_t pool_waits = 0u;
  for (uint32_t attempt = 0u; attempt < 300u && s_tx_upload == NULL;
       ++attempt) {
    if (s_callbacks.reclaim_tx_waveforms != NULL) {
      s_callbacks.reclaim_tx_waveforms();
    }
    s_tx_upload = use_pool
                      ? tx_pool_claim(allocation_bytes)
                      : heap_caps_aligned_alloc(
                            64u, allocation_bytes,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA |
                                MALLOC_CAP_8BIT);
    if (s_tx_upload == NULL) {
      ++pool_waits;
      vTaskDelay(1);
    }
  }
  s_tx_last_pool_wait_ms = pool_waits;
  if (s_tx_upload == NULL) {
    if (s_tx_network_suspended) {
      iq_network_resume_after_usb_tx();
      s_tx_network_suspended = false;
    }
    return "USB TX allocation failed";
  }
  s_tx_word_count = arm.word_count;
  s_tx_received_bytes = 0u;
  s_tx_commit_flags = arm.commit_flags;
  s_tx_start_time_ns = arm.start_time_ns;
  if (!tx_upload_arm_next()) {
    tx_upload_forget(true);
    if (s_tx_network_suspended) {
      iq_network_resume_after_usb_tx();
      s_tx_network_suspended = false;
    }
    return "could not arm USB TX endpoint";
  }
  return NULL;
}

static const char *tx_upload_commit(void) {
  if (s_tx_upload == NULL) {
    return "no USB TX upload is active";
  }
  const uint32_t total_bytes =
      tx_upload_wire_bytes(s_tx_word_count, s_tx_commit_flags);
  const bool final_batch =
      (s_tx_commit_flags & IQ_TX_UDP_FLAG_MORE) == 0u;
  if (s_tx_received_bytes != total_bytes || s_tx_armed_bytes != 0u ||
      usbd_edpt_busy(s_rhport, IQ_USB_EP_TX_OUT)) {
    return "USB TX upload is incomplete";
  }
  bool adopted = false;
  bool stored = false;
  if (s_callbacks.take_tx_waveform != NULL) {
    const iq_tx_waveform_result_t result = s_callbacks.take_tx_waveform(
        s_tx_upload, s_tx_word_count, s_tx_commit_flags, s_tx_start_time_ns);
    if (result == IQ_TX_WAVEFORM_RETRY) {
      ++s_usb_tx_backpressure_retries;
      return "USB TX replay engine is busy";
    }
    adopted = result == IQ_TX_WAVEFORM_ADOPTED;
    stored = adopted;
  } else if (s_callbacks.set_tx_waveform != NULL) {
    stored = s_callbacks.set_tx_waveform(s_tx_upload, s_tx_word_count);
  }
  if (!stored) {
    ++s_usb_tx_commit_rejections;
    ESP_LOGW(TAG,
             "USB TX adopt failed: pool_wait_ms=%" PRIu32
             " arm_elapsed_us=%" PRIi64,
             s_tx_last_pool_wait_ms,
             esp_timer_get_time() - s_tx_arm_started_us);
    if (s_tx_network_suspended) {
      iq_network_resume_after_usb_tx();
      s_tx_network_suspended = false;
    }
    return "USB TX replay engine rejected upload";
  }
  s_usb_tx_bytes += total_bytes;
  ++s_usb_tx_uploads;
  tx_upload_forget(!adopted);
  if (final_batch && s_tx_network_suspended) {
    iq_network_resume_after_usb_tx();
    s_tx_network_suspended = false;
  }
  return NULL;
}

static const char *tx_upload_abort(void) {
  if (s_ep_tx_out != 0u) {
    /* DWC2 has no single-endpoint close implementation in TinyUSB.  A
     * stall/clear cycle cancels the outstanding OUT DMA and resets DATA0. */
    usbd_edpt_stall(s_rhport, IQ_USB_EP_TX_OUT);
    usbd_edpt_clear_stall(s_rhport, IQ_USB_EP_TX_OUT);
  }
  tx_upload_forget(true);
  if (s_tx_network_suspended) {
    iq_network_resume_after_usb_tx();
    s_tx_network_suspended = false;
  }
  if (tud_mounted() && s_ep_tx_out != 0u) {
    return NULL;
  }
  ++s_usb_tx_errors;
  return "USB TX endpoint is not available";
}

static void ctrl_process_request(uint32_t request_bytes) {
  const iq_usb_ctrl_request_t *request =
      (const iq_usb_ctrl_request_t *)s_ctrl_request;
  iq_usb_ctrl_response_t *response = (iq_usb_ctrl_response_t *)s_ctrl_response;
  char *payload_out = (char *)(s_ctrl_response + sizeof(*response));
  const size_t payload_cap = IQ_USB_CTRL_MAX_PAYLOAD;

  memcpy(response->magic, "IQRS", 4u);
  response->sequence = 0u;
  response->status = 1u;
  response->payload_bytes = 0u;

  const char *error = NULL;
  capture_config_t pending_config;
  bool apply_pending_config = false;

  if (request_bytes < sizeof(*request) ||
      memcmp(request->magic, "IQRQ", 4u) != 0 ||
      request->payload_bytes > IQ_USB_CTRL_MAX_PAYLOAD ||
      sizeof(*request) + request->payload_bytes > request_bytes) {
    error = "malformed control request";
  } else {
    response->sequence = request->sequence;
    const char *payload_in =
        (const char *)(s_ctrl_request + sizeof(*request));
    int n = -1;
    switch (request->opcode) {
    case IQ_USB_OP_GET_STATUS:
      n = iq_control_build_status_json(payload_out, payload_cap);
      error = n < 0 ? "status JSON too large" : NULL;
      break;
    case IQ_USB_OP_GET_CONFIG:
      n = iq_control_build_config_json(payload_out, payload_cap);
      error = n < 0 ? "config JSON too large" : NULL;
      break;
    case IQ_USB_OP_PUT_CONFIG: {
      cJSON *json =
          cJSON_ParseWithLength(payload_in, request->payload_bytes);
      if (json == NULL) {
        error = "malformed JSON";
        break;
      }
      error = iq_control_parse_config_json(json, &pending_config);
      cJSON_Delete(json);
      if (error == NULL) {
        apply_pending_config = true;
        n = 0;
      }
      break;
    }
    case IQ_USB_OP_STREAM_START: {
      uint32_t format = IQ_USB_FORMAT_FULL;
      if (request->payload_bytes > 0u) {
        cJSON *json =
            cJSON_ParseWithLength(payload_in, request->payload_bytes);
        if (json != NULL) {
          cJSON *item =
              cJSON_GetObjectItemCaseSensitive(json, "stream_format");
          if (cJSON_IsNumber(item) &&
              (item->valuedouble == (double)IQ_USB_FORMAT_FULL ||
               item->valuedouble == (double)IQ_USB_FORMAT_INT8 ||
               item->valuedouble == (double)IQ_USB_FORMAT_INT4)) {
            format = (uint32_t)item->valuedouble;
          }
          cJSON_Delete(json);
        }
      }
      iq_network_stream_end();
      if (!stream_discard_queued_data()) {
        error = "USB stream endpoint did not quiesce";
        break;
      }
      s_stream_format = format;
      iq_network_stream_set_format(format);
      iq_network_stream_begin(IQ_STREAM_OWNER_USB);
      n = iq_control_build_status_json(payload_out, payload_cap);
      error = n < 0 ? "status JSON too large" : NULL;
      break;
    }
    case IQ_USB_OP_STREAM_STOP:
      iq_network_stream_end();
      if (!stream_discard_queued_data()) {
        error = "USB stream endpoint did not quiesce";
        break;
      }
      n = iq_control_build_status_json(payload_out, payload_cap);
      error = n < 0 ? "status JSON too large" : NULL;
      break;
    case IQ_USB_OP_TX_ARM:
      error = tx_upload_begin((const uint8_t *)payload_in,
                              request->payload_bytes);
      n = 0;
      break;
    case IQ_USB_OP_TX_COMMIT:
      if (request->payload_bytes != 0u) {
        error = "USB TX commit payload must be empty";
      } else {
        error = tx_upload_commit();
      }
      n = 0;
      break;
    case IQ_USB_OP_TX_ABORT:
      if (request->payload_bytes != 0u) {
        error = "USB TX abort payload must be empty";
      } else {
        error = tx_upload_abort();
      }
      n = 0;
      break;
    default:
      error = "unknown opcode";
      break;
    }
    if (error == NULL) {
      response->status = 0u;
      response->payload_bytes = (uint32_t)(n > 0 ? n : 0);
    }
  }

  if (error != NULL) {
    size_t len = strlen(error);
    if (len > payload_cap)
      len = payload_cap;
    memcpy(payload_out, error, len);
    response->status = response->status != 0u ? response->status : 1u;
    response->payload_bytes = (uint32_t)len;
  }

  /* Guarantee short-packet termination for exact-multiple responses. */
  uint32_t total = sizeof(*response) + response->payload_bytes;
  if ((total % 512u) == 0u) {
    payload_out[response->payload_bytes] = ' ';
    ++response->payload_bytes;
    ++total;
  }
  (void)usbd_edpt_xfer(s_rhport, IQ_USB_EP_CTRL_IN, s_ctrl_response,
                       (uint16_t)total, false);

  /* Queue configuration and arming work only after the response transfer is
   * owned by the hardware, mirroring the HTTP handler ordering. */
  if (apply_pending_config) {
    s_callbacks.apply_config(&pending_config);
  }
  if (error == NULL && request->opcode == IQ_USB_OP_STREAM_START) {
    iq_network_stream_arm();
  }
}

/* ---------------- Stream data plane -------------------------------------- */
static void stream_reset_slots(void) {
  uint32_t freed = 0u;
  taskENTER_CRITICAL(&s_usb_mux);
  int slot = s_inflight_slot;
  s_inflight_slot = -1;
  if (slot >= 0 && s_slot_busy[slot]) {
    s_slot_busy[slot] = false;
    ++freed;
  }
  while ((slot = pending_pop_locked()) >= 0) {
    if (s_slot_busy[slot]) {
      s_slot_busy[slot] = false;
      ++freed;
    }
  }
  for (uint32_t i = 0u; i < s_direct_fill_count; ++i) {
    slot = s_direct_fill_slots[i];
    s_direct_fill_slots[i] = -1;
    if (slot >= 0 && s_slot_busy[slot]) {
      s_slot_busy[slot] = false;
      ++freed;
    }
  }
  s_direct_fill_count = 0u;
  s_direct_frame_bytes = 0u;
  s_direct_message_bytes = 0u;
  s_direct_cache_sync = false;
  slot = s_fill_slot;
  s_fill_slot = -1;
  s_fill_bytes = 0u;
  if (slot >= 0 && s_slot_busy[slot]) {
    s_slot_busy[slot] = false;
    ++freed;
  }
  taskEXIT_CRITICAL(&s_usb_mux);
  if (s_slot_sem != NULL) {
    for (uint32_t i = 0u; i < freed; ++i) {
      xSemaphoreGive(s_slot_sem);
    }
  }
}

static bool stream_discard_queued_data(void) {
  if (s_ep_stream_in == 0u) {
    stream_reset_slots();
    return true;
  }

  /* stream_end() prevents new transport tickets.  Allow the one producer
   * copy which may already own a slot to finish before reclaiming it. */
  bool quiet = false;
  const uint32_t quiesce_timeout_us =
      IQ_USB_SLOT_TIMEOUT_MS * 1000u + 50000u;
  const int64_t quiesce_deadline_us =
      esp_timer_get_time() + quiesce_timeout_us;
  while (esp_timer_get_time() < quiesce_deadline_us) {
    taskENTER_CRITICAL(&s_usb_mux);
    quiet = !s_fill_writing && s_direct_fill_count == 0u;
    taskEXIT_CRITICAL(&s_usb_mux);
    if (quiet) {
      break;
    }
    /* The producer which clears s_fill_writing can share this core with the
     * TinyUSB task. A busy delay starves it for the entire timeout and turns a
     * recoverable stop into a permanently wedged endpoint. */
    vTaskDelay(1);
  }
  if (!quiet) {
    return false;
  }

  /* DWC2 does not implement TinyUSB's single-endpoint close API.  Stall
   * disables the IN endpoint and cancels its outstanding DMA transaction;
   * clearing it resets DATA0 and leaves the descriptor configured. */
  usbd_edpt_stall(s_rhport, IQ_USB_EP_STREAM_IN);
  stream_reset_slots();
  usbd_edpt_clear_stall(s_rhport, IQ_USB_EP_STREAM_IN);
  return true;
}

static bool stream_slots_allocate(void) {
  if (s_stream_slot[0] != NULL) {
    return true;
  }
  /* Full-rate staged capture writes each frame directly into one of these
   * contiguous PSRAM DMA batches. DWC2 transmits the previous batch while the
   * modem owns TCM for the next one, avoiding a second PSRAM copy and allowing
   * USB to keep progressing while both HP cores are quiesced. */
  const uint32_t direct_capacity =
      (IQ_USB_DIRECT_BATCH_FRAMES * IQ_USB_MESSAGE_MAX_BYTES + 63u) & ~63u;
  _Static_assert(IQ_USB_SLOT_MAX *
                         ((IQ_USB_DIRECT_BATCH_FRAMES *
                               IQ_USB_MESSAGE_MAX_BYTES +
                           63u) &
                          ~63u) <=
                     IQ_USB_DIRECT_SLAB_BYTES,
                 "direct USB slots exceed their PMA slab");
  s_stream_slab = heap_caps_aligned_alloc(
      IQ_USB_DIRECT_SLAB_BYTES, IQ_USB_DIRECT_SLAB_BYTES,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (s_stream_slab != NULL) {
    for (uint32_t i = 0u; i < IQ_USB_SLOT_MAX; ++i) {
      s_stream_slot[i] = s_stream_slab + i * direct_capacity;
    }
    s_slot_count = IQ_USB_SLOT_MAX;
    s_slot_capacity = direct_capacity;
    s_slots_psram = true;
    ESP_LOGI(TAG, "stream buffers: %u x %" PRIu32
                  " B zero-copy PSRAM (%u frames)",
             IQ_USB_SLOT_MAX, direct_capacity, IQ_USB_DIRECT_BATCH_FRAMES);
    return true;
  }
  s_stream_slab = NULL;

  /* Retain a small internal-DMA fallback for control/config reporting and
   * reduced-rate streams when PSRAM DMA allocation is unavailable. */
  for (uint32_t batch = IQ_USB_BATCH_FRAMES_MAX; batch >= 1u; --batch) {
    const uint32_t capacity =
        batch * IQ_USB_MESSAGE_MAX_BYTES + IQ_USB_MESSAGE_MAX_BYTES / 4u;
    uint8_t *first = heap_caps_malloc(
        capacity, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    uint8_t *second = heap_caps_malloc(
        capacity, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (first != NULL && second != NULL) {
      s_stream_slot[0] = first;
      s_stream_slot[1] = second;
      s_slot_count = 2u;
      s_slot_capacity = capacity;
      /* Burn the semaphore tokens of the unallocated slots. */
      for (uint32_t i = s_slot_count; i < IQ_USB_SLOT_MAX; ++i) {
        (void)xSemaphoreTake(s_slot_sem, 0);
      }
      ESP_LOGI(TAG, "stream buffers: 2 x %" PRIu32 " B (%" PRIu32
                    " frames per transfer)",
               capacity, batch);
      return true;
    }
    free(first);
    free(second);
  }
  return false;
}

/* IDF maps the complete S31 PSRAM range through broad PMA entry 7. Override
 * only the aligned USB slab with the earlier, more-specific entry 6 on the IQ
 * producer core. Write-through keeps DWC2 coherent without imposing external
 * memory latency on the producer's PSRAM task stack or the Ethernet buffers. */
static bool stream_direct_enable_coherent_slab(void) {
  if (s_stream_slab == NULL ||
      (((uintptr_t)s_stream_slab & (IQ_USB_DIRECT_SLAB_BYTES - 1u)) != 0u)) {
    return false;
  }
  const uint32_t core_bit = 1u << xPortGetCoreID();
  if ((s_direct_pma_core_mask & core_bit) != 0u) {
    return true;
  }

  const uintptr_t pma_addr =
      ((uintptr_t)s_stream_slab | (IQ_USB_DIRECT_SLAB_BYTES / 2u - 1u)) >>
      PMA_SHIFT;
  const uint32_t pma_cfg = PMA_NAPOT | PMA_EN | PMA_R | PMA_W |
                           PMA_WRITETHROUGH;
  RV_WRITE_CSR(CSR_PMACFG(IQ_USB_DIRECT_PMA_ENTRY), 0u);
  RV_WRITE_CSR(CSR_PMAADDR(IQ_USB_DIRECT_PMA_ENTRY), pma_addr);
  RV_WRITE_CSR(CSR_PMACFG(IQ_USB_DIRECT_PMA_ENTRY), pma_cfg);
  __asm__ __volatile__("fence rw, rw" ::: "memory");
  if (RV_READ_CSR(CSR_PMAADDR(IQ_USB_DIRECT_PMA_ENTRY)) != pma_addr ||
      RV_READ_CSR(CSR_PMACFG(IQ_USB_DIRECT_PMA_ENTRY)) != pma_cfg) {
    return false;
  }
  s_direct_pma_core_mask |= core_bit;
  ESP_LOGI(TAG, "direct slab %p is write-through on core %d", s_stream_slab,
           xPortGetCoreID());
  return true;
}

/* With the mux held: pick the next slot the idle endpoint should carry.
 * Returns the slot to submit, or -1. */
static int stream_next_submission_locked(void) {
  if (s_inflight_slot >= 0) {
    return -1;
  }
  const int parked = pending_pop_locked();
  if (parked >= 0) {
    s_inflight_slot = (int8_t)parked;
    return parked;
  }
  /* PSRAM slots must be cache-synced by the stream task before the
   * endpoint may read them, so only internal-memory fills are stolen. */
  if (!s_slots_psram && s_fill_slot >= 0 && s_fill_bytes > 0u &&
      !s_fill_writing) {
    s_inflight_slot = s_fill_slot;
    s_slot_bytes[s_inflight_slot] = (uint16_t)s_fill_bytes;
    s_fill_slot = -1;
    s_fill_bytes = 0u;
    return s_inflight_slot;
  }
  return -1;
}

static bool stream_submit_slot(uint8_t rhport, int slot, bool in_isr) {
  const uint32_t started = esp_cpu_get_cycle_count();
  s_slot_submit_cycle[slot] = started;
  const bool submitted = usbd_edpt_xfer(
      rhport, IQ_USB_EP_STREAM_IN, s_stream_slot[slot],
      s_slot_bytes[slot], in_isr);
  if (submitted) {
    ++s_diag_submissions;
    s_diag_submission_bytes += s_slot_bytes[slot];
  }
  return submitted;
}

/* Queue or submit the currently filling slot (stream-task context). */
static void stream_flush_fill_slot(void) {
  int submit = -1;
  taskENTER_CRITICAL(&s_usb_mux);
  if (s_fill_slot < 0 || s_fill_bytes == 0u) {
    taskEXIT_CRITICAL(&s_usb_mux);
    return;
  }
  const int slot = s_fill_slot;
  const uint32_t bytes = s_fill_bytes;
  s_slot_bytes[slot] = (uint16_t)bytes;
  s_fill_slot = -1;
  s_fill_bytes = 0u;
  if (s_slots_psram) {
    /* Write the cached batch back before the endpoint DMA reads it; the
     * sync cannot run inside the critical section. */
    taskEXIT_CRITICAL(&s_usb_mux);
    const uint32_t sync_start = esp_cpu_get_cycle_count();
    (void)esp_cache_msync(s_stream_slot[slot], (bytes + 63u) & ~63u,
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    s_diag_cache_sync_cycles +=
        (uint32_t)(esp_cpu_get_cycle_count() - sync_start);
    taskENTER_CRITICAL(&s_usb_mux);
  }
  if (s_inflight_slot >= 0) {
    /* Endpoint busy: park the batch; the completion ISR picks it up. */
    pending_push_locked((int8_t)slot);
  } else {
    s_inflight_slot = (int8_t)slot;
    submit = slot;
  }
  taskEXIT_CRITICAL(&s_usb_mux);
  if (submit >= 0) {
    if (!stream_submit_slot(s_rhport, submit, false)) {
      taskENTER_CRITICAL(&s_usb_mux);
      s_inflight_slot = -1;
      s_slot_busy[submit] = false;
      taskEXIT_CRITICAL(&s_usb_mux);
      xSemaphoreGive(s_slot_sem);
      ++s_usb_send_errors;
    }
  }
}

void iq_usb_flush(void) { stream_flush_fill_slot(); }

static void stream_write_transport_header(uint8_t *packet,
                                          const uint8_t *frame,
                                          size_t frame_bytes, uint32_t epoch,
                                          uint32_t sequence) {
  const bool iq_frame =
      (memcmp(frame, "IQC", 3u) == 0 &&
       (frame[3] == '1' || frame[3] == '8' || frame[3] == '4')) ||
      memcmp(frame, STREAM_FRAME_MAGIC_REAL8, 4u) == 0;
  uint32_t frame_sequence;
  memcpy(&frame_sequence, frame + 4u, sizeof(frame_sequence));
  uint32_t source_chunk = 0u;
  uint32_t dropped = s_callbacks.get_firmware_dropped_chunks();
  if (iq_frame && frame_bytes >= 40u) {
    memcpy(&source_chunk, frame + 8u, sizeof(source_chunk));
    memcpy(&dropped, frame + 36u, sizeof(dropped));
  }
  uint32_t frame_crc;
  if (iq_frame) {
    memcpy(&frame_crc, frame + frame_bytes - sizeof(frame_crc),
           sizeof(frame_crc));
  } else {
    frame_crc = esp_rom_crc32_le(0u, frame, frame_bytes);
  }

  iq_udp_header_t *header = (iq_udp_header_t *)packet;
  *header = (iq_udp_header_t){
      .magic = {'I', 'Q', 'U', '1'},
      .version = IQ_UDP_VERSION,
      .header_bytes = sizeof(*header),
      .stream_epoch = epoch,
      .datagram_sequence = sequence,
      .frame_sequence = frame_sequence,
      .source_chunk_index = source_chunk,
      .frame_bytes = frame_bytes,
      .frame_crc32 = frame_crc,
      .fragment_offset = 0u,
      .fragment_bytes = (uint16_t)frame_bytes,
      .fragment_index = 0u,
      .fragment_count = 1u,
      .firmware_dropped_chunks = dropped,
  };
  memcpy(header->frame_magic, frame, 4u);
  header->header_crc32 =
      esp_rom_crc32_le(0u, packet, offsetof(iq_udp_header_t, header_crc32));
}

/* The S31 capture backend deliberately retains full IQC1 words in its ring:
 * the source-side packed view is not valid while the modem owns the dump TCM.
 * Pack only at the USB transport boundary.  Writing the compact frame straight
 * into the endpoint batch avoids both an intermediate IQC8 buffer and the
 * previous 4 KiB memcpy, while reducing the high-speed USB wire rate from
 * about 64 MB/s to 32 MB/s at 16 MSa/s. */
static void stream_pack_iqc1_as_iqc8(uint8_t *output,
                                    const uint8_t *input) {
  memcpy(output, input, offsetof(iq_chunk_t, samples));
  output[3] = '8';
  const uint32_t *source = (const uint32_t *)(
      input + offsetof(iq_chunk_t, samples));
  uint8_t *packed = output + offsetof(iq_chunk_t, samples);
  bool scratch_has_current_frame = false;
  if (!s_pie_iq8_checked) {
    s31_pie_pack_iq8(s_iq8_scratch, source, IQ_CHUNK_SAMPLE_WORDS);
    scratch_has_current_frame = true;
    s_pie_iq8_valid = true;
    for (uint32_t sample = 0u; sample < IQ_CHUNK_SAMPLE_WORDS; ++sample) {
      const uint32_t word = source[sample];
      if (s_iq8_scratch[2u * sample] != ((word >> 12) & 0xffu) ||
          s_iq8_scratch[2u * sample + 1u] != ((word >> 2) & 0xffu)) {
        s_pie_iq8_valid = false;
        break;
      }
    }
    s_pie_iq8_checked = true;
  }
  if (s_pie_iq8_valid) {
    (void)scratch_has_current_frame;
    s31_pie_pack_iq8(packed, source, IQ_CHUNK_SAMPLE_WORDS);
  } else {
    uint32_t *packed_words = (uint32_t *)packed;
    for (uint32_t sample = 0u; sample < IQ_CHUNK_SAMPLE_WORDS; sample += 2u) {
      const uint32_t w0 = source[sample];
      const uint32_t w1 = source[sample + 1u];
      const uint32_t p0 =
          ((w0 >> 12) & 0xffu) | ((w0 << 6) & 0xff00u);
      const uint32_t p1 =
          ((w1 >> 12) & 0xffu) | ((w1 << 6) & 0xff00u);
      packed_words[sample / 2u] = p0 | (p1 << 16);
    }
  }
  memset(output + IQ8_FRAME_WIRE_BYTES - sizeof(uint32_t), 0,
         sizeof(uint32_t));
}

bool iq_usb_send_frame(const uint8_t *frame, size_t frame_bytes) {
  if (frame == NULL || frame_bytes < 8u ||
      frame_bytes > sizeof(stream_frame_t) || s_ep_stream_in == 0u ||
      !tud_mounted() || !stream_slots_allocate()) {
    return false;
  }
  uint32_t epoch;
  uint32_t sequence;
  if (!iq_network_stream_tx_ticket(IQ_STREAM_OWNER_USB, &epoch, &sequence)) {
    return false;
  }

  const uint32_t send_start = esp_cpu_get_cycle_count();
  const bool pack_iq8 =
      s_stream_format == IQ_USB_FORMAT_INT8 &&
      frame_bytes == sizeof(iq_chunk_t) &&
      memcmp(frame, STREAM_FRAME_MAGIC_IQ, 4u) == 0;
  const size_t wire_frame_bytes =
      pack_iq8 ? IQ8_FRAME_WIRE_BYTES : frame_bytes;
  const uint32_t message_bytes =
      sizeof(iq_udp_header_t) + wire_frame_bytes;
  uint8_t *packet_base = NULL;
  while (packet_base == NULL) {
    taskENTER_CRITICAL(&s_usb_mux);
    if (s_fill_slot >= 0 && s_slot_capacity - s_fill_bytes >= message_bytes) {
      s_fill_writing = true;
      packet_base = s_stream_slot[s_fill_slot] + s_fill_bytes;
      taskEXIT_CRITICAL(&s_usb_mux);
      break;
    }
    taskEXIT_CRITICAL(&s_usb_mux);
    if (s_fill_slot >= 0) {
      /* Full batch: park it for the endpoint. */
      stream_flush_fill_slot();
    }
    if (s_fill_slot < 0) {
      const uint32_t wait_start = esp_cpu_get_cycle_count();
      if (xSemaphoreTake(s_slot_sem, pdMS_TO_TICKS(IQ_USB_SLOT_TIMEOUT_MS)) ==
          pdFALSE) {
        ++s_usb_send_errors;
        return false;
      }
      s_diag_wait_cycles += (uint32_t)(esp_cpu_get_cycle_count() - wait_start);
      taskENTER_CRITICAL(&s_usb_mux);
      for (uint32_t i = 0u; i < s_slot_count; ++i) {
        if (!s_slot_busy[i]) {
          s_slot_busy[i] = true;
          s_fill_slot = (int8_t)i;
          s_fill_bytes = 0u;
          break;
        }
      }
      taskEXIT_CRITICAL(&s_usb_mux);
      if (s_fill_slot < 0) {
        /* Cannot happen while the semaphore accounts slots, but stay safe. */
        ++s_usb_send_errors;
        return false;
      }
    }
  }

  uint8_t *packet = packet_base;
  uint8_t *wire_frame = packet + sizeof(iq_udp_header_t);
  const uint32_t pack_start = esp_cpu_get_cycle_count();
  if (pack_iq8) {
    stream_pack_iqc1_as_iqc8(wire_frame, frame);
  } else {
    memcpy(wire_frame, frame, frame_bytes);
  }
  s_diag_pack_cycles +=
      (uint32_t)(esp_cpu_get_cycle_count() - pack_start);
  const uint32_t header_start = esp_cpu_get_cycle_count();
  stream_write_transport_header(packet, wire_frame, wire_frame_bytes, epoch,
                                sequence);
  s_diag_header_cycles +=
      (uint32_t)(esp_cpu_get_cycle_count() - header_start);
  int submit = -1;
  bool flush_now = false;
  taskENTER_CRITICAL(&s_usb_mux);
  s_fill_bytes += message_bytes;
  s_fill_writing = false;
  /* If the endpoint is idle, ship what we have immediately; otherwise the
   * batch keeps growing until the completion ISR steals it (internal
   * memory) or, in PSRAM mode, until it is parked — at half capacity when
   * nothing else is queued, so the pipeline never starves. */
  if (s_slots_psram) {
    flush_now = s_pending_count == 0u &&
                (s_inflight_slot < 0 || s_fill_bytes >= s_slot_capacity / 2u);
  } else {
    submit = stream_next_submission_locked();
  }
  taskEXIT_CRITICAL(&s_usb_mux);
  if (flush_now) {
    stream_flush_fill_slot();
  }
  if (submit >= 0) {
    if (!stream_submit_slot(s_rhport, submit, false)) {
      taskENTER_CRITICAL(&s_usb_mux);
      s_inflight_slot = -1;
      s_slot_busy[submit] = false;
      taskEXIT_CRITICAL(&s_usb_mux);
      xSemaphoreGive(s_slot_sem);
      ++s_usb_send_errors;
      return false;
    }
  }
  ++s_usb_frames;
  s_diag_send_cycles += (uint32_t)(esp_cpu_get_cycle_count() - send_start);
  return true;
}

static bool iq_usb_direct_reserve_format(uint32_t count,
                                         stream_frame_t **frames,
                                         uint32_t stream_format,
                                         uint32_t frame_bytes,
                                         uint32_t message_bytes,
                                         bool coherent_writes) {
  if (count == 0u || count > IQ_USB_DIRECT_MAX_FRAMES || frames == NULL ||
      s_stream_format != stream_format || s_ep_stream_in == 0u ||
      !tud_mounted() || !stream_slots_allocate() || !s_slots_psram ||
      (coherent_writes && !stream_direct_enable_coherent_slab()) ||
      message_bytes > IQ_USB_MESSAGE_MAX_BYTES ||
      frame_bytes + sizeof(iq_udp_header_t) != message_bytes) {
    return false;
  }

  const uint32_t needed_slots =
      (count + IQ_USB_DIRECT_BATCH_FRAMES - 1u) /
      IQ_USB_DIRECT_BATCH_FRAMES;
  uint32_t tokens = 0u;
  while (tokens < needed_slots &&
         xSemaphoreTake(s_slot_sem, 0) == pdTRUE) {
    ++tokens;
  }
  if (tokens != needed_slots) {
    while (tokens > 0u) {
      xSemaphoreGive(s_slot_sem);
      --tokens;
    }
    return false;
  }

  bool reserved = false;
  taskENTER_CRITICAL(&s_usb_mux);
  if (s_direct_fill_count == 0u) {
    for (uint32_t i = 0u; i < s_slot_count &&
                         s_direct_fill_count < needed_slots; ++i) {
      if (!s_slot_busy[i]) {
        s_slot_busy[i] = true;
        s_direct_fill_slots[s_direct_fill_count++] = (int8_t)i;
      }
    }
    reserved = s_direct_fill_count == needed_slots;
    if (reserved) {
      s_direct_frame_bytes = frame_bytes;
      s_direct_message_bytes = message_bytes;
      s_direct_cache_sync = !coherent_writes;
    }
    if (!reserved) {
      for (uint32_t i = 0u; i < s_direct_fill_count; ++i) {
        s_slot_busy[s_direct_fill_slots[i]] = false;
        s_direct_fill_slots[i] = -1;
      }
      s_direct_fill_count = 0u;
      s_direct_frame_bytes = 0u;
      s_direct_message_bytes = 0u;
      s_direct_cache_sync = false;
    }
  }
  taskEXIT_CRITICAL(&s_usb_mux);
  if (!reserved) {
    for (uint32_t i = 0u; i < needed_slots; ++i) {
      xSemaphoreGive(s_slot_sem);
    }
    return false;
  }

  for (uint32_t i = 0u; i < count; ++i) {
    const uint32_t batch = i / IQ_USB_DIRECT_BATCH_FRAMES;
    const uint32_t within = i % IQ_USB_DIRECT_BATCH_FRAMES;
    const int slot = s_direct_fill_slots[batch];
    frames[i] = (stream_frame_t *)(
        s_stream_slot[slot] + within * message_bytes +
        sizeof(iq_udp_header_t));
    uint32_t epoch;
    uint32_t sequence;
    if (!iq_network_stream_tx_ticket(IQ_STREAM_OWNER_USB, &epoch, &sequence)) {
      iq_usb_direct_abort();
      return false;
    }
    iq_udp_header_t *header = (iq_udp_header_t *)(
        s_stream_slot[slot] + within * message_bytes);
    *header = (iq_udp_header_t){
        .magic = {'I', 'Q', 'U', '1'},
        .version = IQ_UDP_VERSION,
        .header_bytes = sizeof(*header),
        .stream_epoch = epoch,
        .datagram_sequence = sequence,
        .frame_bytes = frame_bytes,
        .fragment_bytes = (uint16_t)frame_bytes,
        .fragment_count = 1u,
    };
  }
  return true;
}

bool iq_usb_direct_reserve(uint32_t count, stream_frame_t **frames) {
  return iq_usb_direct_reserve_format(
      count, frames, IQ_USB_FORMAT_FULL, sizeof(iq_chunk_t),
      IQ_USB_MESSAGE_MAX_BYTES, true);
}

bool iq_usb_direct_reserve_iq8(uint32_t count, stream_frame_t **frames) {
  return iq_usb_direct_reserve_format(
      count, frames, IQ_USB_FORMAT_INT8, IQ8_FRAME_WIRE_BYTES,
      IQ_USB_IQ8_MESSAGE_BYTES, false);
}

bool iq_usb_direct_reserve_iq4(uint32_t count, stream_frame_t **frames) {
  return iq_usb_direct_reserve_format(
      count, frames, IQ_USB_FORMAT_INT4, IQ4_FRAME_WIRE_BYTES,
      IQ_USB_IQ4_MESSAGE_BYTES, false);
}

void iq_usb_direct_abort(void) {
  int slots[IQ_USB_DIRECT_MAX_SLOTS] = {-1, -1};
  uint32_t count;
  taskENTER_CRITICAL(&s_usb_mux);
  count = s_direct_fill_count;
  s_direct_fill_count = 0u;
  s_direct_frame_bytes = 0u;
  s_direct_message_bytes = 0u;
  s_direct_cache_sync = false;
  for (uint32_t i = 0u; i < count; ++i) {
    slots[i] = s_direct_fill_slots[i];
    s_direct_fill_slots[i] = -1;
    if (slots[i] >= 0) {
      s_slot_busy[slots[i]] = false;
    }
  }
  taskEXIT_CRITICAL(&s_usb_mux);
  for (uint32_t i = 0u; i < count; ++i) {
    xSemaphoreGive(s_slot_sem);
  }
}

bool iq_usb_direct_commit(stream_frame_t **frames, uint32_t count) {
  if (frames == NULL || count == 0u || count > IQ_USB_DIRECT_MAX_FRAMES) {
    iq_usb_direct_abort();
    return false;
  }

  const uint32_t batch_count =
      (count + IQ_USB_DIRECT_BATCH_FRAMES - 1u) /
      IQ_USB_DIRECT_BATCH_FRAMES;
  int slots[IQ_USB_DIRECT_MAX_SLOTS] = {-1, -1};
  uint32_t frame_bytes;
  uint32_t message_bytes;
  bool cache_sync;
  taskENTER_CRITICAL(&s_usb_mux);
  frame_bytes = s_direct_frame_bytes;
  message_bytes = s_direct_message_bytes;
  cache_sync = s_direct_cache_sync;
  const bool owned = s_direct_fill_count == batch_count &&
                     frame_bytes >= 8u &&
                     message_bytes == sizeof(iq_udp_header_t) + frame_bytes;
  for (uint32_t i = 0u; owned && i < batch_count; ++i) {
    slots[i] = s_direct_fill_slots[i];
  }
  taskEXIT_CRITICAL(&s_usb_mux);
  if (!owned) {
    iq_usb_direct_abort();
    return false;
  }

  for (uint32_t i = 0u; i < count; ++i) {
    const uint32_t batch = i / IQ_USB_DIRECT_BATCH_FRAMES;
    const uint32_t within = i % IQ_USB_DIRECT_BATCH_FRAMES;
    uint8_t *const base = s_stream_slot[slots[batch]];
    uint8_t *const expected_frame =
        base + within * message_bytes + sizeof(iq_udp_header_t);
    if ((uint8_t *)frames[i] != expected_frame) {
      iq_usb_direct_abort();
      return false;
    }
    iq_udp_header_t *header =
        (iq_udp_header_t *)(expected_frame - sizeof(iq_udp_header_t));
    const uint32_t epoch = header->stream_epoch;
    const uint32_t sequence = header->datagram_sequence;
    stream_write_transport_header((uint8_t *)header, expected_frame,
                                  frame_bytes, epoch, sequence);
  }

  /* The ordinary PARLIO producer runs with caches available.  Keep its slab
   * cached while filling, then write each complete endpoint transfer back in
   * one operation.  Making every small producer store write-through costs
   * substantially more fabric bandwidth than this batch sync.  Full-format
   * staged capture still requests write-through because its modem/TCM
   * ownership window can make cache maintenance unavailable. */
  if (cache_sync) {
    for (uint32_t batch = 0u; batch < batch_count; ++batch) {
      uint32_t batch_frames = count - batch * IQ_USB_DIRECT_BATCH_FRAMES;
      if (batch_frames > IQ_USB_DIRECT_BATCH_FRAMES) {
        batch_frames = IQ_USB_DIRECT_BATCH_FRAMES;
      }
      const uint32_t bytes = batch_frames * message_bytes;
      (void)esp_cache_msync(s_stream_slot[slots[batch]],
                            (bytes + 63u) & ~63u,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
  }

  int submit = -1;
  taskENTER_CRITICAL(&s_usb_mux);
  if (s_direct_fill_count != batch_count ||
      s_direct_frame_bytes != frame_bytes ||
      s_direct_message_bytes != message_bytes ||
      s_direct_cache_sync != cache_sync) {
    taskEXIT_CRITICAL(&s_usb_mux);
    iq_usb_direct_abort();
    return false;
  }
  s_direct_fill_count = 0u;
  s_direct_frame_bytes = 0u;
  s_direct_message_bytes = 0u;
  s_direct_cache_sync = false;
  __asm__ __volatile__("fence" ::: "memory");
  for (uint32_t batch = 0u; batch < batch_count; ++batch) {
    const int slot = slots[batch];
    s_direct_fill_slots[batch] = -1;
    uint32_t batch_frames = count - batch * IQ_USB_DIRECT_BATCH_FRAMES;
    if (batch_frames > IQ_USB_DIRECT_BATCH_FRAMES) {
      batch_frames = IQ_USB_DIRECT_BATCH_FRAMES;
    }
    s_slot_bytes[slot] = (uint16_t)(batch_frames * message_bytes);
    if (s_inflight_slot >= 0) {
      pending_push_locked((int8_t)slot);
    } else {
      s_inflight_slot = (int8_t)slot;
      submit = slot;
    }
  }
  taskEXIT_CRITICAL(&s_usb_mux);

  if (submit >= 0 &&
      !stream_submit_slot(s_rhport, submit, false)) {
    stream_reset_slots();
    ++s_usb_send_errors;
    return false;
  }
  s_usb_frames += count;
  return true;
}

/* ---------------- Custom class driver ------------------------------------ */
static void drv_init(void) {}

static bool drv_deinit(void) { return true; }

static void drv_reset(uint8_t rhport) {
  (void)rhport;
  s_ep_ctrl_out = 0u;
  s_ep_ctrl_in = 0u;
  s_ep_stream_in = 0u;
  s_ep_tx_out = 0u;
  tx_upload_forget(true);
  if (iq_network_stream_owner() == IQ_STREAM_OWNER_USB) {
    iq_network_stream_end();
  }
  stream_reset_slots();
}

static uint16_t drv_open(uint8_t rhport, const tusb_desc_interface_t *desc_itf,
                         uint16_t max_len) {
  TU_VERIFY(desc_itf->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC, 0);
  (void)max_len;
  s_rhport = rhport;

  uint16_t drv_len = tu_desc_len((const uint8_t *)desc_itf);
  const uint8_t *p_desc = tu_desc_next(desc_itf);
  for (int found = 0; found < desc_itf->bNumEndpoints;
       p_desc = tu_desc_next(p_desc)) {
    if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
      const tusb_desc_endpoint_t *desc_ep =
          (const tusb_desc_endpoint_t *)p_desc;
      TU_ASSERT(usbd_edpt_open(rhport, desc_ep), 0);
      switch (desc_ep->bEndpointAddress) {
      case IQ_USB_EP_CTRL_OUT:
        s_ep_ctrl_out = desc_ep->bEndpointAddress;
        break;
      case IQ_USB_EP_CTRL_IN:
        s_ep_ctrl_in = desc_ep->bEndpointAddress;
        break;
      case IQ_USB_EP_STREAM_IN:
        s_ep_stream_in = desc_ep->bEndpointAddress;
        break;
      case IQ_USB_EP_TX_OUT:
        s_ep_tx_out = desc_ep->bEndpointAddress;
        break;
      default:
        break;
      }
      ++found;
    }
    drv_len += tu_desc_len(p_desc);
  }

  if (s_ep_ctrl_out != 0u) {
    ctrl_prepare_request_read();
  }
  return drv_len;
}

static bool drv_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                const tusb_control_request_t *request) {
  (void)rhport;
  (void)stage;
  (void)request;
  return true;
}

/* Stream completions run in the ISR fast path; control endpoints defer to
 * the TinyUSB task (drv_xfer_cb). */
static bool drv_xfer_isr(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
                         uint32_t xferred_bytes) {
  (void)xferred_bytes;
  if (ep_addr != IQ_USB_EP_STREAM_IN) {
    return false; /* defer to drv_xfer_cb */
  }
  if (result == XFER_RESULT_SUCCESS) {
    int freed = -1;
    int submit = -1;
    taskENTER_CRITICAL_ISR(&s_usb_mux);
    freed = s_inflight_slot;
    s_inflight_slot = -1;
    if (freed >= 0) {
      const uint32_t completion_cycles =
          esp_cpu_get_cycle_count() - s_slot_submit_cycle[freed];
      s_diag_completion_cycles += completion_cycles;
      ++s_diag_completions;
      s_diag_completion_bytes += s_slot_bytes[freed];
      if (completion_cycles > s_diag_max_completion_cycles) {
        s_diag_max_completion_cycles = completion_cycles;
      }
      s_slot_busy[freed] = false;
    }
    /* Prefer a parked batch, otherwise steal the partial fill so the
     * endpoint never idles while data is waiting. */
    submit = stream_next_submission_locked();
    taskEXIT_CRITICAL_ISR(&s_usb_mux);
    if (submit >= 0) {
      (void)stream_submit_slot(rhport, submit, true);
    }
    if (freed >= 0) {
      BaseType_t woken = pdFALSE;
      xSemaphoreGiveFromISR(s_slot_sem, &woken);
      portYIELD_FROM_ISR(woken);
    }
  }
  return true;
}

static bool drv_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
                        uint32_t xferred_bytes) {
  (void)rhport;
  if (ep_addr == IQ_USB_EP_TX_OUT) {
    if (result != XFER_RESULT_SUCCESS || s_tx_upload == NULL ||
        xferred_bytes == 0u || xferred_bytes > s_tx_armed_bytes) {
      ++s_usb_tx_errors;
      tx_upload_forget(true);
      return true;
    }
    uint8_t *completed =
        (uint8_t *)s_tx_upload + s_tx_received_bytes;
    /* DWC2 writes PSRAM behind the data cache. M2C invalidation requires a
     * cache-line-aligned span; tx_upload_begin() pads the allocation so the
     * final short transfer can safely round up here. */
    (void)esp_cache_msync(
        completed, (xferred_bytes + 63u) & ~63u,
        ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    s_tx_received_bytes += xferred_bytes;
    s_tx_armed_bytes = 0u;
    if (!tx_upload_arm_next()) {
      ++s_usb_tx_errors;
      tx_upload_forget(true);
    }
    return true;
  }
  if (result != XFER_RESULT_SUCCESS) {
    return true;
  }
  if (ep_addr == IQ_USB_EP_CTRL_OUT) {
    ctrl_process_request(xferred_bytes);
  } else if (ep_addr == IQ_USB_EP_CTRL_IN) {
    /* Response delivered: accept the next request. */
    ctrl_prepare_request_read();
  }
  return true;
}

static const usbd_class_driver_t s_iq_usb_driver = {
    .name = "IQSDR",
    .init = drv_init,
    .deinit = drv_deinit,
    .reset = drv_reset,
    .open = drv_open,
    .control_xfer_cb = drv_control_xfer_cb,
    .xfer_cb = drv_xfer_cb,
    .xfer_isr = drv_xfer_isr,
    .sof = NULL,
};

const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return &s_iq_usb_driver;
}

/* ---------------- Public API --------------------------------------------- */
uint32_t iq_usb_frames(void) { return s_usb_frames; }

uint32_t iq_usb_send_errors(void) { return s_usb_send_errors; }

uint32_t iq_usb_stream_format(void) { return s_stream_format; }

void iq_usb_stream_diag(iq_usb_stream_diag_t *diag) {
  *diag = (iq_usb_stream_diag_t){
      .send_cycles = s_diag_send_cycles,
      .wait_cycles = s_diag_wait_cycles,
      .pack_cycles = s_diag_pack_cycles,
      .header_cycles = s_diag_header_cycles,
      .cache_sync_cycles = s_diag_cache_sync_cycles,
      .completion_cycles = s_diag_completion_cycles,
      .submissions = s_diag_submissions,
      .submission_bytes = s_diag_submission_bytes,
      .completions = s_diag_completions,
      .completion_bytes = s_diag_completion_bytes,
      .maximum_completion_cycles = s_diag_max_completion_cycles,
  };
}

void iq_usb_tx_diag(uint32_t *uploads, uint32_t *bytes, uint32_t *errors,
                    uint32_t *backpressure_retries,
                    uint32_t *commit_rejections,
                    uint32_t *received_words, bool *active) {
  *uploads = s_usb_tx_uploads;
  *bytes = s_usb_tx_bytes;
  *errors = s_usb_tx_errors;
  *backpressure_retries = s_usb_tx_backpressure_retries;
  *commit_rejections = s_usb_tx_commit_rejections;
  *received_words = (s_tx_commit_flags & IQ_TX_USB_FLAG_PACKED20) != 0u
                        ? (s_tx_received_bytes * 2u) / 5u
                        : s_tx_received_bytes / sizeof(uint32_t);
  *active = s_tx_upload != NULL;
}

bool iq_usb_mounted(void) { return tud_inited() && tud_mounted(); }

static void usb_device_event(tinyusb_event_t *event, void *arg) {
  (void)arg;
  const char *name = "unknown";
  if (event->id == TINYUSB_EVENT_ATTACHED) {
    name = "configured";
  } else if (event->id == TINYUSB_EVENT_DETACHED) {
    name = "detached";
  }
  ESP_LOGI(TAG, "USB device event: %s (port %u)", name, event->rhport);
}

/* Keep this short diagnostic in the production image: it is invaluable when
 * distinguishing a missing native-USB cable/port from a descriptor or stream
 * failure, and costs only one small temporary task during boot. */
static void usb_boot_diag_task(void *arg) {
  (void)arg;
  /* Some early HS-PHY/controller combinations do not expose the reset-time
   * soft-disconnect interval to an already-connected host.  Give the host an
   * unambiguous 100 ms detach pulse after the stack is fully alive. */
  vTaskDelay(pdMS_TO_TICKS(250));
  ESP_LOGI(TAG, "issuing post-boot USB detach/attach pulse");
  tud_disconnect();
  vTaskDelay(pdMS_TO_TICKS(100));
  tud_connect();

  static const uint32_t delays_ms[] = {100u, 650u, 4000u};
  for (size_t i = 0; i < sizeof(delays_ms) / sizeof(delays_ms[0]); ++i) {
    vTaskDelay(pdMS_TO_TICKS(delays_ms[i]));
    ESP_LOGI(TAG,
             "HS state: mounted=%u gotg=%08" PRIx32 " gint=%08" PRIx32
             " mask=%08" PRIx32 " dcfg=%08" PRIx32 " dctl=%08" PRIx32
             " dsts=%08" PRIx32 " pcgc=%08" PRIx32,
             tud_mounted(), USB_OTGHS.gotgctl_reg.val,
             USB_OTGHS.gintsts_reg.val, USB_OTGHS.gintmsk_reg.val,
             USB_OTGHS.dcfg_reg.val, USB_OTGHS.dctl_reg.val,
             USB_OTGHS.dsts_reg.val, USB_OTGHS.pcgcctl_reg.val);
    ESP_LOGI(TAG,
             "HS clocks: hp=%08" PRIx32 " cnnt=%08" PRIx32
             " alive=%08" PRIx32 " utmi06=%08" PRIx32,
             HP_SYS_CLKRST.usb_otghs_ctrl0.val,
             CNNT_SYS_REG.sys_usb_otg20_ctrl.val,
             HP_ALIVE_SYS.usb_otghs_ctrl.val,
             USB_UTMI.fc_06.val);
  }
  vTaskDelete(NULL);
}

void iq_usb_init(const iq_network_callbacks_t *callbacks) {
  s_callbacks = *callbacks;
  s_slot_sem = xSemaphoreCreateCounting(IQ_USB_SLOT_MAX, IQ_USB_SLOT_MAX);
  ESP_ERROR_CHECK(s_slot_sem != NULL ? ESP_OK : ESP_ERR_NO_MEM);

  uint8_t mac[6] = {0};
  (void)esp_read_mac(mac, ESP_MAC_BASE);
  snprintf(s_serial, sizeof(s_serial), "%02x%02x%02x%02x%02x%02x", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Double-buffer the stream IN endpoint FIFO in the DWC2 core; with a
   * single-packet FIFO the core stalls after every 512-byte packet waiting
   * for the DMA refill, halving IN throughput. Must run before stack init. */
  tud_configure_dwc2_t dwc2_cfg = CFG_TUD_CONFIGURE_DWC2_DEFAULT;
  dwc2_cfg.bm_double_buffered = 1u << IQ_USB_STREAM_EP_NUM;
  tud_configure(0, TUD_CFGID_DWC2, &dwc2_cfg);

  tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(usb_device_event);
  tusb_cfg.descriptor.device = &s_device_desc;
  tusb_cfg.descriptor.full_speed_config = s_fs_config_desc;
  tusb_cfg.descriptor.high_speed_config = s_hs_config_desc;
  tusb_cfg.descriptor.string = s_string_desc;
  tusb_cfg.descriptor.string_count =
      sizeof(s_string_desc) / sizeof(s_string_desc[0]);
  /* Keep USB off the capture core; above the HTTP server, below the
   * IQ producer. */
  tusb_cfg.task.xCoreID = 0;
  tusb_cfg.task.priority = 23;
  tusb_cfg.task.size = 6144;
  esp_err_t err = tinyusb_driver_install(&tusb_cfg);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "USB unavailable (%s); continuing without USB transport",
             esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "USB transport ready (serial %s)", s_serial);
  /* This temporary logging task overlaps DHCP and HTTP-server creation.
   * Keep its non-realtime stack in PSRAM so it cannot consume the last
   * contiguous internal block needed by Ethernet control startup. */
  BaseType_t task_ok = xTaskCreatePinnedToCoreWithCaps(
      usb_boot_diag_task, "usb_boot_diag", 3072, NULL, 4, NULL, 0,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (task_ok != pdPASS) {
    ESP_LOGW(TAG, "could not start USB boot diagnostic task");
  }
}
