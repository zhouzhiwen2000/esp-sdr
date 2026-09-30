#include "iq_network.h"
#include "gaintable.h"
#include "iq_usb.h"
#include "ringbuffer.h"
#include "sdr_agc.h"

#include <inttypes.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_private/eth_mac_esp_dma.h"
#include "esp_random.h"
#include "esp_rom_crc.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/dma_types.h"
#include "hal/emac_hal.h"
#include "lwip/etharp.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#include "lwip/tcpip.h"
#include "mdns.h"

#define HTTP_BODY_MAX_BYTES 4096u
#define TX_WAVEFORM_MAX_BYTES (TX_BATCH_WORDS_MAX * sizeof(uint32_t))
#define UDP_PACKET_BYTES                                                       \
  (sizeof(iq_udp_header_t) + IQ_UDP_FRAGMENT_PAYLOAD_BYTES)
#define ETH_HEADER_BYTES 14u
#define IPV4_HEADER_BYTES 20u
#define UDP_HEADER_BYTES 8u
#define TX_FRAME_BYTES                                                        \
  (ETH_HEADER_BYTES + IPV4_HEADER_BYTES + UDP_HEADER_BYTES + UDP_PACKET_BYTES)
#define STREAM_ARM_DELAY_US 100000u
#define IQ_ETH_TX_MUTEX_TIMEOUT_MS 250u
#define TX_UDP_ARM_LIFETIME_US 15000000ll
#define TX_UDP_WORDS_PER_DATAGRAM                                          \
  (IQ_UDP_FRAGMENT_PAYLOAD_BYTES / sizeof(uint32_t))
#define TX_UDP_PACKED20_WORDS_PER_DATAGRAM                                     \
  (IQ_UDP_FRAGMENT_PAYLOAD_BYTES * 2u / 5u)
#define TX_UDP_PACKED16_WORDS_PER_DATAGRAM                                     \
  (IQ_UDP_FRAGMENT_PAYLOAD_BYTES / 2u)
#define TX_UDP_DATAGRAMS_MAX                                               \
  ((TX_BATCH_WORDS_MAX + TX_UDP_WORDS_PER_DATAGRAM - 1u) /                 \
   TX_UDP_WORDS_PER_DATAGRAM)

static const char *TAG = "iq_network";
static iq_network_callbacks_t s_callbacks;
static esp_eth_handle_t s_eth_handle;
static esp_netif_t *s_eth_netif;
static httpd_handle_t s_http_server;
static esp_timer_handle_t s_stream_arm_timer;
static portMUX_TYPE s_stream_mux = portMUX_INITIALIZER_UNLOCKED;
static struct sockaddr_in s_stream_destination;
static uint8_t s_stream_destination_mac[6];
static uint8_t s_source_mac[6];
static volatile bool s_stream_armed;
static volatile iq_stream_owner_t s_stream_owner;
static volatile uint32_t s_stream_format;

static void stream_arm_timer_callback(void *arg) {
  (void)arg;
  taskENTER_CRITICAL(&s_stream_mux);
  s_stream_armed = true;
  taskEXIT_CRITICAL(&s_stream_mux);
}

static volatile bool s_link_up;
static volatile bool s_has_ipv4;
static bool s_ethernet_suspended_for_usb_tx;
static volatile bool s_ethernet_resume_pending;
static TaskHandle_t s_ethernet_resume_task_handle;
static uint32_t s_ipv4_address;
static uint32_t s_stream_epoch;
static uint32_t s_datagram_sequence;
static volatile uint32_t s_udp_frames;
static volatile uint32_t s_udp_datagrams;
static volatile uint32_t s_udp_bytes;
static volatile uint32_t s_udp_send_errors;
static volatile uint32_t s_eth_tx_desc_completed;
static volatile uint32_t s_eth_tx_desc_errors;
static volatile uint32_t s_eth_tx_desc_underflows;
static volatile uint32_t s_eth_tx_desc_flushed;
static volatile uint32_t s_eth_tx_desc_carrier_errors;
static volatile uint32_t s_eth_dma_status;
static volatile uint32_t s_tx_udp_datagrams;
static volatile uint32_t s_tx_udp_bytes;
static volatile uint32_t s_tx_udp_errors;
static volatile uint32_t s_tx_udp_stale_datagrams;
static volatile uint32_t s_tx_udp_backpressure_retries;
static volatile uint32_t s_tx_udp_commit_rejections;
static volatile uint32_t s_tx_udp_batch_id;
static volatile uint32_t s_tx_udp_received_words;
static volatile uint32_t s_tx_udp_committed_words;
static volatile uint32_t s_tx_udp_commits;
static portMUX_TYPE s_tx_udp_arm_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_tx_udp_allowed_ipv4;
static uint32_t s_tx_udp_session_token;
static int64_t s_tx_udp_arm_expires_us;
static uint32_t *s_tx_udp_batch;
static uint32_t s_tx_udp_batch_capacity_words;
static uint8_t s_tx_udp_batch_storage_bits;
static uint32_t s_tx_udp_session_max_words;
static bool s_tx_udp_session_packed20;
static bool s_tx_udp_session_packed16;
static uint32_t s_tx_udp_expected_sequence;
static uint32_t s_tx_udp_final_sequence;
static uint32_t s_tx_udp_total_words;
static uint32_t s_tx_udp_commit_token;
static uint16_t s_tx_udp_commit_flags;
static uint64_t s_tx_udp_start_time_ns;
static bool s_tx_udp_commit_pending;
static bool s_tx_udp_packed20;
static bool s_tx_udp_packed16;
static bool s_tx_udp_expanded_compact;
static uint32_t s_tx_udp_compact_base_bytes;
static volatile uint32_t s_tx_udp_unpack_max_cycles;
static uint32_t s_tx_udp_received_bitmap[(TX_UDP_DATAGRAMS_MAX + 31u) / 32u];
/* IDF normally allocates and copies one frame per RX-task callback.  Drain
 * queued, single-descriptor TX-IQ frames directly into the ordered PSRAM
 * batch; ordinary Ethernet traffic retains the stock ownership contract. */
static volatile uint32_t s_tx_udp_direct_frames;
static volatile uint32_t s_tx_udp_direct_max_cycles;
static volatile uint64_t s_tx_udp_direct_total_cycles;
/* The stream task is the sole writer. The GMAC driver copies a frame into its
 * DMA ring before returning, so this staging frame can be reused immediately. */
static uint8_t s_tx_frame[TX_FRAME_BYTES];
static uint8_t s_iq8_frame[IQ8_FRAME_WIRE_BYTES] __attribute__((aligned(4)));

/* Pinned-IDF driver prefix used only to hold the public transmit mutex across
 * all three UDP fragments of one IQ frame.  Calling the MAC once per fragment
 * while taking the mutex only once avoids six scheduler/semaphore operations
 * per 1024 IQ samples, while lwIP control traffic remains serialized. */
typedef struct {
  esp_eth_mediator_t mediator;
  esp_eth_phy_t *phy;
  esp_eth_mac_t *mac;
  esp_timer_handle_t check_link_timer;
  uint32_t check_link_period_ms;
  bool auto_nego_en;
  eth_speed_t speed;
  eth_duplex_t duplex;
  _Atomic eth_link_t link;
  atomic_int ref_count;
  void *priv;
  _Atomic int fsm;
  SemaphoreHandle_t transmit_mutex;
} iq_eth_driver_prefix_t;

typedef struct {
  emac_hal_context_t hal;
  uint32_t tx_desc_flags;
  uint32_t rx_desc_flags;
  void *descriptors;
  eth_dma_rx_descriptor_t *rx_desc;
  eth_dma_tx_descriptor_t *tx_desc;
  uint8_t *rx_buf[CONFIG_ETH_DMA_RX_BUFFER_NUM];
  uint8_t *tx_buf[CONFIG_ETH_DMA_TX_BUFFER_NUM];
} iq_emac_dma_prefix_t;

typedef struct {
  esp_eth_mac_t parent;
  esp_eth_mediator_t *eth;
  emac_hal_context_t hal;
  intr_handle_t intr_hdl;
  TaskHandle_t rx_task_hdl;
  iq_emac_dma_prefix_t *emac_dma_hndl;
} iq_emac_mac_prefix_t;

#define IQ_EMAC_TDES0_FS_FLAGS_MASK 0x0fcc0000u
#define IQ_EMAC_TDES0_LS_FLAGS_MASK 0x40000000u

static uint16_t ipv4_checksum(const uint8_t *header) {
  uint32_t sum = 0u;
  for (uint32_t i = 0u; i < IPV4_HEADER_BYTES; i += 2u)
    sum += ((uint16_t)header[i] << 8) | header[i + 1u];
  while (sum >> 16)
    sum = (sum & 0xffffu) + (sum >> 16);
  return (uint16_t)~sum;
}

static bool resolve_peer_mac(struct in_addr peer, uint8_t mac[6]) {
  ip4_addr_t peer_ip;
  ip4_addr_set_u32(&peer_ip, peer.s_addr);
  struct eth_addr *eth_addr = NULL;
  const ip4_addr_t *cached_ip = NULL;
  struct netif *netif = esp_netif_get_netif_impl(s_eth_netif);
  LOCK_TCPIP_CORE();
  int index = etharp_find_addr(netif, &peer_ip, &eth_addr, &cached_ip);
  if (index >= 0 && eth_addr != NULL)
    memcpy(mac, eth_addr->addr, 6u);
  UNLOCK_TCPIP_CORE();
  return index >= 0 && eth_addr != NULL;
}

static void build_udp_frame(const struct sockaddr_in *destination,
                            const uint8_t destination_mac[6],
                            size_t udp_payload_bytes,
                            uint16_t identification) {
  memcpy(s_tx_frame, destination_mac, 6u);
  memcpy(s_tx_frame + 6u, s_source_mac, 6u);
  s_tx_frame[12] = 0x08u;
  s_tx_frame[13] = 0x00u;

  uint8_t *ip = s_tx_frame + ETH_HEADER_BYTES;
  memset(ip, 0, IPV4_HEADER_BYTES + UDP_HEADER_BYTES);
  ip[0] = 0x45u;
  uint16_t ip_bytes =
      htons((uint16_t)(IPV4_HEADER_BYTES + UDP_HEADER_BYTES + udp_payload_bytes));
  memcpy(ip + 2u, &ip_bytes, sizeof(ip_bytes));
  uint16_t ip_id = htons(identification);
  memcpy(ip + 4u, &ip_id, sizeof(ip_id));
  ip[6] = 0x40u;
  ip[8] = 64u;
  ip[9] = IPPROTO_UDP;
  memcpy(ip + 12u, &s_ipv4_address, 4u);
  memcpy(ip + 16u, &destination->sin_addr.s_addr, 4u);
  uint16_t checksum = htons(ipv4_checksum(ip));
  memcpy(ip + 10u, &checksum, sizeof(checksum));

  uint8_t *udp = ip + IPV4_HEADER_BYTES;
  uint16_t source_port = htons(IQ_NETWORK_UDP_DEFAULT_PORT);
  uint16_t udp_bytes = htons((uint16_t)(UDP_HEADER_BYTES + udp_payload_bytes));
  memcpy(udp, &source_port, sizeof(source_port));
  memcpy(udp + 2u, &destination->sin_port, sizeof(destination->sin_port));
  memcpy(udp + 4u, &udp_bytes, sizeof(udp_bytes));
}

static esp_err_t yt8531_init(esp_eth_handle_t eth_handle) {
  bool autoneg = true;
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_S_AUTONEGO, &autoneg),
                      TAG, "enable YT8531 auto-negotiation");

  uint32_t value = 0xa001u;
  esp_eth_phy_reg_rw_data_t reg = {.reg_addr = 0x1eu, .reg_value_p = &value};
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg),
                      TAG, "select YT8531 chip config");
  reg.reg_addr = 0x1fu;
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_READ_PHY_REG, &reg),
                      TAG, "read YT8531 chip config");
  value |= BIT(8);
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg),
                      TAG, "set YT8531 RX delay");

  value = 0xa003u;
  reg.reg_addr = 0x1eu;
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg),
                      TAG, "select YT8531 RGMII config");
  reg.reg_addr = 0x1fu;
  ESP_RETURN_ON_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_READ_PHY_REG, &reg),
                      TAG, "read YT8531 RGMII config");
  value = (value & ~0xffu) | (13u << 4) | 13u;
  return esp_eth_ioctl(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg);
}

static esp_err_t ethernet_driver_init(void) {
  ESP_LOGI(TAG, "creating S31 GMAC");
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  mac_config.flags |= ETH_MAC_FLAG_PIN_TO_CORE;
  /* The authenticated TX-IQ fast path is consumed directly by this task.
   * Keep it above lwIP and the stream service so the six DMA descriptors are
   * reclaimed promptly during a sustained host-to-radio transfer. */
  mac_config.rx_task_prio = 23u;
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = -1;
  phy_config.reset_gpio_num = 7;

  eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  emac_config.smi_gpio.mdc_num = 5;
  emac_config.smi_gpio.mdio_num = 6;
  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
  ESP_RETURN_ON_FALSE(mac != NULL, ESP_ERR_NO_MEM, TAG, "create S31 GMAC");
  ESP_LOGI(TAG, "created S31 GMAC; creating PHY");
  esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
  if (phy == NULL) {
    mac->del(mac);
    return ESP_ERR_NO_MEM;
  }
  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  ESP_LOGI(TAG, "created PHY; installing Ethernet driver");
  esp_err_t err = esp_eth_driver_install(&config, &s_eth_handle);
  if (err != ESP_OK) {
    mac->del(mac);
    phy->del(phy);
    return err;
  }
  ESP_RETURN_ON_ERROR(
      esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, s_source_mac), TAG,
      "read Ethernet MAC address");
  ESP_LOGI(TAG, "installed Ethernet driver; configuring YT8531");
  return yt8531_init(s_eth_handle);
}

static uint32_t json_u32(cJSON *object, const char *name, uint32_t value) {
  cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
  if (cJSON_IsNumber(item) && item->valuedouble >= 0.0 &&
      item->valuedouble <= UINT32_MAX) {
    return (uint32_t)item->valuedouble;
  }
  return value;
}

static int32_t json_i32(cJSON *object, const char *name, int32_t value) {
  cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
  if (cJSON_IsNumber(item) && item->valuedouble >= INT32_MIN &&
      item->valuedouble <= INT32_MAX) {
    return (int32_t)item->valuedouble;
  }
  return value;
}

static bool parse_config(cJSON *root, capture_config_t *c) {
  if (!cJSON_IsObject(root))
    return false;
  cJSON *o;
#define GET_OBJECT(name)                                                       \
  o = cJSON_GetObjectItemCaseSensitive(root, name);                            \
  if (cJSON_IsObject(o))
#define U32(name, field) c->field = json_u32(o, name, c->field)
  GET_OBJECT("stream") {
    U32("stream_wifi_packets", stream.stream_wifi_packets);
  }
  GET_OBJECT("radio") {
    U32("rf_freq_hz", radio.rf_freq_hz);
    c->radio.frequency_correction_ppb = json_i32(
        o, "frequency_correction_ppb", c->radio.frequency_correction_ppb);
  }
  GET_OBJECT("gain") {
    U32("gain_mode", gain.gain_mode);
    U32("rx_gain", gain.rx_gain);
    U32("tx_gain", gain.tx_gain);
    U32("expert_gain_word0", gain.expert_gain_word0);
    U32("expert_gain_word1", gain.expert_gain_word1);
    U32("expert_gain_word2", gain.expert_gain_word2);
  }
  GET_OBJECT("bandwidth") {
    U32("bw_mhz", bandwidth.bw_mhz);
    U32("second_chan", bandwidth.second_chan);
  }
  GET_OBJECT("loopback") {
    U32("loopback", loopback.loopback);
    U32("loopback_tx_gain", loopback.loopback_tx_gain);
    U32("loopback_rx_gain", loopback.loopback_rx_gain);
    U32("loopback_bb_gain", loopback.loopback_bb_gain);
  }
  GET_OBJECT("tx") {
    U32("tx_tone_enable", tx.tx_tone_enable);
    c->tx.tx_tone0_step = json_i32(o, "tx_tone0_step", c->tx.tx_tone0_step);
  }
  GET_OBJECT("iq_engine") {
    U32("adc_decimation", iq_engine.adc_decimation);
    U32("adc_source_sel", iq_engine.adc_source_sel);
  }
  GET_OBJECT("trigger") {
    U32("trigger_mode", trigger.trigger_mode);
    cJSON *a = cJSON_GetObjectItemCaseSensitive(o, "trigger_config");
    if (cJSON_IsArray(a)) {
      for (uint32_t i = 0; i < IQ_TRIGGER_CONFIG_WORDS; ++i) {
        cJSON *item = cJSON_GetArrayItem(a, i);
        if (cJSON_IsNumber(item) && item->valuedouble >= 0.0 &&
            item->valuedouble <= UINT32_MAX)
          c->trigger.trigger_config[i] = (uint32_t)item->valuedouble;
      }
    }
  }
  GET_OBJECT("rx_filter") {
    U32("filter_bw_mhz", rx_filter.filter_bw_mhz);
    U32("rx_filter_override", rx_filter.rx_filter_override);
    U32("rx_filter_mode", rx_filter.rx_filter_mode);
    U32("rx_filter_dcap", rx_filter.rx_filter_dcap);
  }
  GET_OBJECT("wifi_tx") {
    U32("wifi_dummy_tx_enable", wifi_tx.wifi_dummy_tx_enable);
    U32("wifi_dummy_tx_interval_ms", wifi_tx.wifi_dummy_tx_interval_ms);
  }
  GET_OBJECT("dc_offset") {
    U32("automatic", dc_offset.automatic);
  }
#undef U32
#undef GET_OBJECT
  return true;
}

static const char *validate_config(const capture_config_t *c) {
  const uint8_t gain_entry_count = gaintable_entry_count();
  if (c->gain.gain_mode > GAIN_MODE_EXPERT)
    return "gain_mode must be 0 (auto), 1 (manual), or 2 (expert)";
  if (c->gain.gain_mode == GAIN_MODE_MANUAL &&
      (gain_entry_count == 0u || c->gain.rx_gain >= gain_entry_count))
    return "manual rx_gain exceeds the calibrated gain table";
  if (c->gain.tx_gain > TX_GAIN_MAX)
    return "tx_gain must be in the range 0..63";
  if (c->radio.frequency_correction_ppb < -RF_CORRECTION_MAX_PPB ||
      c->radio.frequency_correction_ppb > RF_CORRECTION_MAX_PPB)
    return "frequency_correction_ppb must be in the range -100000..100000";
  if (c->dc_offset.automatic > 1u)
    return "dc_offset automatic must be 0 or 1";
  return NULL;
}

int iq_control_build_config_json(char *text, size_t cap) {
  capture_config_t config = {0};
  s_callbacks.get_config(&config);
  int n = snprintf(
      text, cap,
      "{\"stream\":{\"stream_wifi_packets\":%" PRIu32 "},"
      "\"radio\":{\"rf_freq_hz\":%" PRIu32
      ",\"frequency_correction_ppb\":%" PRId32 "},"
      "\"gain\":{\"gain_mode\":%" PRIu32 ",\"rx_gain\":%" PRIu32
      ",\"tx_gain\":%" PRIu32
      ",\"expert_gain_word0\":%" PRIu32 ",\"expert_gain_word1\":%" PRIu32
      ",\"expert_gain_word2\":%" PRIu32 "},"
      "\"bandwidth\":{\"bw_mhz\":%" PRIu32 ",\"second_chan\":%" PRIu32 "},"
      "\"loopback\":{\"loopback\":%" PRIu32 ",\"loopback_tx_gain\":%" PRIu32
      ",\"loopback_rx_gain\":%" PRIu32 ",\"loopback_bb_gain\":%" PRIu32 "},"
      "\"tx\":{\"tx_tone_enable\":%" PRIu32 ",\"tx_tone0_step\":%" PRId32 "},"
      "\"iq_engine\":{\"adc_decimation\":%" PRIu32 ",\"adc_source_sel\":%" PRIu32 "},"
      "\"trigger\":{\"trigger_mode\":%" PRIu32 ",\"trigger_config\":[%" PRIu32
      ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
      ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
      ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]},"
      "\"rx_filter\":{\"filter_bw_mhz\":%" PRIu32
      ",\"rx_filter_override\":%" PRIu32 ",\"rx_filter_mode\":%" PRIu32
      ",\"rx_filter_dcap\":%" PRIu32 "},"
      "\"wifi_tx\":{\"wifi_dummy_tx_enable\":%" PRIu32
      ",\"wifi_dummy_tx_interval_ms\":%" PRIu32 "},"
      "\"dc_offset\":{\"automatic\":%" PRIu32 "}}",
      config.stream.stream_wifi_packets, config.radio.rf_freq_hz,
      config.radio.frequency_correction_ppb,
      config.gain.gain_mode, config.gain.rx_gain, config.gain.tx_gain,
      config.gain.expert_gain_word0,
      config.gain.expert_gain_word1, config.gain.expert_gain_word2,
      config.bandwidth.bw_mhz, config.bandwidth.second_chan,
      config.loopback.loopback, config.loopback.loopback_tx_gain,
      config.loopback.loopback_rx_gain, config.loopback.loopback_bb_gain,
      config.tx.tx_tone_enable, config.tx.tx_tone0_step,
      config.iq_engine.adc_decimation, config.iq_engine.adc_source_sel,
      config.trigger.trigger_mode, config.trigger.trigger_config[0],
      config.trigger.trigger_config[1], config.trigger.trigger_config[2],
      config.trigger.trigger_config[3], config.trigger.trigger_config[4],
      config.trigger.trigger_config[5], config.trigger.trigger_config[6],
      config.trigger.trigger_config[7], config.trigger.trigger_config[8],
      config.trigger.trigger_config[9], config.trigger.trigger_config[10],
      config.trigger.trigger_config[11], config.trigger.trigger_config[12],
      config.trigger.trigger_config[13], config.trigger.trigger_config[14],
      config.trigger.trigger_config[15], config.rx_filter.filter_bw_mhz,
      config.rx_filter.rx_filter_override, config.rx_filter.rx_filter_mode,
      config.rx_filter.rx_filter_dcap, config.wifi_tx.wifi_dummy_tx_enable,
      config.wifi_tx.wifi_dummy_tx_interval_ms, config.dc_offset.automatic);
  return (n > 0 && (size_t)n < cap) ? n : -1;
}

static esp_err_t config_get_handler(httpd_req_t *req) {
  char text[1024];
  int n = iq_control_build_config_json(text, sizeof(text));
  ESP_RETURN_ON_FALSE(n > 0, ESP_ERR_INVALID_SIZE, TAG, "format config JSON");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, text, n);
}

static esp_err_t receive_json(httpd_req_t *req, cJSON **json) {
  if (req->content_len <= 0 || req->content_len >= HTTP_BODY_MAX_BYTES) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid JSON body size");
    return ESP_ERR_INVALID_SIZE;
  }
  char *body = malloc((size_t)req->content_len + 1u);
  if (body == NULL) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    return ESP_ERR_NO_MEM;
  }
  size_t received = 0;
  while (received < (size_t)req->content_len) {
    int n = httpd_req_recv(req, body + received, req->content_len - received);
    if (n <= 0) {
      free(body);
      return ESP_FAIL;
    }
    received += (size_t)n;
  }
  body[received] = '\0';
  *json = cJSON_ParseWithLength(body, received);
  free(body);
  if (*json == NULL) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "malformed JSON");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

const char *iq_control_parse_config_json(struct cJSON *root,
                                         capture_config_t *config) {
  s_callbacks.get_config(config);
  if (!parse_config(root, config)) {
    return "configuration must be an object";
  }
  return validate_config(config);
}

static esp_err_t config_put_handler(httpd_req_t *req) {
  cJSON *json = NULL;
  ESP_RETURN_ON_ERROR(receive_json(req, &json), TAG, "receive config JSON");
  capture_config_t config = {0};
  const char *validation_error = iq_control_parse_config_json(json, &config);
  cJSON_Delete(json);
  if (validation_error != NULL) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, validation_error);
    return ESP_ERR_INVALID_ARG;
  }
  httpd_resp_set_status(req, "202 Accepted");
  esp_err_t response_err = httpd_resp_send(req, NULL, 0);
  /* Applying a radio configuration wakes the high-priority stream task and
   * can take several seconds.  Queue it only after the 202 response has been
   * handed to TCP, otherwise that task can preempt the HTTP server before the
   * client sees any response. */
  s_callbacks.apply_config(&config);
  return response_err;
}

static esp_err_t tx_waveform_put_handler(httpd_req_t *req) {
  if (req->content_len <= 0 ||
      (req->content_len % (int)sizeof(uint32_t)) != 0 ||
      req->content_len > (int)TX_WAVEFORM_MAX_BYTES) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                        "body must contain 1..1048512 packed IQ10 words");
    return ESP_ERR_INVALID_SIZE;
  }
  capture_config_t config;
  s_callbacks.get_config(&config);
  if (s_callbacks.is_config_applying() || config.tx.tx_tone_enable != 0u) {
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, "stop TX before uploading a waveform");
  }
  /* A maximum batch is about 4 MiB. Receive it in PSRAM; the callback copies
   * it to its persistent PSRAM slot before this returns. */
  uint32_t *words = heap_caps_malloc((size_t)req->content_len,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (words == NULL) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "out of memory");
    return ESP_ERR_NO_MEM;
  }
  size_t received = 0u;
  while (received < (size_t)req->content_len) {
    int n = httpd_req_recv(req, (char *)words + received,
                           (size_t)req->content_len - received);
    if (n <= 0) {
      free(words);
      return ESP_FAIL;
    }
    received += (size_t)n;
  }
  uint32_t word_count = (uint32_t)(received / sizeof(uint32_t));
  for (uint32_t i = 0u; i < word_count; ++i) {
    if ((words[i] & 0xfff00000u) != 0u) {
      free(words);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                          "IQ10 word uses reserved upper bits");
      return ESP_ERR_INVALID_ARG;
    }
  }
  bool stored = s_callbacks.set_tx_waveform != NULL &&
                s_callbacks.set_tx_waveform(words, word_count);
  free(words);
  if (!stored) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "could not store waveform");
    return ESP_FAIL;
  }
  httpd_resp_set_status(req, "204 No Content");
  return httpd_resp_send(req, NULL, 0);
}

static bool http_peer_ipv4(httpd_req_t *req, uint32_t *ipv4) {
  struct sockaddr_storage peer = {0};
  socklen_t peer_len = sizeof(peer);
  if (getpeername(httpd_req_to_sockfd(req), (struct sockaddr *)&peer,
                  &peer_len) != 0) {
    return false;
  }
  if (peer.ss_family == AF_INET) {
    *ipv4 = ((struct sockaddr_in *)&peer)->sin_addr.s_addr;
    return true;
  }
  if (peer.ss_family == AF_INET6) {
    const uint8_t *bytes =
        ((struct sockaddr_in6 *)&peer)->sin6_addr.s6_addr;
    static const uint8_t mapped_prefix[12] = {0, 0, 0, 0, 0,    0,
                                              0, 0, 0, 0, 0xff, 0xff};
    if (memcmp(bytes, mapped_prefix, sizeof(mapped_prefix)) == 0) {
      memcpy(ipv4, bytes + 12, sizeof(*ipv4));
      return true;
    }
  }
  return false;
}

static esp_err_t tx_udp_arm_handler(httpd_req_t *req) {
  /* This endpoint is used once per continuous-TX batch. esp_http_server
   * emits the small headers and JSON body separately; with Nagle enabled the
   * body can sit behind a roughly 40 ms delayed ACK on a persistent control
   * connection. Keep this socket low-latency for the remainder of its life. */
  const int http_fd = httpd_req_to_sockfd(req);
  const int tcp_nodelay = 1;
  (void)setsockopt(http_fd, IPPROTO_TCP, TCP_NODELAY, &tcp_nodelay,
                   sizeof(tcp_nodelay));
  uint32_t peer_ipv4 = 0u;
  if (!http_peer_ipv4(req, &peer_ipv4)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                        "TX UDP requires an IPv4 HTTP peer");
    return ESP_ERR_NOT_SUPPORTED;
  }
  cJSON *json = NULL;
  ESP_RETURN_ON_ERROR(receive_json(req, &json), TAG,
                      "receive TX UDP arm JSON");
  uint64_t start_time_ns = 0u;
  uint32_t requested_words = TX_BATCH_WORDS_MAX;
  cJSON *start_json =
      cJSON_GetObjectItemCaseSensitive(json, "start_time_ns");
  if (start_json != NULL) {
    if (!cJSON_IsNumber(start_json) || start_json->valuedouble < 0.0 ||
        start_json->valuedouble > (double)INT64_MAX) {
      cJSON_Delete(json);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                          "start_time_ns must be a non-negative integer");
      return ESP_ERR_INVALID_ARG;
    }
    start_time_ns = (uint64_t)start_json->valuedouble;
  }
  cJSON *words_json =
      cJSON_GetObjectItemCaseSensitive(json, "word_count");
  if (words_json != NULL) {
    if (!cJSON_IsNumber(words_json) || words_json->valuedouble < 1.0 ||
        words_json->valuedouble > TX_BATCH_WORDS_MAX ||
        words_json->valuedouble != (double)(uint32_t)words_json->valuedouble) {
      cJSON_Delete(json);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                          "word_count must be an integer in range");
      return ESP_ERR_INVALID_ARG;
    }
    requested_words = (uint32_t)words_json->valuedouble;
  }
  bool requested_packed20 = false;
  cJSON *packed20_json = cJSON_GetObjectItemCaseSensitive(json, "packed20");
  if (packed20_json != NULL) {
    if (!cJSON_IsBool(packed20_json)) {
  cJSON_Delete(json);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                          "packed20 must be boolean");
      return ESP_ERR_INVALID_ARG;
    }
    requested_packed20 = cJSON_IsTrue(packed20_json);
  }
  bool requested_packed16 = false;
  cJSON *packed16_json = cJSON_GetObjectItemCaseSensitive(json, "packed16");
  if (packed16_json != NULL) {
    if (!cJSON_IsBool(packed16_json)) {
      cJSON_Delete(json);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                          "packed16 must be boolean");
      return ESP_ERR_INVALID_ARG;
    }
    requested_packed16 = cJSON_IsTrue(packed16_json);
  }
  if (requested_packed16 && requested_packed20) {
    cJSON_Delete(json);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                        "packed16 and packed20 are mutually exclusive");
    return ESP_ERR_INVALID_ARG;
  }
  cJSON_Delete(json);
#if CONFIG_IDF_TARGET_ESP32S31
  const uint8_t requested_storage_bits = requested_packed16 ? 16u
                                         : requested_packed20 ? 20u
                                                              : 32u;
#else
  const uint8_t requested_storage_bits = 32u;
#endif
  const size_t requested_bytes = requested_storage_bits == 16u
                                     ? (size_t)requested_words * 2u
                                 : requested_storage_bits == 20u
                                     ? ((size_t)requested_words * 5u + 1u) / 2u
                                     : (size_t)requested_words *
                                           sizeof(*s_tx_udp_batch);
  if (s_tx_udp_batch != NULL &&
      s_tx_udp_batch_storage_bits != requested_storage_bits) {
    heap_caps_free(s_tx_udp_batch);
    s_tx_udp_batch = NULL;
    s_tx_udp_batch_capacity_words = 0u;
  }
  if (s_tx_udp_batch != NULL &&
      s_tx_udp_batch_capacity_words < requested_words) {
    uint32_t *resized = heap_caps_realloc(
        s_tx_udp_batch, requested_bytes,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (resized == NULL) {
      resized = heap_caps_realloc(
          s_tx_udp_batch, requested_bytes,
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (resized != NULL) {
      s_tx_udp_batch = resized;
      s_tx_udp_batch_capacity_words = requested_words;
      s_tx_udp_batch_storage_bits = requested_storage_bits;
    }
  }
  if (s_tx_udp_batch == NULL) {
    /* A full continuation FIFO owns up to six large batches. At 2 MSa/s each
     * 1,048,320-sample buffer remains live for 524 ms, so the historical
     * 300 ms allocator retry falsely reported OOM before the oldest batch
     * could be recycled. Keep the endpoint backpressured for up to two
     * seconds; the Soapy call has a ten-second end-to-end timeout. */
    for (uint32_t attempt = 0u;
         attempt < 2000u && s_tx_udp_batch == NULL; ++attempt) {
      if (s_callbacks.reclaim_tx_waveforms != NULL) {
        s_callbacks.reclaim_tx_waveforms();
      }
      s_tx_udp_batch = heap_caps_malloc(requested_bytes,
          MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
      if (s_tx_udp_batch == NULL) {
        /* Preserve compatibility on targets whose capability allocator does
         * not label external RAM as DMA-addressable. */
        s_tx_udp_batch = heap_caps_malloc(requested_bytes,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      }
      if (s_tx_udp_batch != NULL) {
        s_tx_udp_batch_capacity_words = requested_words;
        s_tx_udp_batch_storage_bits = requested_storage_bits;
      }
      if (s_tx_udp_batch == NULL) {
        vTaskDelay(1);
      }
    }
    if (s_tx_udp_batch == NULL) {
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                          "cannot allocate TX UDP batch buffer");
      return ESP_ERR_NO_MEM;
    }
  }
  if (s_tx_udp_batch_capacity_words < requested_words) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "cannot grow TX UDP batch buffer");
    return ESP_ERR_NO_MEM;
  }
  s_tx_udp_session_max_words = requested_words;
  s_tx_udp_session_packed20 = requested_packed20;
  s_tx_udp_session_packed16 = requested_packed16;
  uint32_t token = esp_random();
  if (token == 0u) {
    token = 1u;
  }
  int64_t expires_us = esp_timer_get_time() + TX_UDP_ARM_LIFETIME_US;
  taskENTER_CRITICAL(&s_tx_udp_arm_mux);
  s_tx_udp_allowed_ipv4 = peer_ipv4;
  s_tx_udp_session_token = token;
  s_tx_udp_arm_expires_us = expires_us;
  s_tx_udp_start_time_ns = start_time_ns;
  taskEXIT_CRITICAL(&s_tx_udp_arm_mux);

  char response[192];
  int length = snprintf(response, sizeof(response),
                        "{\"port\":%u,\"session_token\":%" PRIu32
                        ",\"max_words\":%" PRIu32 ",\"expires_ms\":15000"
                        ",\"commit_count\":%" PRIu32
                        ",\"error_count\":%" PRIu32 "}",
                        IQ_NETWORK_TX_UDP_PORT, token, requested_words,
                        s_tx_udp_commits,
                        s_tx_udp_errors + s_tx_udp_commit_rejections);
  ESP_RETURN_ON_FALSE(length > 0 && length < (int)sizeof(response),
                      ESP_ERR_INVALID_SIZE, TAG, "format TX UDP arm response");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, response, length);
}

static bool tx_udp_session_valid(uint32_t peer_ipv4, uint32_t token) {
  bool valid;
  int64_t now_us = esp_timer_get_time();
  taskENTER_CRITICAL(&s_tx_udp_arm_mux);
  valid = token != 0u && token == s_tx_udp_session_token &&
          peer_ipv4 == s_tx_udp_allowed_ipv4 &&
          now_us < s_tx_udp_arm_expires_us;
  taskEXIT_CRITICAL(&s_tx_udp_arm_mux);
  return valid;
}

static bool tx_udp_session_armed(void) {
  bool armed;
  int64_t now_us = esp_timer_get_time();
  taskENTER_CRITICAL(&s_tx_udp_arm_mux);
  armed = s_tx_udp_session_token != 0u &&
          now_us < s_tx_udp_arm_expires_us;
  taskEXIT_CRITICAL(&s_tx_udp_arm_mux);
  return armed;
}

static void tx_udp_session_disarm(uint32_t token) {
  taskENTER_CRITICAL(&s_tx_udp_arm_mux);
  if (token == s_tx_udp_session_token) {
    s_tx_udp_session_token = 0u;
    s_tx_udp_allowed_ipv4 = 0u;
    s_tx_udp_arm_expires_us = 0;
    s_tx_udp_start_time_ns = 0u;
  }
  taskEXIT_CRITICAL(&s_tx_udp_arm_mux);
}

static bool tx_udp_bitmap_test(uint32_t sequence) {
  return (s_tx_udp_received_bitmap[sequence / 32u] &
          BIT(sequence % 32u)) != 0u;
}

static void tx_udp_bitmap_set(uint32_t sequence) {
  s_tx_udp_received_bitmap[sequence / 32u] |= BIT(sequence % 32u);
}

#if !CONFIG_IDF_TARGET_ESP32S31
static void __attribute__((noinline, optimize("O3,unroll-loops")))
tx_udp_unpack20_in_place(uint32_t *storage, const uint8_t *source,
                         uint32_t sample_count) {
  /* Source and destination share the allocation. Decode backwards because
   * each 32-bit destination sample is wider than its 20-bit wire sample. */
  for (uint32_t index = sample_count; index != 0u;) {
    --index;
    const uint8_t *pair = source + (index >> 1u) * 5u;
    storage[index] = (index & 1u) != 0u
                         ? ((uint32_t)pair[2] >> 4u) |
                               ((uint32_t)pair[3] << 4u) |
                               ((uint32_t)pair[4] << 12u)
                         : (uint32_t)pair[0] |
                               ((uint32_t)pair[1] << 8u) |
                               (((uint32_t)pair[2] & 0x0fu) << 16u);
  }
}

static void __attribute__((noinline, optimize("O3,unroll-loops")))
tx_udp_unpack16_in_place(uint32_t *storage, const uint8_t *source,
                         uint32_t sample_count) {
  /* Decode backwards so expansion cannot overwrite unread compact bytes. */
  for (uint32_t index = sample_count; index != 0u;) {
    --index;
    const int32_t i = (int32_t)(int8_t)source[index * 2u] * 4;
    const int32_t q = (int32_t)(int8_t)source[index * 2u + 1u] * 4;
    storage[index] = ((uint32_t)i & 0x3ffu) |
                     (((uint32_t)q & 0x3ffu) << 10u);
  }
}
#endif

static bool tx_udp_try_commit(void) {
  if (!s_tx_udp_commit_pending ||
      s_tx_udp_received_words != s_tx_udp_total_words) {
    return false;
  }
  if ((s_tx_udp_packed20 || s_tx_udp_packed16) &&
      !s_tx_udp_expanded_compact) {
#if CONFIG_IDF_TARGET_ESP32S31
    /* S31's CPU-fed TXDC backend consumes compact IQ directly. Expanding a
     * 524288-sample batch here either occupies the EMAC receive task for
     * about 40 ms (CPU) or makes a second uncached PSRAM round trip for about
     * 150 ms (BitScrambler).  Both erase the bandwidth gained on the wire.
     * Preserve the allocation base for ownership/recycling and pass the
     * packed flag through to the realtime emitter instead. */
    s_tx_udp_expanded_compact = true;
#else
    const uint32_t unpack_start = esp_cpu_get_cycle_count();
    uint8_t *source = (uint8_t *)s_tx_udp_batch +
                      s_tx_udp_compact_base_bytes;
    const size_t compact_bytes = s_tx_udp_packed16
                                     ? (size_t)s_tx_udp_total_words * 2u
                                     : ((size_t)s_tx_udp_total_words * 5u +
                                        1u) /
                                           2u;
    const size_t unpack_base =
        (size_t)s_tx_udp_total_words * sizeof(*s_tx_udp_batch) -
        compact_bytes;
    if (unpack_base != s_tx_udp_compact_base_bytes) {
      memmove((uint8_t *)s_tx_udp_batch + unpack_base, source,
              compact_bytes);
      source = (uint8_t *)s_tx_udp_batch + unpack_base;
    }
    if (s_tx_udp_packed16) {
      tx_udp_unpack16_in_place(s_tx_udp_batch, source,
                               s_tx_udp_total_words);
    } else {
      tx_udp_unpack20_in_place(s_tx_udp_batch, source,
                               s_tx_udp_total_words);
    }
    const uint32_t unpack_cycles = esp_cpu_get_cycle_count() - unpack_start;
    if (unpack_cycles > s_tx_udp_unpack_max_cycles) {
      s_tx_udp_unpack_max_cycles = unpack_cycles;
    }
    s_tx_udp_expanded_compact = true;
#endif
  }
  bool adopted = false;
  bool stored = false;
  if (s_callbacks.take_tx_waveform != NULL) {
    const iq_tx_waveform_result_t result =
        s_callbacks.take_tx_waveform(s_tx_udp_batch,
                                     s_tx_udp_total_words,
                                     s_tx_udp_commit_flags,
                                     s_tx_udp_start_time_ns);
    if (result == IQ_TX_WAVEFORM_RETRY) {
      ++s_tx_udp_backpressure_retries;
      return false;
    }
    adopted = result == IQ_TX_WAVEFORM_ADOPTED;
    stored = adopted;
  } else if (s_callbacks.set_tx_waveform != NULL) {
    stored = s_callbacks.set_tx_waveform(s_tx_udp_batch,
                                         s_tx_udp_total_words);
  }
  if (!stored) {
    ++s_tx_udp_commit_rejections;
    s_tx_udp_commit_pending = false;
    tx_udp_session_disarm(s_tx_udp_commit_token);
    return false;
  }
  s_tx_udp_committed_words = s_tx_udp_total_words;
  ++s_tx_udp_commits;
  s_tx_udp_commit_pending = false;
  if (adopted) {
    s_tx_udp_batch = NULL;
    s_tx_udp_batch_capacity_words = 0u;
  }
  tx_udp_session_disarm(s_tx_udp_commit_token);
  return true;
}

static bool tx_udp_process_payload(const uint8_t *packet,
                                   uint32_t packet_bytes,
                                   uint32_t peer_ipv4,
                                   uint32_t *ack_token,
                                   uint32_t *ack_batch_id) {
  *ack_token = 0u;
  *ack_batch_id = 0u;
  if (packet_bytes < sizeof(iq_tx_udp_header_t)) {
    ++s_tx_udp_errors;
    return false;
  }
  iq_tx_udp_header_t header;
  memcpy(&header, packet, sizeof(header));
  uint32_t header_crc = esp_rom_crc32_le(
      0u, packet, offsetof(iq_tx_udp_header_t, header_crc32));
  bool packed20 = (header.flags & IQ_TX_UDP_FLAG_PACKED20) != 0u;
  bool packed16 = (header.flags & IQ_TX_UDP_FLAG_PACKED16) != 0u;
  uint32_t words_per_datagram = packed16
                                    ? TX_UDP_PACKED16_WORDS_PER_DATAGRAM
                                : packed20
                                    ? TX_UDP_PACKED20_WORDS_PER_DATAGRAM
                                    : TX_UDP_WORDS_PER_DATAGRAM;
  uint32_t payload_bytes = packed16
                               ? (uint32_t)header.sample_count * 2u
                           : packed20
                               ? ((uint32_t)header.sample_count * 5u + 1u) / 2u
                               : (uint32_t)header.sample_count * sizeof(uint32_t);
  bool reset = (header.flags & IQ_TX_UDP_FLAG_RESET) != 0u;
  bool commit = (header.flags & IQ_TX_UDP_FLAG_COMMIT) != 0u;
  uint32_t rate_code =
      (header.flags & IQ_TX_UDP_RATE_CODE_M) >> IQ_TX_UDP_RATE_CODE_S;
  bool rate_valid = rate_code <= 3u ||
                    (rate_code >= 7u && rate_code <= 14u);
  if (memcmp(header.magic, IQ_TX_UDP_MAGIC, 4u) != 0 ||
      header.version != IQ_TX_UDP_VERSION ||
      header.header_bytes != sizeof(header) ||
      header.header_crc32 != header_crc || header.sample_count == 0u ||
      (packed16 && packed20) ||
      (header.flags & ~IQ_TX_UDP_FLAGS_ALLOWED) != 0u ||
      ((header.flags & IQ_TX_UDP_FLAG_AUTOSTART) != 0u && !commit) ||
      ((header.flags & IQ_TX_UDP_FLAG_AUTOSTART) != 0u && !rate_valid) ||
      ((packed20 || packed16) &&
       (header.flags & IQ_TX_UDP_FLAG_AUTOSTART) != 0u &&
       rate_code >= 14u) ||
      ((header.flags & IQ_TX_UDP_FLAG_AUTOSTART) == 0u &&
       (header.flags & (IQ_TX_UDP_RATE_CODE_M |
                        IQ_TX_UDP_FLAG_MORE |
                        IQ_TX_UDP_FLAG_CONTINUE)) != 0u) ||
      payload_bytes > IQ_UDP_FRAGMENT_PAYLOAD_BYTES ||
      packet_bytes != sizeof(header) + payload_bytes ||
      header.sample_offset % words_per_datagram != 0u ||
      header.datagram_sequence !=
          header.sample_offset / words_per_datagram ||
      (!commit && header.sample_count != words_per_datagram)) {
    ++s_tx_udp_errors;
    return false;
  }
  if (!tx_udp_session_valid(peer_ipv4, header.session_token)) {
    ++s_tx_udp_stale_datagrams;
    return false;
  }
  if (packed20 != s_tx_udp_session_packed20 ||
      packed16 != s_tx_udp_session_packed16 ||
      header.sample_count > s_tx_udp_session_max_words ||
      header.sample_offset >
          s_tx_udp_session_max_words - header.sample_count) {
    ++s_tx_udp_errors;
    return false;
  }
  bool ack_requested =
      (header.flags & IQ_TX_UDP_FLAG_ACK_REQUEST) != 0u;
  *ack_token = header.session_token;
  *ack_batch_id = header.batch_id;
  const uint8_t *payload = packet + sizeof(header);
  if (header.payload_crc32 != 0u &&
      header.payload_crc32 != esp_rom_crc32_le(0u, payload, payload_bytes)) {
    ++s_tx_udp_errors;
    return ack_requested;
  }
  if (reset) {
    if (s_tx_udp_batch == NULL || header.datagram_sequence != 0u) {
      ++s_tx_udp_errors;
      return ack_requested;
    }
    memset(s_tx_udp_received_bitmap, 0, sizeof(s_tx_udp_received_bitmap));
    s_tx_udp_batch_id = header.batch_id;
    s_tx_udp_received_words = 0u;
    s_tx_udp_committed_words = 0u;
    s_tx_udp_expected_sequence = 0u;
    s_tx_udp_final_sequence = 0u;
    s_tx_udp_total_words = 0u;
    s_tx_udp_commit_token = 0u;
    s_tx_udp_commit_flags = 0u;
    s_tx_udp_commit_pending = false;
    s_tx_udp_packed20 = packed20;
    s_tx_udp_packed16 = packed16;
    s_tx_udp_expanded_compact = false;
    s_tx_udp_compact_base_bytes = 0u;
#if !CONFIG_IDF_TARGET_ESP32S31
    if (packed20 || packed16) {
      /* Other targets expand in place and therefore keep the compact source
       * above the 32-bit destination to avoid overwriting unread bytes. */
      const size_t compact_bytes = packed16
                                       ? (size_t)s_tx_udp_session_max_words * 2u
                                       : ((size_t)s_tx_udp_session_max_words *
                                              5u +
                                          1u) /
                                             2u;
      s_tx_udp_compact_base_bytes =
          s_tx_udp_session_max_words * sizeof(*s_tx_udp_batch) - compact_bytes;
    }
#endif
  }
  if (s_tx_udp_batch == NULL || header.batch_id != s_tx_udp_batch_id ||
      packed20 != s_tx_udp_packed20 ||
      packed16 != s_tx_udp_packed16 ||
      header.datagram_sequence >= TX_UDP_DATAGRAMS_MAX ||
      (s_tx_udp_commit_pending &&
       header.datagram_sequence > s_tx_udp_final_sequence)) {
    ++s_tx_udp_errors;
    return ack_requested;
  }
  if (tx_udp_bitmap_test(header.datagram_sequence)) {
    /* Do not short-circuit the state transition when the packet also asks
     * for an ACK. A lossless final datagram normally carries ACK_REQUEST;
     * skipping try_commit() there left the complete batch armed forever. */
    bool committed = tx_udp_try_commit();
    return ack_requested || committed;
  }
  /* The replay engine consumes only the packed low 20 bits.  Scanning all
   * 350 words here costs a full extra pass over every high-rate datagram; the
   * authenticated host packer already guarantees the IQ10 representation. */
  if (packed16) {
    memcpy((uint8_t *)s_tx_udp_batch + s_tx_udp_compact_base_bytes +
               header.sample_offset * 2u,
           payload, payload_bytes);
  } else if (packed20) {
    memcpy((uint8_t *)s_tx_udp_batch + s_tx_udp_compact_base_bytes +
               header.sample_offset * 5u / 2u,
           payload, payload_bytes);
  } else {
  memcpy(s_tx_udp_batch + header.sample_offset, payload, payload_bytes);
  }
  tx_udp_bitmap_set(header.datagram_sequence);
  ++s_tx_udp_datagrams;
  s_tx_udp_bytes += payload_bytes;
  if (commit) {
    s_tx_udp_final_sequence = header.datagram_sequence;
    s_tx_udp_total_words = header.sample_offset + header.sample_count;
    s_tx_udp_commit_token = header.session_token;
    s_tx_udp_commit_flags = header.flags;
    s_tx_udp_commit_pending = true;
  }
  while (s_tx_udp_expected_sequence < TX_UDP_DATAGRAMS_MAX &&
         tx_udp_bitmap_test(s_tx_udp_expected_sequence)) {
    uint32_t words = s_tx_udp_packed16
                         ? TX_UDP_PACKED16_WORDS_PER_DATAGRAM
                     : s_tx_udp_packed20
                         ? TX_UDP_PACKED20_WORDS_PER_DATAGRAM
                         : TX_UDP_WORDS_PER_DATAGRAM;
    if (s_tx_udp_commit_pending &&
        s_tx_udp_expected_sequence == s_tx_udp_final_sequence) {
      words = s_tx_udp_total_words -
              s_tx_udp_expected_sequence * words;
    }
    s_tx_udp_received_words += words;
    ++s_tx_udp_expected_sequence;
    if (s_tx_udp_commit_pending &&
        s_tx_udp_expected_sequence > s_tx_udp_final_sequence) {
      break;
    }
  }
  bool committed = tx_udp_try_commit();
  return ack_requested || committed;
}

static uint16_t load_be16(const uint8_t *bytes) {
  return ((uint16_t)bytes[0] << 8) | bytes[1];
}

static void tx_udp_send_ack(const uint8_t peer_mac[6], uint32_t peer_ipv4,
                            uint16_t peer_port, uint32_t session_token,
                            uint32_t batch_id) {
  iq_tx_udp_ack_t ack = {
      .magic = {'I', 'Q', 'A', '1'},
      .version = IQ_TX_UDP_VERSION,
      .header_bytes = sizeof(iq_tx_udp_ack_t),
      .session_token = session_token,
      .batch_id = batch_id,
      .next_sequence = s_tx_udp_expected_sequence,
      .received_words = s_tx_udp_received_words,
      .committed_words = s_tx_udp_committed_words,
      /* The ACK/arm transaction counter combines disjoint status counters so
       * the host detects either malformed input or a terminal handoff. */
      .error_count = s_tx_udp_errors + s_tx_udp_commit_rejections,
      .commit_count = s_tx_udp_commits,
  };
  ack.header_crc32 = esp_rom_crc32_le(
      0u, (const uint8_t *)&ack,
      offsetof(iq_tx_udp_ack_t, header_crc32));

  uint8_t frame[ETH_HEADER_BYTES + IPV4_HEADER_BYTES + UDP_HEADER_BYTES +
                sizeof(iq_tx_udp_ack_t)] = {0};
  memcpy(frame, peer_mac, 6u);
  memcpy(frame + 6u, s_source_mac, 6u);
  frame[12] = 0x08u;
  frame[13] = 0x00u;
  uint8_t *ip = frame + ETH_HEADER_BYTES;
  ip[0] = 0x45u;
  uint16_t ip_bytes = htons(IPV4_HEADER_BYTES + UDP_HEADER_BYTES +
                            sizeof(iq_tx_udp_ack_t));
  memcpy(ip + 2u, &ip_bytes, sizeof(ip_bytes));
  uint16_t identification = htons((uint16_t)batch_id);
  memcpy(ip + 4u, &identification, sizeof(identification));
  ip[6] = 0x40u;
  ip[8] = 64u;
  ip[9] = IPPROTO_UDP;
  memcpy(ip + 12u, &s_ipv4_address, sizeof(s_ipv4_address));
  memcpy(ip + 16u, &peer_ipv4, sizeof(peer_ipv4));
  uint16_t ip_checksum = htons(ipv4_checksum(ip));
  memcpy(ip + 10u, &ip_checksum, sizeof(ip_checksum));
  uint8_t *udp = ip + IPV4_HEADER_BYTES;
  uint16_t source_port = htons(IQ_NETWORK_TX_UDP_PORT);
  uint16_t destination_port = htons(peer_port);
  uint16_t udp_bytes = htons(UDP_HEADER_BYTES + sizeof(iq_tx_udp_ack_t));
  memcpy(udp, &source_port, sizeof(source_port));
  memcpy(udp + 2u, &destination_port, sizeof(destination_port));
  memcpy(udp + 4u, &udp_bytes, sizeof(udp_bytes));
  memcpy(udp + UDP_HEADER_BYTES, &ack, sizeof(ack));
  if (esp_eth_transmit(s_eth_handle, frame, sizeof(frame)) != ESP_OK) {
    ESP_LOGW(TAG, "TX UDP ACK transmit failed");
  }
}

uint8_t *__real_emac_esp_dma_alloc_recv_buf(
    emac_esp_dma_handle_t emac_dma, uint32_t *size);

static bool tx_udp_direct_frame_candidate(const uint8_t *frame,
                                          uint32_t length) {
  if (length < ETH_HEADER_BYTES + IPV4_HEADER_BYTES ||
      load_be16(frame + 12u) != 0x0800u) {
    return false;
  }
  const uint8_t *ip = frame + ETH_HEADER_BYTES;
  uint32_t ip_header_bytes = (uint32_t)(ip[0] & 0x0fu) * 4u;
  uint32_t ip_total_bytes = load_be16(ip + 2u);
  if ((ip[0] >> 4u) != 4u || ip_header_bytes < IPV4_HEADER_BYTES ||
      ip_header_bytes + UDP_HEADER_BYTES > ip_total_bytes ||
      ip_total_bytes > length - ETH_HEADER_BYTES ||
      ip[9] != IPPROTO_UDP ||
      (load_be16(ip + 6u) & 0x3fffu) != 0u) {
    return false;
  }
  const uint8_t *udp = ip + ip_header_bytes;
  uint32_t udp_bytes = load_be16(udp + 4u);
  return load_be16(udp + 2u) == IQ_NETWORK_TX_UDP_PORT &&
         udp_bytes >= UDP_HEADER_BYTES &&
         udp_bytes <= ip_total_bytes - ip_header_bytes;
}

static void tx_udp_process_direct_frame(const uint8_t *frame) {
  const uint8_t *ip = frame + ETH_HEADER_BYTES;
  uint32_t ip_header_bytes = (uint32_t)(ip[0] & 0x0fu) * 4u;
  const uint8_t *udp = ip + ip_header_bytes;
  uint32_t udp_bytes = load_be16(udp + 4u);
  uint32_t peer_ipv4;
  memcpy(&peer_ipv4, ip + 12u, sizeof(peer_ipv4));
  uint32_t ack_token;
  uint32_t ack_batch_id;
  bool send_ack = tx_udp_process_payload(
      udp + UDP_HEADER_BYTES, udp_bytes - UDP_HEADER_BYTES, peer_ipv4,
      &ack_token, &ack_batch_id);
  if (send_ack) {
    tx_udp_send_ack(frame + 6u, peer_ipv4, load_be16(udp), ack_token,
                    ack_batch_id);
  }
}

uint8_t *__wrap_emac_esp_dma_alloc_recv_buf(
    emac_esp_dma_handle_t emac_dma_handle, uint32_t *size) {
  iq_emac_dma_prefix_t *emac_dma =
      (iq_emac_dma_prefix_t *)emac_dma_handle;
  /* S31's EMAC descriptors and buffers live in coherent internal DMA RAM.
   * The IDF driver's DMA_CACHE_{INVALIDATE,WB} macros intentionally compile
   * to no-ops on this target; esp_cache_msync() rejects this address class. */
  uint32_t drained = 0u;
  while (drained < CONFIG_ETH_DMA_RX_BUFFER_NUM) {
    eth_dma_rx_descriptor_t *descriptor = emac_dma->rx_desc;
    bool cpu_owned = descriptor != NULL &&
                     descriptor->RDES0.Own == EMAC_LL_DMADESC_OWNER_CPU;
    if (!cpu_owned) {
      break;
    }
    bool single_desc = descriptor->RDES0.FirstDescriptor &&
                       descriptor->RDES0.LastDescriptor &&
                       !descriptor->RDES0.ErrSummary &&
                       descriptor->RDES0.FrameLength >= 4u;
    if (!single_desc) {
      break;
    }
    uint32_t frame_length = descriptor->RDES0.FrameLength - 4u;
    uint8_t *frame = (uint8_t *)(uintptr_t)descriptor->Buffer1Addr;
    if (frame_length > *size ||
        !tx_udp_direct_frame_candidate(frame, frame_length)) {
      break;
    }
    const uint32_t process_start_cycle = esp_cpu_get_cycle_count();
    tx_udp_process_direct_frame(frame);
    emac_dma->rx_desc = (eth_dma_rx_descriptor_t *)(uintptr_t)
        descriptor->Buffer2NextDescAddr;
    descriptor->RDES0.Own = EMAC_LL_DMADESC_OWNER_DMA;
    uint32_t process_cycles =
        esp_cpu_get_cycle_count() - process_start_cycle;
    ++s_tx_udp_direct_frames;
    s_tx_udp_direct_total_cycles += process_cycles;
    if (process_cycles > s_tx_udp_direct_max_cycles) {
      s_tx_udp_direct_max_cycles = process_cycles;
    }
    ++drained;
  }
  if (drained != 0u) {
    emac_hal_receive_poll_demand(&emac_dma->hal);
  }
  return __real_emac_esp_dma_alloc_recv_buf(emac_dma_handle, size);
}

/* Bypass lwIP only for the token-gated TX-IQ port.  The EMAC driver verifies
 * the Ethernet FCS; the application always checks its header CRC, optionally
 * checks a payload CRC, and binds every datagram to the HTTP peer's address. */
static esp_err_t iq_eth_input(esp_eth_handle_t eth_handle, uint8_t *buffer,
                              uint32_t length, void *priv, void *info) {
  (void)eth_handle;
  (void)info;
  if (buffer != NULL && length >= ETH_HEADER_BYTES + IPV4_HEADER_BYTES &&
      load_be16(buffer + 12u) == 0x0800u) {
    const uint8_t *ip = buffer + ETH_HEADER_BYTES;
    uint32_t ip_header_bytes = (uint32_t)(ip[0] & 0x0fu) * 4u;
    uint32_t ip_total_bytes = load_be16(ip + 2u);
    if ((ip[0] >> 4) == 4u && ip_header_bytes >= IPV4_HEADER_BYTES &&
        ip_header_bytes + UDP_HEADER_BYTES <= ip_total_bytes &&
        ip_total_bytes <= length - ETH_HEADER_BYTES && ip[9] == IPPROTO_UDP &&
        (load_be16(ip + 6u) & 0x3fffu) == 0u) {
      const uint8_t *udp = ip + ip_header_bytes;
      uint32_t udp_bytes = load_be16(udp + 4u);
      if (load_be16(udp + 2u) == IQ_NETWORK_TX_UDP_PORT) {
        if (udp_bytes < UDP_HEADER_BYTES ||
            udp_bytes > ip_total_bytes - ip_header_bytes) {
          ++s_tx_udp_errors;
        } else {
          uint32_t peer_ipv4;
          memcpy(&peer_ipv4, ip + 12u, sizeof(peer_ipv4));
          uint32_t ack_token;
          uint32_t ack_batch_id;
          bool send_ack = tx_udp_process_payload(
              udp + UDP_HEADER_BYTES, udp_bytes - UDP_HEADER_BYTES, peer_ipv4,
              &ack_token, &ack_batch_id);
          if (send_ack) {
            tx_udp_send_ack(buffer + 6u, peer_ipv4, load_be16(udp),
                            ack_token, ack_batch_id);
          }
        }
        free(buffer);
        return ESP_OK;
      }
    }
  }
  return esp_netif_receive((esp_netif_t *)priv, buffer, length, NULL);
}

int iq_control_build_status_json(char *text, size_t cap) {
  capture_config_t config = {0};
  s_callbacks.get_config(&config);
  uint32_t rx_decimation = config.iq_engine.adc_decimation;
  if (rx_decimation < 1u || rx_decimation > 10u) {
    rx_decimation = 1u;
  }
  const uint32_t rx_sample_rate_hz =
      IQ_RX_BASE_SAMPLE_RATE_HZ / rx_decimation;
  char ip[INET_ADDRSTRLEN] = "0.0.0.0";
  struct in_addr address = {.s_addr = s_ipv4_address};
  (void)inet_ntop(AF_INET, &address, ip, sizeof(ip));
  const uint8_t gain_entry_count = gaintable_entry_count();
  const unsigned gain_max = gain_entry_count > 0u
                                ? (unsigned)gain_entry_count - 1u
                                : RX_GAIN_MIN_DB;
  const iq_stream_owner_t owner = s_stream_owner;
  const char *owner_name = owner == IQ_STREAM_OWNER_ETH   ? "ethernet"
                           : owner == IQ_STREAM_OWNER_USB ? "usb"
                                                          : "none";
  uint32_t usb_tx_uploads = 0u;
  uint32_t usb_tx_bytes = 0u;
  uint32_t usb_tx_errors = 0u;
  uint32_t usb_tx_backpressure_retries = 0u;
  uint32_t usb_tx_commit_rejections = 0u;
  uint32_t usb_tx_received_words = 0u;
  bool usb_tx_active = false;
  iq_usb_tx_diag(&usb_tx_uploads, &usb_tx_bytes, &usb_tx_errors,
                 &usb_tx_backpressure_retries,
                 &usb_tx_commit_rejections,
                 &usb_tx_received_words, &usb_tx_active);
  iq_tx_replay_diag_t tx_replay = {0};
  s_callbacks.get_tx_replay_diag(&tx_replay);
  const uint64_t hardware_time_ns =
      (uint64_t)esp_timer_get_time() * 1000u;
  int n = snprintf(
      text, cap,
      "{\"hostname\":\"" IQ_NETWORK_HOSTNAME ".local\",\"link_up\":%s,"
      "\"has_ipv4\":%s,\"ipv4\":\"%s\",\"streaming\":%s,"
      "\"rx_sample_rate_hz\":%" PRIu32
      ",\"rx_decimation\":%" PRIu32 ","
      "\"stream_owner\":\"%s\",\"usb_mounted\":%s,"
      "\"usb_frames\":%" PRIu32 ",\"usb_send_errors\":%" PRIu32
      ",\"usb_stream_format\":%" PRIu32
      ",\"stream_format\":%" PRIu32
      ",\"usb_tx_uploads\":%" PRIu32
      ",\"usb_tx_errors\":%" PRIu32
      ",\"usb_tx_backpressure_retries\":%" PRIu32
      ",\"usb_tx_commit_rejections\":%" PRIu32
      ","
      "\"config_applying\":%s,\"stream_epoch\":%" PRIu32
      ",\"reset_reason\":%u"
      ",\"hardware_time_ns\":%" PRIu64
      ",\"udp_frames\":%" PRIu32 ",\"udp_datagrams\":%" PRIu32
      ",\"udp_bytes\":%" PRIu32 ",\"udp_send_errors\":%" PRIu32
      ",\"tx_udp_datagrams\":%" PRIu32 ",\"tx_udp_bytes\":%" PRIu32
      ",\"tx_udp_errors\":%" PRIu32
      ",\"tx_udp_stale_datagrams\":%" PRIu32
      ",\"tx_udp_backpressure_retries\":%" PRIu32
      ",\"tx_udp_commit_rejections\":%" PRIu32
      ",\"tx_udp_commits\":%" PRIu32 ",\"tx_udp_armed\":%s"
      ",\"tx_replay\":{\"words\":%" PRIu32
      ",\"rate_code\":%" PRIu32 ",\"segments\":%" PRIu32
      ",\"total_cycles\":%" PRIu32 ",\"sample_cycles\":%" PRIu32
      ",\"gap_cycles\":%" PRIu32
      ",\"maximum_gap_cycles\":%" PRIu32
      ",\"tcm_stage_copy_max_cycles\":%" PRIu32
      ",\"deadline_late_max_cycles\":%" PRIu32
      ",\"deadline_late_max_word\":%" PRIu32
      ",\"requested_start_time_ns\":%" PRIu64
      ",\"actual_start_time_ns\":%" PRIu64
      ",\"start_error_ns\":%" PRId64
      ",\"queue_underflow\":%s"
      ",\"deadline_missed\":%s}"
      ",\"firmware_dropped_chunks\":%" PRIu32
      ",\"source_chunk_index\":%" PRIu32 ",\"dcoc_diag\":%" PRIu32
      ",\"dc_offset_automatic\":%s,\"dcoc_active\":%s"
      ",\"software_agc\":{\"active\":%s,\"current_gain\":%" PRIu32
      ",\"last_robust_peak\":%" PRIu32 ",\"gain_changes\":%" PRIu32 "}"
      ",\"adc_dump_cfg\":%" PRIu32 ",\"adc_dump_mode\":%" PRIu32 ","
      "\"manual_rx_gain\":{\"unit\":\"dB\",\"minimum\":%u,"
      "\"maximum\":%u,\"step\":%u}}",
      s_link_up ? "true" : "false", s_has_ipv4 ? "true" : "false", ip,
      s_stream_armed ? "true" : "false", rx_sample_rate_hz, rx_decimation,
      owner_name,
      iq_usb_mounted() ? "true" : "false", iq_usb_frames(),
      iq_usb_send_errors(), iq_usb_stream_format(), iq_network_stream_format(),
      usb_tx_uploads, usb_tx_errors,
      usb_tx_backpressure_retries, usb_tx_commit_rejections,
      s_callbacks.is_config_applying() ? "true" : "false", s_stream_epoch,
      (unsigned)esp_reset_reason(),
      hardware_time_ns,
      s_udp_frames, s_udp_datagrams, s_udp_bytes, s_udp_send_errors,
      s_tx_udp_datagrams, s_tx_udp_bytes, s_tx_udp_errors,
      s_tx_udp_stale_datagrams,
      s_tx_udp_backpressure_retries,
      s_tx_udp_commit_rejections, s_tx_udp_commits,
      tx_udp_session_armed() ? "true" : "false",
      tx_replay.words, tx_replay.rate_code, tx_replay.segments,
      tx_replay.total_cycles, tx_replay.sample_cycles, tx_replay.gap_cycles,
      tx_replay.maximum_gap_cycles, tx_replay.tcm_stage_copy_max_cycles,
      tx_replay.deadline_late_max_cycles, tx_replay.deadline_late_max_word,
      tx_replay.requested_start_time_ns,
      tx_replay.actual_start_time_ns, tx_replay.start_error_ns,
      tx_replay.queue_underflow ? "true" : "false",
      tx_replay.deadline_missed ? "true" : "false",
      s_callbacks.get_firmware_dropped_chunks(),
      s_callbacks.get_source_chunk_index(), s_callbacks.get_dcoc_diag(),
      config.dc_offset.automatic != 0u ? "true" : "false",
      s_callbacks.get_dcoc_active() ? "true" : "false",
      sdr_agc_active() ? "true" : "false", sdr_agc_current_gain(),
      sdr_agc_last_peak(), sdr_agc_gain_changes(),
      s_callbacks.get_adc_dump_cfg(), s_callbacks.get_adc_dump_mode(),
      RX_GAIN_MIN_DB, gain_max, RX_GAIN_STEP_DB);
  return (n > 0 && (size_t)n < cap) ? n : -1;
}

static esp_err_t send_status(httpd_req_t *req) {
  char text[2048];
  int n = iq_control_build_status_json(text, sizeof(text));
  ESP_RETURN_ON_FALSE(n > 0, ESP_ERR_INVALID_SIZE, TAG, "format status JSON");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, text, n);
}

static esp_err_t status_get_handler(httpd_req_t *req) {
  return send_status(req);
}

static esp_err_t stream_start_handler(httpd_req_t *req) {
  cJSON *json = NULL;
  ESP_RETURN_ON_ERROR(receive_json(req, &json), TAG, "receive stream JSON");
  cJSON *port_json = cJSON_GetObjectItemCaseSensitive(json, "port");
  if (!cJSON_IsNumber(port_json) || port_json->valuedouble < 1 ||
      port_json->valuedouble > 65535) {
    cJSON_Delete(json);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "port must be 1..65535");
    return ESP_ERR_INVALID_ARG;
  }
  uint16_t port = (uint16_t)port_json->valueint;
  uint32_t format = IQ_USB_FORMAT_FULL;
  cJSON *format_json =
      cJSON_GetObjectItemCaseSensitive(json, "stream_format");
  if (format_json != NULL) {
    if (!cJSON_IsNumber(format_json) || format_json->valuedouble < 0.0 ||
        format_json->valuedouble > (double)IQ_USB_FORMAT_INT8 ||
        format_json->valuedouble != (double)format_json->valueint) {
      cJSON_Delete(json);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                          "stream_format must be 0 or 1");
      return ESP_ERR_INVALID_ARG;
    }
    format = (uint32_t)format_json->valueint;
  }
  cJSON_Delete(json);

  struct sockaddr_storage peer_storage = {0};
  socklen_t peer_len = sizeof(peer_storage);
  int http_fd = httpd_req_to_sockfd(req);
  if (getpeername(http_fd, (struct sockaddr *)&peer_storage, &peer_len) != 0) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "cannot determine IPv4 peer");
    return ESP_FAIL;
  }
  struct sockaddr_in peer = {.sin_family = AF_INET};
  if (peer_storage.ss_family == AF_INET) {
    peer.sin_addr = ((struct sockaddr_in *)&peer_storage)->sin_addr;
  } else if (peer_storage.ss_family == AF_INET6) {
    const uint8_t *bytes =
        ((struct sockaddr_in6 *)&peer_storage)->sin6_addr.s6_addr;
    static const uint8_t mapped_prefix[12] = {0, 0, 0, 0, 0,    0,
                                              0, 0, 0, 0, 0xff, 0xff};
    if (memcmp(bytes, mapped_prefix, sizeof(mapped_prefix)) != 0) {
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                          "HTTP peer is not IPv4");
      return ESP_ERR_NOT_SUPPORTED;
    }
    memcpy(&peer.sin_addr.s_addr, bytes + 12, sizeof(peer.sin_addr.s_addr));
  } else {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "unknown HTTP peer family");
    return ESP_ERR_NOT_SUPPORTED;
  }
  peer.sin_port = htons(port);
  uint8_t destination_mac[6];
  if (!resolve_peer_mac(peer.sin_addr, destination_mac)) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "HTTP peer is not present in ARP cache");
    return ESP_FAIL;
  }
  taskENTER_CRITICAL(&s_stream_mux);
  s_stream_destination = peer;
  memcpy(s_stream_destination_mac, destination_mac,
         sizeof(s_stream_destination_mac));
  taskEXIT_CRITICAL(&s_stream_mux);
  iq_network_stream_set_format(format);
  iq_network_stream_begin(IQ_STREAM_OWNER_ETH);
  esp_err_t response = send_status(req);
  iq_network_stream_arm();
  return response;
}

static esp_err_t stream_stop_handler(httpd_req_t *req) {
  iq_network_stream_end();
  return send_status(req);
}

iq_stream_owner_t iq_network_stream_owner(void) { return s_stream_owner; }

void iq_network_stream_set_format(uint32_t format) {
  taskENTER_CRITICAL(&s_stream_mux);
  s_stream_format = format;
  taskEXIT_CRITICAL(&s_stream_mux);
}

uint32_t iq_network_stream_format(void) { return s_stream_format; }

void iq_network_stream_begin(iq_stream_owner_t owner) {
  taskENTER_CRITICAL(&s_stream_mux);
  /* New start wins: replacing the owner silently stops the previous
   * transport because send_frame and tx_ticket check ownership. */
  s_stream_owner = owner;
  s_stream_armed = false;
  ++s_stream_epoch;
  if (s_stream_epoch == 0)
    ++s_stream_epoch;
  s_datagram_sequence = 1;
  taskEXIT_CRITICAL(&s_stream_mux);
}

void iq_network_stream_arm(void) {
  s_callbacks.set_capture_armed(true);
  (void)esp_timer_stop(s_stream_arm_timer);
  ESP_ERROR_CHECK(esp_timer_start_once(s_stream_arm_timer, STREAM_ARM_DELAY_US));
}

void iq_network_stream_end(void) {
  (void)esp_timer_stop(s_stream_arm_timer);
  taskENTER_CRITICAL(&s_stream_mux);
  s_stream_armed = false;
  s_stream_owner = IQ_STREAM_OWNER_NONE;
  taskEXIT_CRITICAL(&s_stream_mux);
  s_callbacks.set_capture_armed(false);
}

bool iq_network_stream_tx_ticket(iq_stream_owner_t owner, uint32_t *epoch,
                                 uint32_t *sequence) {
  bool ok;
  taskENTER_CRITICAL(&s_stream_mux);
  ok = s_stream_armed && s_stream_owner == owner;
  if (ok) {
    *epoch = s_stream_epoch;
    *sequence = s_datagram_sequence++;
  }
  taskEXIT_CRITICAL(&s_stream_mux);
  return ok;
}

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");
extern const char espargos_logo_svg_start[]
    asm("_binary_espargos_logo_svg_start");
extern const char espargos_logo_svg_end[]
    asm("_binary_espargos_logo_svg_end");

static esp_err_t send_static_asset(httpd_req_t *req, const char *type,
                                   const char *cache_control,
                                   const char *start, const char *end) {
  httpd_resp_set_type(req, type);
  httpd_resp_set_hdr(req, "Cache-Control", cache_control);
  httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
  return httpd_resp_send(req, start, end - start - 1u);
}

static esp_err_t index_handler(httpd_req_t *req) {
  return send_static_asset(req, "text/html; charset=utf-8",
                           "no-cache", index_html_start, index_html_end);
}

static esp_err_t logo_handler(httpd_req_t *req) {
  return send_static_asset(req, "image/svg+xml", "public, max-age=3600",
                           espargos_logo_svg_start, espargos_logo_svg_end);
}

static void start_http_server(void) {
  if (s_http_server != NULL)
    return;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = IQ_NETWORK_HTTP_PORT;
  config.stack_size = 6144;
  /* TX staging, EMAC RX, and TinyUSB deliberately cooperate at priority 23.
   * The per-batch UDP arm must join that yield set: leaving HTTP at priority
   * 20 starves the continuation publish while the internal TX ring is full,
   * then drains the ring at every host-batch boundary. The server blocks when
   * idle, so this does not consume data-plane CPU between requests. */
  config.task_priority = 23;
  config.task_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  /* Keep control off the dedicated core-1 PARLIO producer. At 40 MSa/s that
   * priority-24 producer is continuously runnable, so an unpinned HTTP task
   * can accept a TCP connection yet never execute its handler. */
  config.core_id = 0;
  config.max_uri_handlers = 9;
  ESP_ERROR_CHECK(httpd_start(&s_http_server, &config));
  const httpd_uri_t handlers[] = {
      {.uri = "/", .method = HTTP_GET, .handler = index_handler},
      {.uri = "/espargos-logo.svg",
       .method = HTTP_GET,
       .handler = logo_handler},
      {.uri = "/api/v1/status",
       .method = HTTP_GET,
       .handler = status_get_handler},
      {.uri = "/api/v1/config",
       .method = HTTP_GET,
       .handler = config_get_handler},
      {.uri = "/api/v1/config",
       .method = HTTP_PUT,
       .handler = config_put_handler},
      {.uri = "/api/v1/tx/waveform",
       .method = HTTP_PUT,
       .handler = tx_waveform_put_handler},
      {.uri = "/api/v1/tx/udp/arm",
       .method = HTTP_POST,
       .handler = tx_udp_arm_handler},
      {.uri = "/api/v1/stream/start",
       .method = HTTP_POST,
       .handler = stream_start_handler},
      {.uri = "/api/v1/stream/stop",
       .method = HTTP_POST,
       .handler = stream_stop_handler},
  };
  for (size_t i = 0; i < sizeof(handlers) / sizeof(handlers[0]); ++i) {
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &handlers[i]));
  }
}

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id,
                              void *data) {
  (void)arg;
  (void)base;
  (void)data;
  if (id == ETHERNET_EVENT_CONNECTED) {
    s_link_up = true;
    eth_speed_t speed = ETH_SPEED_10M;
    (void)esp_eth_ioctl(s_eth_handle, ETH_CMD_G_SPEED, &speed);
    ESP_LOGI(TAG, "Ethernet link up at %u Mbit/s",
             speed == ETH_SPEED_10M    ? 10u
             : speed == ETH_SPEED_100M ? 100u
                                       : 1000u);
  }
  if (id == ETHERNET_EVENT_DISCONNECTED || id == ETHERNET_EVENT_STOP) {
    ESP_LOGI(TAG, "Ethernet link down");
    s_link_up = false;
    s_has_ipv4 = false;
    s_ipv4_address = 0;
  }
}

static void got_ip_handler(void *arg, esp_event_base_t base, int32_t id,
                           void *data) {
  (void)arg;
  (void)base;
  (void)id;
  ip_event_got_ip_t *event = data;
  s_ipv4_address = event->ip_info.ip.addr;
  s_has_ipv4 = true;
  ESP_LOGI(TAG, "DHCP IPv4 address: " IPSTR, IP2STR(&event->ip_info.ip));
  start_http_server();
}

static void ethernet_resume_task(void *arg) {
  (void)arg;
  while (true) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!__atomic_exchange_n(&s_ethernet_resume_pending, false,
                             __ATOMIC_ACQ_REL)) {
      continue;
    }
    esp_err_t err = esp_eth_start(s_eth_handle);
    if (err == ESP_OK) {
      __atomic_store_n(&s_ethernet_suspended_for_usb_tx, false,
                       __ATOMIC_RELEASE);
      ESP_LOGI(TAG, "Ethernet resumed after native USB TX");
    } else {
      ESP_LOGW(TAG, "could not resume Ethernet after USB TX: %s",
               esp_err_to_name(err));
      __atomic_store_n(&s_ethernet_resume_pending, true, __ATOMIC_RELEASE);
      xTaskNotifyGive(s_ethernet_resume_task_handle);
    }
  }
}

void iq_control_init(const iq_network_callbacks_t *callbacks) {
  s_callbacks = *callbacks;
  s_stream_epoch = esp_random();
  if (s_stream_epoch == 0)
    s_stream_epoch = 1;

  const esp_timer_create_args_t stream_arm_timer_args = {
      .callback = stream_arm_timer_callback,
      .name = "iq_stream_arm",
  };
  ESP_ERROR_CHECK(
      esp_timer_create(&stream_arm_timer_args, &s_stream_arm_timer));
}

void iq_network_init(const iq_network_callbacks_t *callbacks) {
  iq_control_init(callbacks);

  esp_err_t eth_err = ethernet_driver_init();
  if (eth_err != ESP_OK) {
    ESP_LOGW(TAG, "Ethernet unavailable (%s); continuing without network",
             esp_err_to_name(eth_err));
    return;
  }
  esp_netif_config_t config = ESP_NETIF_DEFAULT_ETH();
  s_eth_netif = esp_netif_new(&config);
  ESP_ERROR_CHECK(s_eth_netif != NULL ? ESP_OK : ESP_ERR_NO_MEM);
  esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_eth_handle);
  ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));
  ESP_ERROR_CHECK(esp_eth_update_input_path_info(
      s_eth_handle, iq_eth_input, s_eth_netif));
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                             eth_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                             got_ip_handler, NULL));
  ESP_ERROR_CHECK(mdns_init());
  ESP_ERROR_CHECK(mdns_hostname_set(IQ_NETWORK_HOSTNAME));
  ESP_ERROR_CHECK(mdns_instance_name_set("ESP-SDR"));
  ESP_ERROR_CHECK(mdns_service_add("ESP-SDR HTTP", "_http", "_tcp",
                                   IQ_NETWORK_HTTP_PORT, NULL, 0));
  ESP_ERROR_CHECK(
      xTaskCreatePinnedToCoreWithCaps(
          ethernet_resume_task, "eth_usb_resume", 3072, NULL, 4,
          &s_ethernet_resume_task_handle, 0,
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS
          ? ESP_OK
          : ESP_ERR_NO_MEM);
  ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
  ESP_LOGI(TAG, "Ethernet started; waiting for link and DHCP");
}

void iq_network_suspend_for_usb_tx(void) {
  __atomic_store_n(&s_ethernet_resume_pending, false, __ATOMIC_RELEASE);
  if (s_eth_handle == NULL ||
      __atomic_load_n(&s_ethernet_suspended_for_usb_tx, __ATOMIC_ACQUIRE)) {
    return;
  }
  esp_err_t err = esp_eth_stop(s_eth_handle);
  if (err == ESP_OK) {
    __atomic_store_n(&s_ethernet_suspended_for_usb_tx, true,
                     __ATOMIC_RELEASE);
    ESP_LOGI(TAG, "Ethernet paused for native USB TX");
  } else {
    ESP_LOGW(TAG, "could not pause Ethernet for USB TX: %s",
             esp_err_to_name(err));
  }
}

uint32_t iq_network_usb_tx_stage_buffers(uint32_t **buffers,
                                         uint32_t max_buffers,
                                         uint32_t *words_per_buffer) {
  if (buffers == NULL || words_per_buffer == NULL || max_buffers == 0u ||
      s_eth_handle == NULL ||
      !__atomic_load_n(&s_ethernet_suspended_for_usb_tx, __ATOMIC_ACQUIRE)) {
    return 0u;
  }
  /* The private prefixes above intentionally mirror this repository's pinned
   * ESP-IDF version. esp_eth_stop() has quiesced DMA here, so exposing the RX
   * buffer addresses cannot race the EMAC receive task. esp_eth_start() later
   * rebuilds descriptor ownership before accepting frames. */
  iq_eth_driver_prefix_t *driver = (iq_eth_driver_prefix_t *)s_eth_handle;
  iq_emac_mac_prefix_t *mac = (iq_emac_mac_prefix_t *)driver->mac;
  iq_emac_dma_prefix_t *dma = mac != NULL ? mac->emac_dma_hndl : NULL;
  if (dma == NULL) {
    return 0u;
  }
  uint32_t count = CONFIG_ETH_DMA_RX_BUFFER_NUM;
  if (count > max_buffers) {
    count = max_buffers;
  }
  for (uint32_t i = 0u; i < count; ++i) {
    if (dma->rx_buf[i] == NULL) {
      return 0u;
    }
    buffers[i] = (uint32_t *)dma->rx_buf[i];
  }
  *words_per_buffer = CONFIG_ETH_DMA_BUFFER_SIZE / sizeof(uint32_t);
  return count;
}

void iq_network_resume_after_usb_tx(void) {
  if (s_eth_handle == NULL ||
      !__atomic_load_n(&s_ethernet_suspended_for_usb_tx, __ATOMIC_ACQUIRE)) {
    return;
  }
  __atomic_store_n(&s_ethernet_resume_pending, true, __ATOMIC_RELEASE);
  if (s_ethernet_resume_task_handle != NULL) {
    xTaskNotifyGive(s_ethernet_resume_task_handle);
  }
}

bool iq_network_stream_armed(void) { return s_stream_armed; }

bool iq_network_send_frames(const uint8_t *const *frames,
                            const size_t *frame_sizes,
                            uint32_t frame_count) {
  if (frames == NULL || frame_sizes == NULL || frame_count == 0u ||
      frame_count > CONFIG_ETH_DMA_TX_BUFFER_NUM) {
    return false;
  }
  uint8_t fragment_counts[CONFIG_ETH_DMA_TX_BUFFER_NUM];
  uint32_t total_fragments = 0u;
  for (uint32_t frame_index = 0u; frame_index < frame_count; ++frame_index) {
    if (frames[frame_index] == NULL || frame_sizes[frame_index] < 8u) {
      return false;
    }
    uint32_t fragments =
        (frame_sizes[frame_index] + IQ_UDP_FRAGMENT_PAYLOAD_BYTES - 1u) /
        IQ_UDP_FRAGMENT_PAYLOAD_BYTES;
    if (fragments == 0u || fragments > UINT8_MAX ||
        total_fragments + fragments > CONFIG_ETH_DMA_TX_BUFFER_NUM) {
      return false;
    }
    fragment_counts[frame_index] = (uint8_t)fragments;
    total_fragments += fragments;
  }
  struct sockaddr_in destination;
  uint8_t destination_mac[6];
  uint32_t epoch;
  taskENTER_CRITICAL(&s_stream_mux);
  bool armed = s_stream_armed && s_stream_owner == IQ_STREAM_OWNER_ETH;
  destination = s_stream_destination;
  memcpy(destination_mac, s_stream_destination_mac, sizeof(destination_mac));
  epoch = s_stream_epoch;
  taskEXIT_CRITICAL(&s_stream_mux);
  if (!armed || !s_link_up || !s_has_ipv4)
    return false;

  iq_eth_driver_prefix_t *driver = (iq_eth_driver_prefix_t *)s_eth_handle;
  if (xSemaphoreTake(driver->transmit_mutex,
                     pdMS_TO_TICKS(IQ_ETH_TX_MUTEX_TIMEOUT_MS)) == pdFALSE) {
    ++s_udp_send_errors;
    return false;
  }
  iq_emac_mac_prefix_t *emac = (iq_emac_mac_prefix_t *)driver->mac;
  iq_emac_dma_prefix_t *dma = emac->emac_dma_hndl;
  eth_dma_tx_descriptor_t *first_desc = dma->tx_desc;
  eth_dma_tx_descriptor_t *desc = first_desc;
  uint32_t descriptor_deadline = esp_cpu_get_cycle_count() + 80000000u;
  uint32_t scheduler_yield_deadline =
      esp_cpu_get_cycle_count() + 320000u;
  while (true) {
    bool available = true;
    desc = first_desc;
    for (uint32_t i = 0u; i < total_fragments; ++i) {
      (void)esp_cache_msync(desc, EMAC_HAL_DMA_DESC_SIZE,
                            ESP_CACHE_MSYNC_FLAG_DIR_M2C);
      available &= desc->TDES0.Own == EMAC_LL_DMADESC_OWNER_CPU;
      desc = (eth_dma_tx_descriptor_t *)desc->Buffer2NextDescAddr;
    }
    if (available)
      break;
    /* A stop request runs in the higher-priority HTTP task. Drop the
     * in-progress, not-yet-owned descriptor batch immediately so the normal
     * Ethernet driver can acquire its mutex and transmit the stop response. */
    if (!s_stream_armed || s_stream_owner != IQ_STREAM_OWNER_ETH) {
      xSemaphoreGive(driver->transmit_mutex);
      return false;
    }
    if ((int32_t)(esp_cpu_get_cycle_count() - descriptor_deadline) >= 0) {
      ++s_udp_send_errors;
      xSemaphoreGive(driver->transmit_mutex);
      return false;
    }
    /* The GMAC enters TX-buffer-unavailable when it catches the CPU-owned
     * tail of the short descriptor ring.  Re-kick it while earlier
     * descriptors complete. Normal turnover takes microseconds; only block
     * a scheduler tick after one millisecond of continuous pressure so HTTP
     * stop/control remains responsive during a genuine overload. */
    emac_hal_transmit_poll_demand(&dma->hal);
    uint32_t now = esp_cpu_get_cycle_count();
    if ((int32_t)(now - scheduler_yield_deadline) >= 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      scheduler_yield_deadline = esp_cpu_get_cycle_count() + 320000u;
    } else {
      esp_rom_delay_us(2u);
    }
  }

  /* Every descriptor is inspected exactly once immediately before reuse.
   * The DMA replaces these status bits on each transmission, so the counters
   * describe completed descriptors rather than polling iterations. */
  desc = first_desc;
  for (uint32_t i = 0u; i < total_fragments; ++i) {
    ++s_eth_tx_desc_completed;
    if (desc->TDES0.ErrSummary != 0u)
      ++s_eth_tx_desc_errors;
    if (desc->TDES0.UnderflowErr != 0u)
      ++s_eth_tx_desc_underflows;
    if (desc->TDES0.FrameFlushed != 0u)
      ++s_eth_tx_desc_flushed;
    if (desc->TDES0.NoCarrier != 0u || desc->TDES0.LossCarrier != 0u)
      ++s_eth_tx_desc_carrier_errors;
    desc = (eth_dma_tx_descriptor_t *)desc->Buffer2NextDescAddr;
  }

  uint8_t *packet = s_tx_frame + ETH_HEADER_BYTES + IPV4_HEADER_BYTES +
                    UDP_HEADER_BYTES;
  iq_udp_header_t *header = (iq_udp_header_t *)packet;
  desc = first_desc;
  for (uint32_t frame_index = 0u; frame_index < frame_count; ++frame_index) {
    const uint8_t *frame = frames[frame_index];
    size_t frame_bytes = frame_sizes[frame_index];
    uint32_t frame_sequence;
    memcpy(&frame_sequence, frame + 4u, sizeof(frame_sequence));
    uint32_t source_chunk = 0u;
    uint32_t dropped = s_callbacks.get_firmware_dropped_chunks();
    bool iq_frame =
        memcmp(frame, STREAM_FRAME_MAGIC_IQ, 4u) == 0 ||
        memcmp(frame, STREAM_FRAME_MAGIC_IQ8, 4u) == 0 ||
        memcmp(frame, STREAM_FRAME_MAGIC_REAL8, 4u) == 0;
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
    uint32_t fragment_count = fragment_counts[frame_index];
    for (uint32_t i = 0u, offset = 0u; i < fragment_count; ++i) {
      uint32_t remaining = frame_bytes - offset;
      uint16_t payload_bytes = remaining > IQ_UDP_FRAGMENT_PAYLOAD_BYTES
                                   ? IQ_UDP_FRAGMENT_PAYLOAD_BYTES
                                   : (uint16_t)remaining;
      *header = (iq_udp_header_t){
          .magic = {'I', 'Q', 'U', '1'},
          .version = IQ_UDP_VERSION,
          .header_bytes = sizeof(*header),
          .stream_epoch = epoch,
          .datagram_sequence = s_datagram_sequence,
          .frame_sequence = frame_sequence,
          .source_chunk_index = source_chunk,
          .frame_bytes = frame_bytes,
          .frame_crc32 = frame_crc,
          .fragment_offset = offset,
          .fragment_bytes = payload_bytes,
          .fragment_index = i,
          .fragment_count = fragment_count,
          .firmware_dropped_chunks = dropped,
      };
      memcpy(header->frame_magic, frame, 4u);
      header->header_crc32 = esp_rom_crc32_le(
          0u, packet, offsetof(iq_udp_header_t, header_crc32));
      size_t packet_bytes = sizeof(*header) + payload_bytes;
      build_udp_frame(&destination, destination_mac, packet_bytes,
                      (uint16_t)s_datagram_sequence);
      const uint32_t prefix_bytes =
          ETH_HEADER_BYTES + IPV4_HEADER_BYTES + UDP_HEADER_BYTES +
          sizeof(*header);
      uint8_t *dma_buffer = (uint8_t *)desc->Buffer1Addr;
      memcpy(dma_buffer, s_tx_frame, prefix_bytes);
      memcpy(dma_buffer + prefix_bytes, frame + offset, payload_bytes);
      uint32_t ethernet_bytes = prefix_bytes + payload_bytes;
      desc->TDES0.Value &=
          ~(IQ_EMAC_TDES0_FS_FLAGS_MASK | IQ_EMAC_TDES0_LS_FLAGS_MASK);
      desc->TDES0.FirstSegment = 1u;
      desc->TDES0.LastSegment = 1u;
      desc->TDES0.Value |=
          dma->tx_desc_flags &
          (IQ_EMAC_TDES0_FS_FLAGS_MASK | IQ_EMAC_TDES0_LS_FLAGS_MASK);
      desc->TDES1.TransmitBuffer1Size = ethernet_bytes;
      uint32_t cache_bytes = (ethernet_bytes + 63u) & ~63u;
      (void)esp_cache_msync(dma_buffer, cache_bytes,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M);
      desc = (eth_dma_tx_descriptor_t *)desc->Buffer2NextDescAddr;
      ++s_datagram_sequence;
      ++s_udp_datagrams;
      s_udp_bytes += (uint32_t)packet_bytes;
      offset += payload_bytes;
    }
    ++s_udp_frames;
  }
  desc = first_desc;
  for (uint32_t i = 0u; i < total_fragments; ++i) {
    desc->TDES0.Own = EMAC_LL_DMADESC_OWNER_DMA;
    (void)esp_cache_msync(desc, EMAC_HAL_DMA_DESC_SIZE,
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    desc = (eth_dma_tx_descriptor_t *)desc->Buffer2NextDescAddr;
  }
  dma->tx_desc = desc;
  emac_hal_transmit_poll_demand(&dma->hal);
  s_eth_dma_status = emac_hal_get_intr_status(&dma->hal);
  xSemaphoreGive(driver->transmit_mutex);
  return true;
}

bool iq_network_send_frame(const uint8_t *frame, size_t frame_bytes) {
  const uint8_t *frames[1] = {frame};
  size_t frame_sizes[1] = {frame_bytes};
  return iq_network_send_frames(frames, frame_sizes, 1u);
}

bool iq_network_send_frame_int8(const uint8_t *frame, size_t frame_bytes) {
  if (frame == NULL || frame_bytes != sizeof(iq_chunk_t) ||
      memcmp(frame, STREAM_FRAME_MAGIC_IQ, 4u) != 0) {
    return false;
  }
  memcpy(s_iq8_frame, frame, offsetof(iq_chunk_t, samples));
  s_iq8_frame[3] = '8';
  const uint32_t *source = (const uint32_t *)(
      frame + offsetof(iq_chunk_t, samples));
  uint32_t *packed = (uint32_t *)(
      s_iq8_frame + offsetof(iq_chunk_t, samples));
  for (uint32_t sample = 0u; sample < IQ_CHUNK_SAMPLE_WORDS; sample += 2u) {
    uint32_t w0 = source[sample];
    uint32_t w1 = source[sample + 1u];
    uint32_t p0 = ((w0 >> 12) & 0xffu) | ((w0 << 6) & 0xff00u);
    uint32_t p1 = ((w1 >> 12) & 0xffu) | ((w1 << 6) & 0xff00u);
    packed[sample / 2u] = p0 | (p1 << 16);
  }
  memset(s_iq8_frame + IQ8_FRAME_WIRE_BYTES - sizeof(uint32_t), 0,
         sizeof(uint32_t));
  return iq_network_send_frame(s_iq8_frame, sizeof(s_iq8_frame));
}
