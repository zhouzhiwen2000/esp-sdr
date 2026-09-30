#include "wifi_tx_rx.h"

#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "soc/interrupt_core0_reg.h"
#include "soc/interrupt_core1_reg.h"
#include "soc/interrupts.h"
#include "soc/soc.h"

#include "app_config.h"

#define HZ_PER_MHZ 1000000u
#define WIFI_REPORT_MIN_INTERVAL_US 5000u
#define WIFI_DUMMY_TX_TASK_STACK_BYTES 1536u
#define WIFI_DUMMY_TX_INTERVAL_MAX_WAIT_MS 100u

static TaskHandle_t s_dummy_tx_task_handle;
static portMUX_TYPE s_config_mux = portMUX_INITIALIZER_UNLOCKED;
static wifi_tx_rx_report_cb_t s_report_cb;
static wifi_tx_rx_config_t s_config;
static volatile bool s_stream_armed;
static volatile uint32_t s_report_sequence = 1;
static volatile uint32_t s_dropped_reports;
static volatile uint32_t s_last_report_us;
static bool s_wifi_started;
static bool s_promiscuous_started;
static bool s_promiscuous_reports_enabled;
#if CONFIG_IDF_TARGET_ESP32S31
static TaskHandle_t s_replay_wifi_task;
static uint32_t s_replay_intr_map[8];
static bool s_replay_wifi_isolated;
#endif

static uint32_t clamp_u32(uint32_t value, uint32_t min, uint32_t max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static wifi_tx_rx_config_t config_snapshot(void)
{
    wifi_tx_rx_config_t config;
    taskENTER_CRITICAL(&s_config_mux);
    config = s_config;
    taskEXIT_CRITICAL(&s_config_mux);
    return config;
}

static uint8_t wifi_channel_from_freq_hz(uint32_t freq_hz)
{
    uint32_t freq_mhz = (freq_hz + (HZ_PER_MHZ / 2u)) / HZ_PER_MHZ;
    if (freq_mhz >= 2412u && freq_mhz <= 2472u) {
        return (uint8_t)clamp_u32((freq_mhz + 2u - 2407u) / 5u, 1u, 13u);
    }
    if (freq_mhz >= 2482u) {
        return 14u;
    }
    return 1u;
}

static wifi_second_chan_t wifi_second_channel_from_config(const wifi_tx_rx_config_t *config)
{
    if (config->bw_mhz < 40u) {
        return WIFI_SECOND_CHAN_NONE;
    }
    if (config->second_chan == SECOND_CHAN_BELOW) {
        return WIFI_SECOND_CHAN_BELOW;
    }
    return WIFI_SECOND_CHAN_ABOVE;
}

static void update_wifi_channel(const wifi_tx_rx_config_t *config)
{
    if (s_wifi_started) {
        wifi_second_chan_t second = wifi_second_channel_from_config(config);
        (void)esp_wifi_set_bandwidth(WIFI_IF_STA,
                                     second == WIFI_SECOND_CHAN_NONE ? WIFI_BW20 : WIFI_BW40);
        (void)esp_wifi_set_channel(wifi_channel_from_freq_hz(config->rf_freq_hz), second);
    }
}

static void copy_mac_or_zero(uint8_t dst[6], const uint8_t *payload, uint32_t payload_len,
                             uint32_t offset)
{
    if (payload_len >= offset + 6u) {
        memcpy(dst, payload + offset, 6u);
    } else {
        memset(dst, 0, 6u);
    }
}

static uint32_t wifi_payload_len(uint32_t frame_len, uint32_t frame_type,
                                 uint32_t frame_subtype, uint32_t frame_control)
{
    uint32_t header_len = 0u;
    if (frame_type == 0u) {
        header_len = 24u;
    } else if (frame_type == 1u) {
        header_len = (frame_subtype == 12u || frame_subtype == 13u) ? 10u : 16u;
    } else if (frame_type == 2u) {
        bool to_ds = (frame_control & (1u << 8)) != 0u;
        bool from_ds = (frame_control & (1u << 9)) != 0u;
        header_len = 24u + (to_ds && from_ds ? 6u : 0u) +
                     ((frame_subtype & 0x08u) != 0u ? 2u : 0u);
    }
    return frame_len > header_len + 4u ? frame_len - header_len - 4u : 0u;
}

static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!s_stream_armed || s_config.stream_packets == 0u ||
        type == WIFI_PKT_MISC || buf == NULL || s_report_cb == NULL) {
        return;
    }
    const wifi_promiscuous_pkt_t *packet = (const wifi_promiscuous_pkt_t *)buf;
    uint32_t timestamp_us = packet->rx_ctrl.timestamp;
    uint32_t last_report_us = s_last_report_us;
    if ((uint32_t)(timestamp_us - last_report_us) < WIFI_REPORT_MIN_INTERVAL_US) {
        ++s_dropped_reports;
        return;
    }
    s_last_report_us = timestamp_us;

    const uint8_t *payload = packet->payload;
    uint32_t frame_len = packet->rx_ctrl.sig_len;
    if (frame_len < 2u) {
        return;
    }

    uint32_t frame_control = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8);
    uint32_t frame_type = (frame_control >> 2) & 0x3u;
    uint32_t frame_subtype = (frame_control >> 4) & 0xfu;

    wifi_packet_report_t report = {
        .magic = {'W', 'P', 'K', '1'},
        .sequence = s_report_sequence++,
        .rx_timestamp_us = timestamp_us,
        .channel = packet->rx_ctrl.channel,
        .rssi = packet->rx_ctrl.rssi,
        .rate = packet->rx_ctrl.rate,
        .sig_mode = packet->rx_ctrl.cur_bb_format,
        .mcs = 0u,
        .frame_type = frame_type,
        .frame_subtype = frame_subtype,
        .frame_control = frame_control,
        .frame_len = frame_len,
        .payload_len = wifi_payload_len(frame_len, frame_type, frame_subtype, frame_control),
        .dropped_reports = s_dropped_reports,
    };
    copy_mac_or_zero(report.receiver_mac, payload, frame_len, 4u);
    copy_mac_or_zero(report.sender_mac, payload, frame_len, 10u);
    copy_mac_or_zero(report.bssid, payload, frame_len, 16u);
    report.crc32 = esp_rom_crc32_le(0u, (const uint8_t *)&report,
                                    offsetof(wifi_packet_report_t, crc32));
    if (!s_report_cb(&report)) {
        ++s_dropped_reports;
    }
}

static void send_dummy_wifi_packet(void)
{
    uint8_t packet[] = {
        0xd0, 0x00,
        0x00, 0x00,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x01,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x00, 0x00,
        0x7f,
        0x18, 0xfe, 0x34,
        0x44, 0x49, 0x51, 0x00,
    };
    uint8_t mac[6];
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
        memcpy(&packet[10], mac, sizeof(mac));
    }
    (void)esp_wifi_80211_tx(WIFI_IF_STA, packet, sizeof(packet), true);
}

static void wifi_dummy_tx_task(void *arg)
{
    (void)arg;
    int64_t next_tx_us = 0;
    while (true) {
        wifi_tx_rx_config_t config = config_snapshot();
        if (config.dummy_tx_enable != 0u && s_wifi_started) {
            int64_t now_us = esp_timer_get_time();
            if (next_tx_us == 0 || now_us >= next_tx_us) {
                send_dummy_wifi_packet();
                next_tx_us = now_us + ((int64_t)config.dummy_tx_interval_ms * 1000);
            }
            uint32_t delay_ms = 1u;
            if (next_tx_us > now_us) {
                uint64_t wait_ms = (uint64_t)(next_tx_us - now_us + 999) / 1000u;
                delay_ms = clamp_u32((uint32_t)wait_ms, 1u,
                                     WIFI_DUMMY_TX_INTERVAL_MAX_WAIT_MS);
            }
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        } else {
            next_tx_us = 0;
            vTaskDelay(pdMS_TO_TICKS(100u));
        }
    }
}

static void update_promiscuous_state(void)
{
    bool want_reports = s_config.stream_packets != 0u;
    bool want_promiscuous =
#if CONFIG_IDF_TARGET_ESP32S31
        true;
#else
        want_reports;
#endif
    if (!want_promiscuous && !s_promiscuous_started) {
        return;
    }
    if (want_promiscuous == s_promiscuous_started &&
        want_reports == s_promiscuous_reports_enabled) {
        return;
    }
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT |
                       WIFI_PROMIS_FILTER_MASK_DATA,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    if (want_reports) {
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_rx_cb));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(NULL));
    }
    s_promiscuous_reports_enabled = want_reports;
    if (want_promiscuous && !s_promiscuous_started) {
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
        s_promiscuous_started = true;
    } else if (!want_promiscuous && s_promiscuous_started) {
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
        s_promiscuous_started = false;
    }
}

bool wifi_tx_rx_pause_for_replay(void)
{
    const bool was_active = s_wifi_started && s_promiscuous_started;
#if CONFIG_IDF_TARGET_ESP32S31
    if (s_wifi_started && !s_replay_wifi_isolated) {
        /* Do not call any Wi-Fi control API here: both sniffer-disable and
         * driver-stop teardown race the independent Ethernet/tcpip tasks
         * through scarce internal heap on this S31 build. Preserve all Wi-Fi
         * allocations and isolate only its already-running execution paths. */
        s_replay_wifi_task = xTaskGetHandle("wifi");
        if (s_replay_wifi_task != NULL) {
            vTaskSuspend(s_replay_wifi_task);
        }
        const uint32_t registers[8] = {
            INTERRUPT_CORE0_MODEM_WIFI_MAC_INTR_MAP_REG,
            INTERRUPT_CORE0_MODEM_WIFI_MAC_NMI_INTR_MAP_REG,
            INTERRUPT_CORE0_MODEM_WIFI_PWR_INTR_MAP_REG,
            INTERRUPT_CORE0_MODEM_WIFI_BB_INTR_MAP_REG,
            INTERRUPT_CORE1_MODEM_WIFI_MAC_INTR_MAP_REG,
            INTERRUPT_CORE1_MODEM_WIFI_MAC_NMI_INTR_MAP_REG,
            INTERRUPT_CORE1_MODEM_WIFI_PWR_INTR_MAP_REG,
            INTERRUPT_CORE1_MODEM_WIFI_BB_INTR_MAP_REG,
        };
        for (size_t i = 0; i < 8; ++i) {
            s_replay_intr_map[i] = REG_READ(registers[i]);
            /* Zero is the S31 interrupt matrix's disconnected target.  Do
             * not use a generic CPU interrupt number here: that routes a
             * live modem source into an unrelated handler and eventually
             * faults core 0 during a long replay. */
            REG_WRITE(registers[i], ETS_INVALID_INUM);
        }
        s_replay_wifi_isolated = true;
    }
#endif
    return was_active;
}

void wifi_tx_rx_resume_after_replay(bool was_active)
{
#if CONFIG_IDF_TARGET_ESP32S31
    /* The vendor Wi-Fi task cannot safely resume after the direct replay path
     * has repurposed its modem state: a pending PHY/MAC callback enters ROM
     * with stale driver context and faults shortly after a longer burst.  IQ
     * RX and direct TX use the modem registers directly, so keep that task and
     * its modem interrupt sources quiesced once arbitrary replay is entered.
     * Wi-Fi packet metadata is consequently unavailable after the first TX,
     * but USB, Ethernet, direct IQ RX, retuning, and later TX remain alive. */
    (void)was_active;
#else
    if (was_active) {
        update_promiscuous_state();
    }
#endif
}

void wifi_tx_rx_init(wifi_tx_rx_report_cb_t report_cb)
{
    s_report_cb = report_cb;
    if (!s_wifi_started) {
        wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
        wifi_config.static_rx_buf_num = 2;
        wifi_config.dynamic_rx_buf_num = 2;
        wifi_config.dynamic_tx_buf_num = 1;
        wifi_config.rx_mgmt_buf_num = 1;
        wifi_config.ampdu_rx_enable = 0;
        wifi_config.ampdu_tx_enable = 0;
        wifi_config.rx_ba_win = 0;
        wifi_config.mgmt_sbuf_num = 6;
        wifi_config.nvs_enable = 0;
        ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        s_wifi_started = true;
        update_wifi_channel(&s_config);
    }
    update_promiscuous_state();
    if (s_dummy_tx_task_handle == NULL) {
        xTaskCreate(wifi_dummy_tx_task, "wifi_dummy_tx", WIFI_DUMMY_TX_TASK_STACK_BYTES,
                    NULL, 8, &s_dummy_tx_task_handle);
    }
}

void wifi_tx_rx_apply_config(const wifi_tx_rx_config_t *config)
{
    taskENTER_CRITICAL(&s_config_mux);
    s_config = *config;
    taskEXIT_CRITICAL(&s_config_mux);
    if (config->stream_packets != 0u || config->dummy_tx_enable != 0u) {
        update_wifi_channel(config);
    }
    if (s_wifi_started) {
        update_promiscuous_state();
    }
}

void wifi_tx_rx_set_stream_armed(bool armed)
{
    s_stream_armed = armed;
}
