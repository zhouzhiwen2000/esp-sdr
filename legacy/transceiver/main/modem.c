#include "modem.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_cpu.h"
#include "esp_phy_cert_test.h"
#include "esp_phy_init.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "hal/wdt_hal.h"
#include "sdkconfig.h"
#include "soc/hp_system_reg.h"
#include "soc/interrupts.h"
#if CONFIG_IDF_TARGET_ESP32S31
#include "soc/lp_system_reg.h"
#else
#include "soc/lpperi_reg.h"
#endif

#include "gaintable.h"
#include "phy_pbus_reg_names.h"
#include "phy_regs.h"
#include "reg_helpers.h"

#define HZ_PER_KHZ 1000u
#define HZ_PER_MHZ 1000000u

extern void phy_force_rx_gain(uint32_t enable, uint32_t gain);
extern void phy_rfrx_sat_rst(uint32_t enable);
extern void phy_set_rx_gain_table(uint32_t freq_mhz, uint32_t debug);
extern uint32_t phy_i2c_readReg(uint32_t block, uint32_t host_id, uint32_t reg_addr);
extern void phy_i2c_writeReg(uint32_t block, uint32_t host_id, uint32_t reg_addr, uint32_t data);
extern uint8_t phy_param[];
extern void phy_loopback_mode_en(uint32_t en);
extern void phy_pbus_debugmode(void);
extern void phy_pbus_force_test(uint32_t block, uint32_t bank, uint32_t value);
extern uint32_t phy_pbus_rd(uint32_t block, uint32_t bank);
extern void phy_pbus_workmode(void);
extern void phy_pbus_xpd_rx_on(uint32_t en);
extern void phy_pbus_xpd_tx_off(void);
extern void phy_txcal_debuge_mode_(void);
extern void phy_pbus_rx_dco_cal(uint32_t arg0, uint32_t *arg1, uint32_t arg2, uint32_t arg3,
                                uint32_t arg4);
extern void phy_bb_bss_cbw40_dig(uint32_t enable);
extern void phy_bb_cbw_chan_cfg(uint32_t cfg);
extern void phy_bt_filter_reg(void);
extern void ble_rx_start(uint32_t phy, uint32_t radio_mode);
extern void bt_bb_v2_rx_set(uint32_t rx_comp);
extern void force_iq_set(uint32_t enable, uint32_t iq_sel, uint32_t dc,
                         uint32_t reserved);
extern void phy_mac_tx_chan_offset(uint32_t offset);
extern void phy_set_loopback_gain(uint32_t tx_gain, uint32_t rx_gain, uint32_t bb_gain);
extern void phy_chip_set_chan(uint32_t channel_or_freq, uint32_t cbw);
extern void phy_set_freq(uint32_t freq_mhz, int32_t freq_offset_khz);
extern void phy_set_rxclk_en(uint32_t en);
extern void phy_set_txclk_en(uint32_t en);
extern void phy_dac_rate_set(uint32_t value);
extern void phy_start_tx_tone_step(uint32_t tone0_en, int32_t tone0_step, uint32_t tone0_phase,
                                   uint32_t tone1_en, int32_t tone1_step,
                                   uint32_t tone1_phase);
extern void phy_stop_tx_tone(uint32_t enable);
extern void phy_wifi_fbw_sel(uint32_t enable);
extern void phy_write_gain_mem(uint32_t word0, uint32_t word1, uint32_t word2, uint32_t index);

typedef enum {
    PHY_SIGNAL_PATH_NORMAL = 0,
    PHY_SIGNAL_PATH_LOOPBACK = 1,
} phy_signal_path_t;

static modem_config_t s_config;
static uint32_t s_rf_freq_hz = RF_FREQ_DEFAULT_HZ;
static uint32_t s_current_rf_freq_hz;
static uint32_t s_current_cbw40 = UINT32_MAX;
static phy_signal_path_t s_phy_signal_path = PHY_SIGNAL_PATH_NORMAL;
static bool s_tx_tone_running;
static bool s_external_tx_env;
static bool s_external_tone_running;
static uint32_t s_tx_tone0_step;
static bool s_bbtop_filter_defaults_valid;
static uint8_t s_bbtop_wifirx0_filter_defaults[2];
static uint8_t s_bbtop_bt_filter_defaults[9];
static uint32_t s_fe_rx_filter_mode_default;
static uint32_t s_agc_force_bt_mode_default;
static uint32_t s_fe_bt_filter_default;
static uint32_t s_fe_bt_rx_force_default;
static bool s_agc_gain_defaults_valid;
static uint32_t s_agc_gain_init_default;
static uint32_t s_agc_power_high_threshold_default;

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

static uint32_t clamp_freq_hz(uint32_t freq_hz)
{
    return clamp_u32(freq_hz, RF_FREQ_MIN_HZ, RF_FREQ_MAX_HZ);
}

static void disable_rtc_wdt(void)
{
    wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
    wdt_hal_write_protect_disable(&rtc_wdt_ctx);
    wdt_hal_set_flashboot_en(&rtc_wdt_ctx, false);
    wdt_hal_feed(&rtc_wdt_ctx);
    wdt_hal_disable(&rtc_wdt_ctx);
    wdt_hal_write_protect_enable(&rtc_wdt_ctx);
}

static void disable_peri_timeout_protectors(void)
{
#if CONFIG_IDF_TARGET_ESP32S31
    reg32_set_bits_addr(LP_SYSTEM_REG_LP_PERI_TIMEOUT_CONF_REG,
                        LP_SYSTEM_REG_LP_PERI_TIMEOUT_INT_CLEAR);
    reg32_clear_bits_addr(LP_SYSTEM_REG_LP_PERI_TIMEOUT_CONF_REG,
                          LP_SYSTEM_REG_LP_PERI_TIMEOUT_PROTECT_EN);
    reg32_set_bits_addr(HP_SYSTEM_CPU_PERI0_TIMEOUT_CONF_REG,
                        HP_SYSTEM_CPU_PERI0_TIMEOUT_INT_CLR);
    reg32_clear_bits_addr(HP_SYSTEM_CPU_PERI0_TIMEOUT_CONF_REG,
                          HP_SYSTEM_CPU_PERI0_TIMEOUT_PROTECT_EN);
    reg32_set_bits_addr(HP_SYSTEM_CPU_PERI1_TIMEOUT_CONF_REG,
                        HP_SYSTEM_CPU_PERI1_TIMEOUT_INT_CLR);
    reg32_clear_bits_addr(HP_SYSTEM_CPU_PERI1_TIMEOUT_CONF_REG,
                          HP_SYSTEM_CPU_PERI1_TIMEOUT_PROTECT_EN);
    reg32_set_bits_addr(HP_SYSTEM_HP_SYSTEM_PERI0_TIMEOUT_CONF_REG,
                        HP_SYSTEM_HP_SYSTEM_PERI0_TIMEOUT_INT_CLR);
    reg32_clear_bits_addr(HP_SYSTEM_HP_SYSTEM_PERI0_TIMEOUT_CONF_REG,
                          HP_SYSTEM_HP_SYSTEM_PERI0_TIMEOUT_PROTECT_EN);
    reg32_set_bits_addr(HP_SYSTEM_HP_SYSTEM_PERI1_TIMEOUT_CONF_REG,
                        HP_SYSTEM_HP_SYSTEM_PERI1_TIMEOUT_INT_CLR);
    reg32_clear_bits_addr(HP_SYSTEM_HP_SYSTEM_PERI1_TIMEOUT_CONF_REG,
                          HP_SYSTEM_HP_SYSTEM_PERI1_TIMEOUT_PROTECT_EN);
#else
    reg32_set_bits_addr(LPPERI_BUS_TIMEOUT_REG, LPPERI_LP_PERI_TIMEOUT_INT_CLEAR);
    reg32_clear_bits_addr(LPPERI_BUS_TIMEOUT_REG, LPPERI_LP_PERI_TIMEOUT_PROTECT_EN);
    reg32_set_bits_addr(HP_SYSTEM_CPU_PERI_TIMEOUT_CONF_REG,
                        HP_SYSTEM_CPU_PERI_TIMEOUT_INT_CLEAR);
    reg32_clear_bits_addr(HP_SYSTEM_CPU_PERI_TIMEOUT_CONF_REG,
                          HP_SYSTEM_CPU_PERI_TIMEOUT_PROTECT_EN);
    reg32_set_bits_addr(HP_SYSTEM_HP_PERI_TIMEOUT_CONF_REG,
                        HP_SYSTEM_HP_PERI_TIMEOUT_INT_CLEAR);
    reg32_clear_bits_addr(HP_SYSTEM_HP_PERI_TIMEOUT_CONF_REG,
                          HP_SYSTEM_HP_PERI_TIMEOUT_PROTECT_EN);
#endif
}

static void mask_lp_peri_timeout_interrupt(void)
{
#if CONFIG_IDF_TARGET_ESP32S31
    reg32_set_bits_addr(LP_SYSTEM_REG_LP_PERI_TIMEOUT_CONF_REG,
                        LP_SYSTEM_REG_LP_PERI_TIMEOUT_INT_CLEAR);
#else
    reg32_set_bits_addr(LPPERI_BUS_TIMEOUT_REG, LPPERI_LP_PERI_TIMEOUT_INT_CLEAR);
#endif
    esp_rom_route_intr_matrix(esp_cpu_get_core_id(), ETS_LP_PERI_TIMEOUT_INTR_SOURCE,
                              ETS_INVALID_INUM);
}

void modem_prepare_direct_phy_access(void)
{
    disable_rtc_wdt();
    disable_peri_timeout_protectors();
    mask_lp_peri_timeout_interrupt();
}

static uint32_t config_cbw40(const modem_config_t *config)
{
    if (config->bw_mhz < 40u) {
        return 0u;
    }
    return config->second_chan == SECOND_CHAN_BELOW ? 3u : 2u;
}

static void test_filter_band_set_direct(uint32_t enable_40mhz)
{
    uint8_t value = enable_40mhz != 0u ? phy_param[239] : phy_param[240];
    phy_i2c_writeReg(103u, 1u, 28u, value);
    phy_i2c_writeReg(103u, 1u, 29u, value);
}

static void apply_wifi_bandwidth_config(const modem_config_t *config)
{
    uint32_t cbw40 = config_cbw40(config);
    uint32_t enable = cbw40 != 0u ? 1u : 0u;

    phy_bb_bss_cbw40_dig(enable);
    test_filter_band_set_direct(enable);
    phy_bb_cbw_chan_cfg(cbw40);
    phy_mac_tx_chan_offset(cbw40);
    phy_wifi_fbw_sel(enable);
    phy_bt_filter_reg();
}

static void bbtop_write(uint32_t reg_addr, uint32_t value)
{
    phy_i2c_writeReg(103u, 1u, reg_addr, value);
}

static void bbtop_write_pair(uint32_t reg0, uint32_t reg1, uint32_t value)
{
    bbtop_write(reg0, value);
    bbtop_write(reg1, value);
}

static uint8_t bbtop_read(uint32_t reg_addr)
{
    return (uint8_t)phy_i2c_readReg(103u, 1u, reg_addr);
}

static void capture_bbtop_filter_defaults(void)
{
    s_fe_rx_filter_mode_default =
        reg32p_read_field(&MODEM_WIFI_FE_DATA.RX_FILTER_MODE, MODEM_WIFI_FE_RX_FILTER_MODE_M,
                          MODEM_WIFI_FE_RX_FILTER_MODE_S);
    for (uint32_t i = 0; i < 2u; ++i) {
        s_bbtop_wifirx0_filter_defaults[i] = bbtop_read(4u + i);
    }
    s_bbtop_bt_filter_defaults[0] = bbtop_read(3u);
    for (uint32_t i = 0; i < 8u; ++i) {
        s_bbtop_bt_filter_defaults[i + 1u] = bbtop_read(20u + i);
    }
    s_agc_force_bt_mode_default =
        reg32p_read_field(&MODEM_WIFI_AGC.AGCBT_CTRL1, 0x000000c0u, 6u);
    s_fe_bt_filter_default =
        reg32p_read(&MODEM_WIFI_FE_CTRL.BT_FILTER_WIFI_BW_SEL) & 0x03c00000u;
    s_fe_bt_rx_force_default =
        reg32p_read(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL) & 0x00003300u;
    s_bbtop_filter_defaults_valid = true;
}

static void restore_bbtop_filter_defaults(void)
{
    if (!s_bbtop_filter_defaults_valid) {
        capture_bbtop_filter_defaults();
    }
    reg32p_write_field(&MODEM_WIFI_FE_DATA.RX_FILTER_MODE, MODEM_WIFI_FE_RX_FILTER_MODE_M,
                       MODEM_WIFI_FE_RX_FILTER_MODE_S,
                       s_fe_rx_filter_mode_default);
    for (uint32_t i = 0; i < 2u; ++i) {
        bbtop_write(4u + i, s_bbtop_wifirx0_filter_defaults[i]);
    }
    bbtop_write(3u, s_bbtop_bt_filter_defaults[0]);
    for (uint32_t i = 0; i < 8u; ++i) {
        bbtop_write(20u + i, s_bbtop_bt_filter_defaults[i + 1u]);
    }
    reg32p_write_field(&MODEM_WIFI_AGC.AGCBT_CTRL1, 0x000000c0u, 6u,
                       s_agc_force_bt_mode_default);
    reg32p_write_bits(&MODEM_WIFI_FE_CTRL.BT_FILTER_WIFI_BW_SEL, 0x03c00000u,
                      s_fe_bt_filter_default);
    reg32p_write_bits(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL, 0x00003300u,
                      s_fe_bt_rx_force_default);
}

static void apply_wifirx0_filter_dcap(uint32_t dcap)
{
    bbtop_write_pair(4u, 5u, dcap);
}

static void force_bt_rx_analog_path(void)
{
    /* Exact register sequence used by S31 librftest bt_rx_force(1, 1). */
    reg32p_write_field(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL,
                       0x00003000u, 12u, 3u);
    esp_rom_delay_us(10u);
    reg32p_write_field(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL,
                       0x00000300u, 8u, 2u);
    esp_rom_delay_us(1u);
    reg32p_write_field(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL,
                       0x00000300u, 8u, 3u);
    esp_rom_delay_us(1u);
}

static void apply_vendor_dump_interface_mode(uint32_t loop_mode)
{
    /* S31 librftest set_dump_mode().  This is a separate producer/interface
     * mux from ADC_DUMP_MODE.SOURCE_SEL and was missing from the earlier
     * source-only scan.  Mode zero selects the normal FE dump interface;
     * every nonzero vendor mode selects its alternate/loop interface. */
    reg32_clear_bits_addr(PHY_MODEM_BASE_ADDR + 0x08ccu, 0x00780000u);
    uint32_t value = reg32_read_addr(PHY_MODEM_BASE_ADDR + 0x70b8u);
    value &= ~0x7u;
    if (loop_mode == 0u) {
        value |= 1u;
    }
    reg32_write_addr(PHY_MODEM_BASE_ADDR + 0x70b8u, value);
}

static void apply_chip724_bt_loopback_route(void)
{
    /* Exact analog-loop route used by the archived CHIP724 filter_rx_loop()
     * characterization.  It is intentionally diagnostic: unlike the OTA
     * route it closes the on-chip TX->BTRX path, giving us a controlled tone
     * with which to identify the downstream dump source/formatter. */
    force_iq_set(1u, 1u, 0u, 0u);
    force_iq_set(1u, 0u, 0u, 0u);
    phy_pbus_debugmode();
    phy_pbus_force_test(PHY_PBUS_BLOCK_RFRX1, PHY_PBUS_BANK_EN1, 0x104u);
    phy_pbus_force_test(PHY_PBUS_BLOCK_RFTX1, PHY_PBUS_BANK_EN1, 0x07fu);
    phy_pbus_force_test(PHY_PBUS_BLOCK_RFTX2, PHY_PBUS_BANK_EN1, 0x00cu);
    phy_pbus_force_test(PHY_PBUS_BLOCK_BB, PHY_PBUS_BANK_EN1, 0x1fbu);
    phy_pbus_force_test(PHY_PBUS_BLOCK_BB, PHY_PBUS_BANK_EN2, 0u);
    phy_pbus_force_test(PHY_PBUS_BLOCK_RFTX1, PHY_PBUS_BANK_EN2, 0x18u);
    bbtop_write(0u, bbtop_read(0u) | BIT(6));
}

static void apply_bt_filter_probe(const modem_config_t *config)
{
    const uint32_t bt_if_ctrl =
        (config->rx_filter_override == 32u ||
         config->rx_filter_override == 33u)
            ? (config->rx_filter_mode & 3u)
            : 2u;
    /* CHIP724's OTA RX-filter test forces the AGC into BT mode independently
     * of starting the packet receiver. This is the missing routing dimension
     * in the earlier dump-source-only scan. */
    reg32p_write_field(&MODEM_WIFI_AGC.AGCBT_CTRL1, 0x000000c0u, 6u, 3u);

    /* FLT_BT_IF_CTRL is [25:24]: 0=low-pass, 1=0.9 MHz, 2=1.0 MHz,
     * 3=1.1 MHz in BLE-1M mode.  Overrides 32/33 expose all four values via
     * rx_filter_mode; older diagnostics retain the known 1.0 MHz value.
     * Keep the adjacent 2M-family selector bits [23:22] clear. */
    reg32p_write_bits(&MODEM_WIFI_FE_CTRL.BT_FILTER_WIFI_BW_SEL,
                      0x03c00000u, bt_if_ctrl << 24u);

    /* OTA mode uses the real BTRX bank with the complex calibration loop
     * disconnected. Keep the PHY-calibrated per-stage DCap topology here:
     * BTRX0=(dcap2+6), BTRX1=(dcap2-2,dcap2), BTRX2=(dcap3,dcap3), followed
     * by the first BTTX pair. Writing one value into all eight registers was
     * not equivalent to the vendor calibration and can detune the cascade. */
    /* CHIP724 moved encmplx_btrx from bit 3 to bit 2 when its RX DCap banks
     * shrank to six bits.  S31 has the same six-bit/multi-bank topology, so
     * clear both historical locations for the OTA route.  Earlier probes
     * cleared only bit 3 and could therefore have left the BTRX calibration
     * loop selected. */
    bbtop_write(3u, bbtop_read(3u) & ~((uint32_t)BIT(2) | BIT(3)));
    force_bt_rx_analog_path();

    if (config->rx_filter_override >= 4u) {
        /* Complete the special setup used by librftest adctrig mode 12 and
         * start its BLE receive producer. Overrides 2/3 deliberately omit
         * this so the dependency can be isolated on hardware. */
        /* rftest_open_clk() runs before any BLE test-mode access. Without
         * these gates ble_rx_start stalls on a clocked-off BTLC register and
         * HP WDT1 resets the chip. */
        reg32_set_bits_addr(PHY_MODEM_BASE_ADDR + 0x9c04u, 0xfffff000u);
        reg32_write_addr(PHY_MODEM_BASE_ADDR + 0x9c0cu, UINT32_MAX);
        reg32_write_addr(PHY_MODEM_BASE_ADDR + 0x9c14u, UINT32_MAX);
        reg32_write_addr(PHY_MODEM_BASE_ADDR + 0xf018u, UINT32_MAX);
        reg32_write_addr(PHY_MODEM_BASE_ADDR + 0xf01cu, UINT32_MAX);
        reg32_write_addr(PHY_MODEM_BASE_ADDR + 0xf020u, UINT32_MAX);
        if (config->rx_filter_override >= 6u) {
            /* rftest_init() normally reaches this through
             * bt_bb_v2_init_cmplx().  ble_rx_start() alone only programs the
             * link-controller test event; it does not initialize the BT RX
             * baseband, AGC, correlator, DPO, or digital filter selectors. */
            bt_bb_v2_rx_set(4u);
            reg32p_write_field(&MODEM_WIFI_AGC.AGCBT_CTRL1, 0x000000c0u, 6u,
                               3u);
            reg32p_write_bits(&MODEM_WIFI_FE_CTRL.BT_FILTER_WIFI_BW_SEL,
                              0x03c00000u, bt_if_ctrl << 24u);
            force_bt_rx_analog_path();
        }
        reg32_clear_bits_addr(PHY_MODEM_BASE_ADDR + 0x9c04u, BIT(21));
        reg32_set_bits_addr(PHY_MODEM_BASE_ADDR + 0x20b4u, BIT(0));
        uint32_t bt_ctrl_2b = reg32_read_addr(PHY_MODEM_BASE_ADDR + 0x20acu);
        reg32_write_addr(PHY_MODEM_BASE_ADDR + 0x20acu,
                         (bt_ctrl_2b & 0x1fffffffu) | 0x40000000u);
        ble_rx_start(0u, 0u);

        /* Exact CHIP724 filter_rx_dcap_sweep PBUS routing prerequisites. */
        phy_pbus_debugmode();
        phy_pbus_force_test(PHY_PBUS_BLOCK_BB, PHY_PBUS_BANK_EN1,
                            phy_pbus_rd(PHY_PBUS_BLOCK_BB,
                                        PHY_PBUS_BANK_EN1) | 2u);
        phy_pbus_force_test(PHY_PBUS_BLOCK_RFTX1, PHY_PBUS_BANK_EN2,
                            phy_pbus_rd(PHY_PBUS_BLOCK_RFTX1,
                                        PHY_PBUS_BANK_EN2) | 0x18u);

        if (config->rx_filter_override == 10u ||
            config->rx_filter_override == 11u ||
            config->rx_filter_override == 14u ||
            config->rx_filter_override == 15u ||
            (config->rx_filter_override >= 20u &&
             config->rx_filter_override <= 27u) ||
            config->rx_filter_override == 28u ||
            config->rx_filter_override == 29u ||
            config->rx_filter_override == 30u ||
            config->rx_filter_override == 31u) {
            /* CHIP724/S31 candidate layout: filter_cmplx_ctrl[1:0],
             * encmplx_btrx[2]. */
            uint32_t value = bbtop_read(3u);
            uint32_t complex_ctrl = (config->rx_filter_override == 28u ||
                                     config->rx_filter_override == 29u)
                                        ? 0u
                                        : (config->rx_filter_mode & 3u);
            value = (value & ~0x0fu) | BIT(2) | complex_ctrl;
            bbtop_write(3u, value);
        } else if (config->rx_filter_override == 12u ||
                   config->rx_filter_override == 13u) {
            /* CHIP722/723 candidate layout retained as an A/B control:
             * filter_cmplx_ctrl[2:0], encmplx_btrx[3]. */
            uint32_t value = bbtop_read(3u);
            value = (value & ~0x1fu) | BIT(3) | (config->rx_filter_mode & 7u);
            bbtop_write(3u, value);
        }

        if (config->rx_filter_override >= 16u &&
            config->rx_filter_override <= 35u) {
            const bool alternate_dump_interface =
                config->rx_filter_override == 18u ||
                config->rx_filter_override == 19u ||
                config->rx_filter_override == 22u ||
                config->rx_filter_override == 23u ||
                config->rx_filter_override == 26u ||
                config->rx_filter_override == 27u;
            apply_vendor_dump_interface_mode(alternate_dump_interface ? 1u : 0u);
        }
        if (config->rx_filter_override >= 24u &&
            config->rx_filter_override <= 35u) {
            apply_chip724_bt_loopback_route();
        }
    }
}

/* Map an SDR-style two-sided analog bandwidth request to the WifiRX0 6-bit
 * filter capacitor DAC. The interpolation exposes the DAC's useful resolution
 * instead of presenting a handful of presets. */
static uint8_t filter_bw_to_dcap(uint32_t bw_mhz)
{
    static const struct {
        uint8_t dcap;
        uint8_t bw_mhz;
    } cal[] = {
        {0u, 54u}, {8u, 36u}, {16u, 30u}, {28u, 21u},
        {32u, 18u}, {48u, 15u}, {60u, 13u},
    };
    const uint32_t n = sizeof(cal) / sizeof(cal[0]);

    if (bw_mhz == RX_FILTER_BW_OPEN || bw_mhz >= cal[0].bw_mhz) {
        return cal[0].dcap;
    }
    if (bw_mhz <= cal[n - 1u].bw_mhz) {
        return cal[n - 1u].dcap;
    }
    for (uint32_t i = 1u; i < n; ++i) {
        if (bw_mhz >= cal[i].bw_mhz) {
            uint32_t bw_hi = cal[i - 1u].bw_mhz;
            uint32_t span = bw_hi - cal[i].bw_mhz;
            uint32_t dspan = cal[i].dcap - cal[i - 1u].dcap;
            return (uint8_t)(cal[i - 1u].dcap +
                (dspan * (bw_hi - bw_mhz) + span / 2u) / span);
        }
    }
    return cal[n - 1u].dcap;
}

static void apply_rx_bandwidth_config(const modem_config_t *config)
{
    restore_bbtop_filter_defaults();

    if (config->rx_filter_override == 0u) {
        apply_wifirx0_filter_dcap(filter_bw_to_dcap(config->filter_bw_mhz));
        return;
    }

    if (config->rx_filter_override >= 36u) {
        /* Wi-Fi dump-byte census: vary only the downstream debug formatter.
         * Keep the calibrated antenna/FE route identical to production. */
        apply_wifirx0_filter_dcap(filter_bw_to_dcap(config->filter_bw_mhz));
        return;
    }

    if (config->rx_filter_override >= 2u) {
        apply_bt_filter_probe(config);
        return;
    }

    if (config->rx_filter_mode < 16u) {
        reg32p_write_field(&MODEM_WIFI_FE_DATA.RX_FILTER_MODE, MODEM_WIFI_FE_RX_FILTER_MODE_M,
                           MODEM_WIFI_FE_RX_FILTER_MODE_S, config->rx_filter_mode);
    }

    apply_wifirx0_filter_dcap(config->rx_filter_dcap);
}

static void set_txon_for_tone(uint32_t enable)
{
    reg32p_clear_bits(&MODEM_WIFI_AGC.AGCCCA_CTRL0, BIT(16));

    if (enable != 0u) {
        reg32p_write_field(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL, 0x00000300u, 8u, 2u);
        esp_rom_delay_us(1);
        reg32p_set_bits(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL, 0x00000c00u);
    } else {
        if (reg32p_read_field(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL, 0x00000c00u, 10u) == 3u) {
            reg32p_write_field(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL, 0x00000c00u, 10u, 2u);
            esp_rom_delay_us(1);
        }
        reg32p_clear_bits(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL, 0x00000c00u);
        esp_rom_delay_us(1);
        reg32p_clear_bits(&MODEM_WIFI_FE_CTRL.ADC_ON_TXON_CTRL, 0x00000300u);
    }
    esp_rom_delay_us(1);
}

static void set_rx_init_gain(uint32_t gain)
{
    reg32p_write_field(&MODEM_WIFI_AGC.AGCPWR_CTRL25, 0x01fc0000u, 18u, gain);
    reg32p_write_field(&MODEM_WIFI_AGC.AGCGAIN_CTRL_4, 0x000001fcu, 2u, gain);
}

static void split_rf_freq(uint32_t freq_hz, uint32_t *freq_mhz, int32_t *freq_offset_khz)
{
    uint32_t rounded_mhz = (freq_hz + (HZ_PER_MHZ / 2u)) / HZ_PER_MHZ;
    uint32_t center_hz = rounded_mhz * HZ_PER_MHZ;
    int32_t offset_hz = (int32_t)freq_hz - (int32_t)center_hz;
    *freq_mhz = rounded_mhz;
    *freq_offset_khz = offset_hz >= 0 ? (offset_hz + 500) / (int32_t)HZ_PER_KHZ :
                                        (offset_hz - 500) / (int32_t)HZ_PER_KHZ;
}

static uint32_t corrected_rf_freq_hz(uint32_t requested_hz,
                                    int32_t correction_ppb)
{
    requested_hz = clamp_freq_hz(requested_hz);
    int64_t product = (int64_t)requested_hz * correction_ppb;
    int64_t correction_hz = product >= 0
                                ? (product + 500000000LL) / 1000000000LL
                                : (product - 500000000LL) / 1000000000LL;
    return (uint32_t)((int64_t)requested_hz - correction_hz);
}

static void tune_rf(uint32_t freq_hz)
{
    freq_hz = clamp_freq_hz(freq_hz);
    uint32_t programmed_hz = corrected_rf_freq_hz(
        freq_hz, s_config.frequency_correction_ppb);
    uint32_t cbw40 = config_cbw40(&s_config);
    s_rf_freq_hz = freq_hz;
    if (s_current_rf_freq_hz == programmed_hz && s_current_cbw40 == cbw40) {
        return;
    }

    uint32_t freq_mhz;
    int32_t freq_offset_khz;
    split_rf_freq(programmed_hz, &freq_mhz, &freq_offset_khz);

    /* Tune SDR centers directly in MHz.  phy_set_chanfreq() first maps an
     * arbitrary frequency through the integer Wi-Fi-channel abstraction;
     * that path contributed about 18 dB of additional, repeatable
     * center-dependent TX loss around 2.46 GHz even after phy_set_freq()
     * applied the nominal center.  The S31
     * PHY accepts values above 2411 as MHz here, matching the direct strategy
     * used by the C61 IQ transceiver.  phy_set_freq() then applies the signed
     * sub-MHz remainder. */
    phy_chip_set_chan(freq_mhz, cbw40);
    phy_set_freq(freq_mhz, freq_offset_khz);
    apply_wifi_bandwidth_config(&s_config);
    s_current_rf_freq_hz = programmed_hz;
    s_current_cbw40 = cbw40;
}

void modem_calibrate_dco(void)
{
    uint32_t seed[10] = {0x00010001u, 0x00010001u};
    phy_pbus_rx_dco_cal(0x1000u, seed, 10u, 0u, 0u);
}

static void stop_tx_tone_generator(void)
{
    if (s_external_tone_running) {
        uint32_t channel = (s_config.rf_freq_hz <= 2412u * HZ_PER_MHZ)
                               ? 1u
                               : (s_config.rf_freq_hz - 2407u * HZ_PER_MHZ +
                                  2500000u) /
                                     (5u * HZ_PER_MHZ);
        if (channel > 14u) {
            channel = 14u;
        }
        esp_phy_wifi_tx_tone(0u, channel, 0u);
        s_external_tone_running = false;
    }
    phy_start_tx_tone_step(0, 0, 0, 0, 0, 0);
    phy_stop_tx_tone(1);
    phy_stop_tx_tone(0);
    s_tx_tone_running = false;
}

static void start_tx_tone_generator(int32_t tone0_step)
{
    uint32_t tone_ctrl = reg32p_read(&MODEM_WIFI_FE_DATA.TX_MEASURE_PWDET_TONE_CTRL);
    uint32_t scale_shift = (tone_ctrl & BIT(29)) != 0u ? 7u : 5u;
    uint32_t step_mask = (tone_ctrl & BIT(29)) != 0u ? 0xfffu : 0x3ffu;
    int32_t tone_step = (tone0_step << scale_shift) / 5;

    phy_start_tx_tone_step(1, tone_step & (int32_t)step_mask, 0, 0, 0, 0);
    s_tx_tone_running = true;
    s_tx_tone0_step = tone0_step;
}

static void disable_loopback_signal_path(void)
{
    esp_rom_delay_us(50);
    phy_loopback_mode_en(0);
    esp_rom_delay_us(50);
}

static void enter_normal_signal_path(void)
{
    disable_loopback_signal_path();
    phy_pbus_workmode();
    phy_pbus_xpd_rx_on(1u);
    phy_set_rxclk_en(1u);
    s_phy_signal_path = PHY_SIGNAL_PATH_NORMAL;
}

static void enter_loopback_signal_path(bool calibrate)
{
    uint32_t seed[10] = {0x00010001u, 0x00010001u};

    phy_set_txclk_en(1);
    phy_set_rxclk_en(1);
    phy_pbus_debugmode();
    phy_pbus_xpd_rx_on(0);
    phy_loopback_mode_en(1);
    phy_set_loopback_gain(s_config.loopback_tx_gain, s_config.loopback_rx_gain,
                          s_config.loopback_bb_gain);
    if (calibrate) {
        phy_pbus_rx_dco_cal(0x800u, seed, 10u, 0u, 0u);
    }
    s_phy_signal_path = PHY_SIGNAL_PATH_LOOPBACK;
}

static void apply_loopback_gain_config(const modem_config_t *config)
{
    phy_pbus_debugmode();
    phy_set_loopback_gain(config->loopback_tx_gain, config->loopback_rx_gain,
                          config->loopback_bb_gain);
}

static void apply_tx_tone_config(void)
{
    if (s_config.tx_tone_enable == 1u) {
        if (s_config.loopback == 0u) {
            uint32_t channel = (s_config.rf_freq_hz <= 2412u * HZ_PER_MHZ)
                                   ? 1u
                                   : (s_config.rf_freq_hz -
                                      2407u * HZ_PER_MHZ + 2500000u) /
                                         (5u * HZ_PER_MHZ);
            if (channel > 14u) {
                channel = 14u;
            }
            if (!s_external_tone_running) {
                /* Use the complete vendor certification path, including
                 * channel interpolation and calibrated TX setup. */
                esp_phy_wifi_tx_tone(1u, channel, 0u);
                s_external_tone_running = true;
                s_tx_tone_running = true;
                s_tx_tone0_step = s_config.tx_tone0_step;
            }
        } else {
            phy_set_txclk_en(1u);
            set_txon_for_tone(1u);
            if (!s_tx_tone_running ||
                s_tx_tone0_step != s_config.tx_tone0_step) {
                start_tx_tone_generator(s_config.tx_tone0_step);
            }
        }
    } else if (s_config.tx_tone_enable == 2u) {
        /* Memory-to-DAC replay.  main.c owns the replay SRAM and
         * MACTOADCDUMP engine; the modem side only supplies clocks, TXON and a
         * quiet normal baseband source. */
        stop_tx_tone_generator();
        if (s_config.loopback == 0u) {
            phy_txcal_debuge_mode_();
        } else {
            phy_set_txclk_en(1u);
        }
        set_txon_for_tone(1u);
    } else {
        stop_tx_tone_generator();
        set_txon_for_tone(0);
        phy_set_txclk_en(0u);
    }
}

static void stop_tx_tone_keep_loopback_path(void)
{
    stop_tx_tone_generator();
    set_txon_for_tone(0);
    phy_set_txclk_en(1u);
}

static void write_expert_gain_slots(const modem_config_t *config)
{
    phy_write_gain_mem(config->expert_gain_word0, config->expert_gain_word1,
                       config->expert_gain_word2, EXPERT_GAIN_SLOT);
    phy_write_gain_mem(config->expert_gain_word0, config->expert_gain_word1,
                       config->expert_gain_word2, EXPERT_GAIN_SLOT_SECOND_TABLE);
}

/* Unlike the C61, the S31 gain RAM ignores writes from the live workmode RX
 * path; it only accepts them in the pbus-debug-mode / clocks-forced
 * environment that phy_wr_rx_gain_mem_new sets up (verified on hardware:
 * un-bracketed entry rewrites leave the analog DC unchanged, bracketed ones
 * step it). Enter that environment for gain-mem writes, then restore the
 * operating signal path. */
static void gain_mem_write_env_enter(void)
{
    phy_pbus_debugmode();
    phy_pbus_xpd_rx_on(0);
    phy_set_rxclk_en(1u);
    phy_set_txclk_en(1u);
}

static void gain_mem_write_env_exit(void)
{
    if (s_phy_signal_path == PHY_SIGNAL_PATH_NORMAL) {
        phy_set_txclk_en(s_config.tx_tone_enable != 0u ? 1u : 0u);
        phy_pbus_workmode();
        phy_pbus_xpd_rx_on(1u);
        phy_set_rxclk_en(1u);
    }
    /* loopback: pbus debug mode with xpd_rx off IS the operating state */
}

/* Write a gain-mem entry to both tables and re-force the gain index so the
 * FE re-latches the entry, including its DC compensation DAC codes (they
 * only reach the DACs on an index (re-)apply). Used by the DCOC servo
 * (dcoc.c); the edits are capture-session-local, the tables are rebuilt on
 * the next engine restart (same lifecycle as the expert slots). */
void modem_dcoc_write_entry(uint32_t slot, uint32_t w0, uint32_t w1, uint32_t w2)
{
    gain_mem_write_env_enter();
    phy_write_gain_mem(w0, w1, w2, slot);
    phy_write_gain_mem(w0, w1, w2, slot + EXPERT_GAIN_SLOT_SECOND_TABLE);
    gain_mem_write_env_exit();
    phy_force_rx_gain(1u, slot);
    set_rx_init_gain(slot);
}

static void capture_agc_gain_defaults(void)
{
    s_agc_gain_init_default = reg32p_read(&MODEM_WIFI_AGC.AGCGAIN_CTRL_4);
    s_agc_power_high_threshold_default = reg32p_read(&MODEM_WIFI_AGC.AGCPWR_CTRL25);
    s_agc_gain_defaults_valid = true;
}

static void restore_agc_gain_defaults(void)
{
    if (!s_agc_gain_defaults_valid) {
        capture_agc_gain_defaults();
    }
    reg32p_write(&MODEM_WIFI_AGC.AGCGAIN_CTRL_4, s_agc_gain_init_default);
    reg32p_write(&MODEM_WIFI_AGC.AGCPWR_CTRL25, s_agc_power_high_threshold_default);
}

void modem_force_rx_gain_slot(uint32_t slot)
{
    gaintable_entry_t entry;
    if (slot < GAINTABLE_MAX_ENTRIES &&
        gaintable_get_entry((uint8_t)slot, &entry)) {
        gain_mem_write_env_enter();
        phy_write_gain_mem(entry.word0, entry.word1, entry.word2,
                           slot + EXPERT_GAIN_SLOT_SECOND_TABLE);
        gain_mem_write_env_exit();
    }
    phy_force_rx_gain(1u, slot);
    set_rx_init_gain(slot);
}

void modem_select_rx_gain_slot_fast(uint32_t slot)
{
    phy_force_rx_gain(1u, slot);
    set_rx_init_gain(slot);
}

void modem_prepare_sdr_agc_gain_table(void)
{
    uint32_t count = gaintable_entry_count();
    gain_mem_write_env_enter();
    for (uint32_t slot = 0u; slot < count; ++slot) {
        gaintable_entry_t entry;
        if (gaintable_get_entry((uint8_t)slot, &entry)) {
            phy_write_gain_mem(entry.word0, entry.word1, entry.word2,
                               slot + EXPERT_GAIN_SLOT_SECOND_TABLE);
        }
    }
    gain_mem_write_env_exit();
}

static void apply_gain_config(const modem_config_t *config)
{
    /* The packet AGC's RF saturation protection steps front-end gain down on
     * ADC clipping INDEPENDENTLY of the forced gain word (observed on the
     * C61 sensors: under a strong CW, units randomly dropped ~20 dB for
     * seconds with the forced gain index unchanged; at signal levels below
     * clipping the drops vanish). In the forced-gain modes clipping is the
     * operator's choice and the samples are raw data, not packets. The SDR
     * auto mode also uses explicit forced slots, so disable this hidden
     * intervention in every mode. */
    phy_rfrx_sat_rst(0u);
    if (config->gain_mode == GAIN_MODE_EXPERT) {
        write_expert_gain_slots(config);
        phy_force_rx_gain(1u, EXPERT_GAIN_SLOT);
        set_rx_init_gain(EXPERT_GAIN_SLOT);
    } else if (config->gain_mode == GAIN_MODE_MANUAL) {
        /* Mirror the forced entry into the SECOND gain table before forcing:
         * the FE reads table 2 in some AGC-FSM contexts (11b/DSSS detection),
         * and beyond the high table's calibrated entries those slots still
         * hold init-time filler words at a wildly different gain. The DCOC
         * servo also mirrors on its writes, but only acts when there is DC
         * error to correct. */
        modem_force_rx_gain_slot(config->rx_gain);
    } else {
        restore_agc_gain_defaults();
        phy_force_rx_gain(0u, 0u);
    }
}

esp_err_t modem_init(const modem_config_t *config)
{
    s_config = *config;
    capture_bbtop_filter_defaults();
    capture_agc_gain_defaults();
    tune_rf(s_config.rf_freq_hz);
    return ESP_OK;
}

void modem_apply_rx_config(const modem_config_t *config)
{
    s_config = *config;
    if (s_config.loopback == 0u && s_config.tx_tone_enable != 0u &&
        !s_external_tx_env) {
        /* The Ethernet build's Wi-Fi MAC is unassociated and only supplies
         * the initialized PHY/sniffer environment. Keep that environment
         * alive while the short replay temporarily owns the PBUS route;
         * stop+direct-enable cannot restore the S31 live RX diagnostic bus
         * without a complete chip reset. */
        s_external_tx_env = true;
        s_current_rf_freq_hz = 0u;
    } else if (s_external_tx_env &&
               (s_config.loopback != 0u || s_config.tx_tone_enable == 0u)) {
        /* TX calibration/debug mode occasionally leaves the live receive
         * diagnostic producer stopped even after restoring PBUS work mode.
         * A PHY sleep/wake cycle resets that hidden producer without tearing
         * down the Wi-Fi driver's scarce heap allocations or touching the
         * independent Ethernet control plane.  Keep the Wi-Fi task and modem
         * interrupts isolated: resuming them with replay-mutated MAC state is
         * unsafe, while direct I/Q only needs the initialized PHY fabric. */
        modem_recover_rx_after_tx_replay();
        /* Recovery now follows the final RX configuration, so later
         * gain-only/config fast paths do not need another PHY cycle. */
        s_external_tx_env = false;
    }
    if (s_config.loopback != 0u) {
        if (s_config.tx_tone_enable == 1u) {
            bool tone_was_running = s_tx_tone_running &&
                                    s_tx_tone0_step == s_config.tx_tone0_step &&
                                    s_current_rf_freq_hz == corrected_rf_freq_hz(
                                        s_config.rf_freq_hz,
                                        s_config.frequency_correction_ppb);
            if (!tone_was_running) {
                stop_tx_tone_generator();
                phy_set_txclk_en(1u);
                s_current_rf_freq_hz = 0u;
                tune_rf(s_config.rf_freq_hz);
            }
            enter_loopback_signal_path(false);
            apply_tx_tone_config();
        } else {
            tune_rf(s_config.rf_freq_hz);
            enter_loopback_signal_path(false);
        }
    } else {
        enter_normal_signal_path();
        if (s_config.tx_tone_enable == 0u) {
            phy_pbus_xpd_tx_off();
        }
        phy_set_txclk_en(s_config.tx_tone_enable != 0u ? 1u : 0u);
        phy_set_rxclk_en(1u);
        tune_rf(s_config.rf_freq_hz);
    }
    if (s_config.loopback != 0u) {
        phy_set_txclk_en(1u);
        phy_set_rxclk_en(1u);
    }
    if (s_config.loopback != 0u && s_config.tx_tone_enable == 2u) {
        apply_tx_tone_config();
    } else if (s_config.loopback != 0u && s_config.tx_tone_enable == 0u) {
        stop_tx_tone_keep_loopback_path();
    } else if (s_config.loopback == 0u || s_config.tx_tone_enable == 0u) {
        apply_tx_tone_config();
    }
    apply_gain_config(&s_config);
    apply_wifi_bandwidth_config(&s_config);
    apply_rx_bandwidth_config(&s_config);
}

void modem_apply_live_rx_config(const modem_config_t *config)
{
    s_config = *config;
    if (config->loopback != 0u) {
        apply_loopback_gain_config(config);
    }
    apply_gain_config(config);
}

void modem_prepare_tx_replay(const modem_config_t *config)
{
    /* The RF synthesizer, channel bandwidth and normal signal path were
     * already applied while TX was disabled.  Publish the replay-mode config
     * so modem_rearm_tx_replay() can switch the calibrated PBUS route, TX
     * clock, gain and TXON without repeating the multi-second full PHY setup. */
    s_config = *config;
    if (s_config.loopback == 0u) {
        s_external_tx_env = true;
    }
}

void modem_rearm_tx_replay(void)
{
    if (s_config.tx_tone_enable != 2u) {
        return;
    }

    /* adctrig_prepare() resets the S31 data-dump fabric after the normal
     * modem configuration pass.  Reassert the complete replay TX state only
     * after that reset: normal (non-tone) baseband selection, TX clocks,
     * calibrated TX debug mode for the antenna path, and forced TXON. */
    stop_tx_tone_generator();
    /* The replay-rate experiment is encoded in ADC_DUMP_MODE by main.c.
     * Keep the analog DAC rate at its calibrated default; sweeping this field
     * independently produced no change in replay cadence. */
    phy_dac_rate_set(0u);
    phy_set_txclk_en(1u);
    if (s_config.loopback == 0u) {
        phy_txcal_debuge_mode_();
        /* Keep the calibrated TX routing/DCO state, but override its RF gain
         * with the host-selected conservative code. Calling
         * phy_pbus_xpd_tx_on() here would also replace the calibrated DCO and
         * baseband gain, so update only the RFTX2 gain word. */
        phy_pbus_force_test(PHY_PBUS_BLOCK_RFTX2, PHY_PBUS_BANK_EN1,
                            (s_config.tx_gain + 448u) & 0xffffu);
    }
    set_txon_for_tone(1u);
}

void modem_stop_tx_replay(void)
{
    set_txon_for_tone(0u);
    phy_set_txclk_en(0u);
}

void modem_recover_rx_after_tx_replay(void)
{
    if (!s_external_tx_env) {
        return;
    }
    stop_tx_tone_generator();
    set_txon_for_tone(0u);
    esp_phy_disable(PHY_MODEM_WIFI);
    esp_rom_delay_us(1000u);
    esp_phy_enable(PHY_MODEM_WIFI);
    esp_rom_delay_us(1000u);
    /* Keep the latch set for a possible later RX backend reconfiguration.
     * modem_apply_rx_config() repeats recovery after applying that path and
     * then clears it. This eager cycle still covers unchanged RX activation. */
    s_current_rf_freq_hz = 0u;
}

/* cal_state_flags lives at byte offset 164 in the phy_param block (asserted in
 * phy_param.h). Bit 0x200 latches "RX gain table already generated". */
#define PHY_PARAM_CAL_STATE_FLAGS_OFFSET 164u
#define PHY_CAL_FLAG_RX_GAIN_TABLE_DONE 0x200u

void modem_setup_rx_gain_table(void)
{
    /* Populate the per-slot RX gain ramp in AGC gain memory. phy_init() normally
     * does this, but our direct-PHY-access path never runs it, so the manual
     * rx_gain slot index maps to a flat (~2-level) gain. phy_set_rx_gain_table()
     * only (re)writes the ramp when cal_state_flags bit 0x200 is clear, and the
     * engine/FE bring-up can leave the gain memory stale, so force a regen. */
    *(volatile uint32_t *)(phy_param + PHY_PARAM_CAL_STATE_FLAGS_OFFSET) &=
        ~(uint32_t)PHY_CAL_FLAG_RX_GAIN_TABLE_DONE;
    uint32_t freq_mhz = (s_rf_freq_hz + (HZ_PER_MHZ / 2u)) / HZ_PER_MHZ;
    phy_set_rx_gain_table(freq_mhz, 0u);
}

void modem_enter_debug_mode(void)
{
    phy_pbus_debugmode();
}

void modem_enter_work_mode(void)
{
    phy_pbus_workmode();
}

uint32_t modem_rf_freq_hz(void)
{
    return s_rf_freq_hz;
}
