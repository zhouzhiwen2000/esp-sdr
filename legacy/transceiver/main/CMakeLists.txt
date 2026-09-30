set(esp_sdr_supported_targets esp32c5 esp32s3 esp32s31 esp32c61)
if(NOT IDF_TARGET IN_LIST esp_sdr_supported_targets)
    message(FATAL_ERROR "No ESP-SDR RF backend for ${IDF_TARGET}. Generic board firmware is available for esp32c5 and esp32s3; esp32s31/esp32c61 use their existing specialized backends.")
endif()

option(SAMPLE_RATE_PROBE "Build volatile hardware sample-rate diagnostics (never package)" OFF)

if(IDF_TARGET STREQUAL "esp32s3")
    idf_component_register(SRCS "s3_main.c" "s3_serial.c"
        REQUIRES esp_phy esp_wifi nvs_flash esp_timer esp_driver_usb_serial_jtag esp_driver_uart)
    if(SAMPLE_RATE_PROBE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE SAMPLE_RATE_PROBE=1)
    endif()
    if(FILTER_REGISTER_PROBE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE FILTER_REGISTER_PROBE=1)
    endif()
    if(S3_RF_PROBE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE S3_RF_PROBE=1)
    endif()
    target_link_options(${COMPONENT_LIB} INTERFACE "-T${CMAKE_CURRENT_LIST_DIR}/s3_sram_guard.ld")
    return()
endif()

if(IDF_TARGET STREQUAL "esp32c5")
    idf_component_register(SRCS "c5_main.c"
        REQUIRES esp_phy esp_wifi nvs_flash esp_timer esp_driver_usb_serial_jtag)
    if(SAMPLE_RATE_PROBE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE SAMPLE_RATE_PROBE=1)
    endif()
    if(FILTER_REGISTER_PROBE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE FILTER_REGISTER_PROBE=1)
    endif()
    target_link_options(${COMPONENT_LIB} INTERFACE
        "-T${CMAKE_CURRENT_LIST_DIR}/c5_sram_guard.ld")
    if(C5_REPLAY_PROBE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE C5_REPLAY_PROBE=1)
    endif()
    if(C5_TUNE_PROBE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE C5_TUNE_PROBE=1)
    endif()
    return()
endif()

set(esp_sdr_sources
    "main.c" "wifi_tx_rx.c" "ringbuffer.c" "modem.c" "iq_network.c"
    "dcoc.c" "gaintable.c" "sdr_agc.c" "s31_pie_copy.S")

set(esp_sdr_requires
    heap esp_timer esp_event esp_mm esp_psram nvs_flash esp_driver_gpio esp_driver_parlio
    esp_driver_gptimer esp_driver_dma esp_hw_support esp_hal_wdt esp_wifi
    esp_eth esp_netif esp_http_server espressif__cjson espressif__mdns lwip
    espressif__esp_tinyusb esp-open-phy-lib esp32c61-phy-regs ulp)

if(CONFIG_ESP_SDR_TRANSPORT_USB)
  list(APPEND esp_sdr_sources "iq_usb.c")
endif()

idf_component_register(SRCS ${esp_sdr_sources}
                       REQUIRES ${esp_sdr_requires}
                       EMBED_TXTFILES "web/index.html" "web/espargos-logo.svg")

# gaintable.c snapshots the boot-calibrated RX gain table by intercepting the
# open PHY lib's table writers.
target_link_options(${COMPONENT_LIB} INTERFACE
                    "-Wl,--wrap=phy_set_rx_gain_table"
                    "-Wl,--wrap=phy_write_gain_mem"
                    "-Wl,--wrap=emac_esp_dma_alloc_recv_buf")

if(CONFIG_IDF_TARGET_ESP32S31 AND NOT CONFIG_ESP_SDR_S31_CONTINUOUS_REAL_IF)
  set(ulp_app_name s31_tcm_probe)
  set(ulp_lp_core_sources "ulp/main.c")
  set(ulp_exp_dep_srcs "main.c")
  ulp_embed_binary(${ulp_app_name} "${ulp_lp_core_sources}" "${ulp_exp_dep_srcs}")
endif()
