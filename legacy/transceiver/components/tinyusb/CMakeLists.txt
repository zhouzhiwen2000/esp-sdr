# Component-manager-free alias for espressif/tinyusb. esp_tinyusb refers to
# this dependency by its registry name, "tinyusb".
set(tinyusb_root "${CMAKE_CURRENT_LIST_DIR}/../../managed_components/espressif__tinyusb")
idf_build_get_property(target IDF_TARGET)
string(TOUPPER "OPT_MCU_${target}" tusb_mcu)

set(tinyusb_sources
    "src/class/cdc/cdc_device.c"
    "src/class/hid/hid_device.c"
    "src/class/midi/midi_device.c"
    "src/class/midi/midi2_device.c"
    "src/class/msc/msc_device.c"
    "src/class/mtp/mtp_device.c"
    "src/class/printer/printer_device.c"
    "src/class/vendor/vendor_device.c"
    "src/class/audio/audio_device.c"
    "src/class/video/video_device.c"
    "src/class/bth/bth_device.c"
    "src/class/usbtmc/usbtmc_device.c"
    "src/class/net/ecm_rndis_device.c"
    "lib/networking/rndis_reports.c"
    "src/class/net/ncm_device.c"
    "src/class/dfu/dfu_device.c"
    "src/class/dfu/dfu_rt_device.c"
    "src/portable/synopsys/dwc2/dcd_dwc2.c"
    "src/portable/synopsys/dwc2/dwc2_common.c"
    "src/common/tusb_fifo.c"
    "src/device/usbd.c"
    "src/tusb.c")
list(TRANSFORM tinyusb_sources PREPEND "${tinyusb_root}/")

idf_component_register(
    SRCS ${tinyusb_sources}
    INCLUDE_DIRS "${tinyusb_root}/src"
    PRIV_INCLUDE_DIRS "${tinyusb_root}/src" "${tinyusb_root}/src/device"
                      "${tinyusb_root}/lib/networking"
    PRIV_REQUIRES esp_netif
    REQUIRES freertos)

target_compile_definitions(${COMPONENT_LIB} PUBLIC "CFG_TUSB_MCU=${tusb_mcu}")
set_source_files_properties("${tinyusb_root}/src/device/usbd.c"
                            PROPERTIES COMPILE_FLAGS "-Wno-type-limits")
