cmake_minimum_required(VERSION 3.16)

# C5 uses the fixed USB Serial/JTAG controller and its own RF backend.
if(IDF_TARGET STREQUAL "esp32c5" OR IDF_TARGET STREQUAL "esp32s3")
    set(ENV{IDF_COMPONENT_MANAGER} "0")
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    idf_build_set_property(MINIMAL_BUILD ON)
    project(esp_sdr)
    return()
endif()

# All registry components used by this project are checked into
# managed_components and listed below. Keep normal builds deterministic and
# offline; developers can still opt into the registry by exporting
# IDF_COMPONENT_MANAGER=1 before configuring a fresh build directory.
if(NOT DEFINED ENV{IDF_COMPONENT_MANAGER})
    set(ENV{IDF_COMPONENT_MANAGER} "0")
endif()

list(APPEND EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/esp32c61-phy-regs"
    "${CMAKE_CURRENT_LIST_DIR}/components/esp-open-phy-lib"
    "${CMAKE_CURRENT_LIST_DIR}/components/tinyusb"
)

# Make checked-in managed components available to offline builds, but do not
# pass absent directories to ESP-IDF: when the component manager is enabled it
# must be allowed to download those dependencies during configuration.
foreach(managed_component
        espressif__cjson
        espressif__mdns
        espressif__esp_tinyusb)
    set(managed_component_dir
        "${CMAKE_CURRENT_LIST_DIR}/managed_components/${managed_component}")
    if(EXISTS "${managed_component_dir}/CMakeLists.txt")
        list(APPEND EXTRA_COMPONENT_DIRS "${managed_component_dir}")
    endif()
endforeach()

set(CONFIG_OPEN_WIFI_USE_IDF_WIFI_BASELINE y)
set(CONFIG_OPEN_WIFI_ENABLE_BLOB_WRAPS y)
set(CONFIG_OPEN_WIFI_ENABLE_WIFI_BLOB_WRAPS "")

set(configured_target "${IDF_TARGET}")
if(NOT configured_target AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig")
    file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig" configured_target_line
         REGEX "^CONFIG_IDF_TARGET=\"[^\"]+\"$")
    string(REGEX REPLACE ".*=\"([^\"]+)\"" "\\1" configured_target "${configured_target_line}")
endif()

if(configured_target STREQUAL "esp32s31")
    set(use_stock_phy_blob y)
else()
    set(use_stock_phy_blob "$ENV{USE_STOCK_PHY_BLOB}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(esp_sdr)

function(filter_esp_phy_blob_links target)
    foreach(prop LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
        get_target_property(link_libs ${target} ${prop})
        if(link_libs)
            set(filtered_link_libs)
            foreach(link_lib ${link_libs})
                if(NOT link_lib MATCHES "esp_phy_lib(phy|btbb)|/esp_phy/lib/.*/lib(phy|btbb)\\.a$")
                    list(APPEND filtered_link_libs "${link_lib}")
                endif()
            endforeach()
            set_target_properties(${target} PROPERTIES ${prop} "${filtered_link_libs}")
        endif()
    endforeach()
endfunction()

if(NOT use_stock_phy_blob)
    set(empty_esp_phy_blob_archive "${CMAKE_BINARY_DIR}/empty-esp-phy-blob.a")
    execute_process(COMMAND "${CMAKE_AR}" qc "${empty_esp_phy_blob_archive}"
                    RESULT_VARIABLE empty_esp_phy_blob_result)
    if(NOT empty_esp_phy_blob_result EQUAL 0)
        message(FATAL_ERROR "Failed to create ${empty_esp_phy_blob_archive}")
    endif()

    foreach(blob_target esp_phy_libphy esp_phy_libbtbb)
        if(TARGET ${blob_target})
            set_target_properties(${blob_target} PROPERTIES
                IMPORTED_LOCATION "${empty_esp_phy_blob_archive}")
        endif()
    endforeach()

    idf_component_get_property(esp_phy_lib esp_phy COMPONENT_LIB)
    filter_esp_phy_blob_links(${esp_phy_lib})

    idf_build_get_property(build_component_targets __BUILD_COMPONENT_TARGETS)
    foreach(component_target ${build_component_targets})
        if(TARGET ${component_target})
            filter_esp_phy_blob_links(${component_target})
        endif()
    endforeach()

    idf_build_get_property(project_elf EXECUTABLE)
    filter_esp_phy_blob_links(${project_elf})

    idf_build_get_property(ldgen_fragment_files __LDGEN_FRAGMENT_FILES)
    if(ldgen_fragment_files)
        list(FILTER ldgen_fragment_files EXCLUDE REGEX "/esp_phy/linker\\.lf$")
        idf_build_set_property(__LDGEN_FRAGMENT_FILES "${ldgen_fragment_files}")
    endif()
endif()
