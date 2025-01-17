# Copyright (c) Microsoft Corporation. All rights reserved.
# SPDX-License-Identifier: MIT

set(ROOT_PATH
    ${CMAKE_CURRENT_LIST_DIR}/../../../../../..
)

set(VERIFIED_TELEMETRY
    ${ROOT_PATH}/libs/Verified-Telemetry
)

if (DEFINED CONFIG_AZURE_SAMPLE_USE_PLUG_AND_PLAY)
    file(GLOB_RECURSE COMPONENT_SOURCES
        ${ROOT_PATH}/demos/sample_azure_iot_pnp/*.c
    )
else()
    file(GLOB_RECURSE COMPONENT_SOURCES
        ${ROOT_PATH}/demos/sample_azure_iot/*.c
    )
endif()

idf_component_get_property(MBEDTLS_DIR mbedtls COMPONENT_DIR)

list(APPEND COMPONENT_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/backoff_algorithm.c
    ${CMAKE_CURRENT_LIST_DIR}/transport_tls_esp32.c
    ${CMAKE_CURRENT_LIST_DIR}/crypto_esp32.c

    ${VERIFIED_TELEMETRY}/src/core/fallcurve/vt_fc_object_database_fetch.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/vt_fc_object_database_sync.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/vt_fc_object_initialize.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/vt_fc_object_sensor_calibrate.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/vt_fc_object_sensor_status.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/internal/vt_fc_database_fetch.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/internal/vt_fc_database_reset.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/internal/vt_fc_database_store.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/internal/vt_fc_read.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/internal/vt_fc_signature_collection_settings.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/internal/vt_fc_signature_compute.c
    ${VERIFIED_TELEMETRY}/src/core/fallcurve/internal/vt_fc_signature_evaluate.c
    ${VERIFIED_TELEMETRY}/src/middleware/freertos_middleware/FreeRTOS_verified_telemetry.c
    ${VERIFIED_TELEMETRY}/src/middleware/freertos_middleware/FreeRTOS_vt_fallcurve_component.c

    ../../verified-telemetry/sample_pnp_device_component.c
    ../../verified-telemetry/sample_vt_device_driver.c    

)

set(COMPONENT_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/../../config
    ${CMAKE_CURRENT_LIST_DIR}/../../verified-telemetry
    ${CMAKE_CURRENT_LIST_DIR}
    ${MBEDTLS_DIR}/mbedtls/include
    ${ROOT_PATH}/demos/common/transport
    ${ROOT_PATH}/demos/common/utilities
    ${VERIFIED_TELEMETRY}/inc/core
    ${VERIFIED_TELEMETRY}/inc/platform
    ${VERIFIED_TELEMETRY}/inc/core/fallcurve
    ${VERIFIED_TELEMETRY}/inc/core/fallcurve/config
    ${VERIFIED_TELEMETRY}/inc/middleware/freertos_middleware
    ${VERIFIED_TELEMETRY}/inc/core/fallcurve/internal

)

idf_component_register(
    SRCS ${COMPONENT_SOURCES}
    INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
    REQUIRES mcp320x esp_adc_cal mbedtls tcp_transport coreMQTT azure-sdk-for-c azure-iot-middleware-freertos)

