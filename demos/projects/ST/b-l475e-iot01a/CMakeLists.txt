# Copyright (c) Microsoft Corporation. All rights reserved.
# SPDX-License-Identifier: MIT

stm32_fetch_cube(L4)

find_package(CMSIS COMPONENTS STM32L475VG REQUIRED)
find_package(HAL COMPONENTS STM32L475VG REQUIRED)
find_package(BSP COMPONENTS STM32L475E-IOT01 REQUIRED)
#find_package(BSP COMPONENTS STM32L475E_IOT01 REQUIRED)



# set parent scope path
set(BOARD_DEMO_CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config CACHE INTERNAL "Config path")
set(BOARD_DEMO_FREERTOS_PORT_PATH ${FreeRTOS_ARM_CM4F_PATH} CACHE INTERNAL "FreeRTOS Port used ")

include(${CMAKE_CURRENT_SOURCE_DIR}/gcc_flags.cmake)

include_directories(${BOARD_DEMO_CONFIG_PATH})

file(GLOB STCODE_SOURCES st_code/*.c)
set(PROJECT_SOURCES
    ${STCODE_SOURCES}
    port/sockets_wrapper_stm32l475.c
    main.c)

if(NOT (TARGET SAMPLE::DEVICE))
    add_library(SAMPLE::DEVICE INTERFACE IMPORTED)

    target_sources(SAMPLE::DEVICE INTERFACE
      ${CMAKE_CURRENT_SOURCE_DIR}/st_code/sample_pnp_device_component.c
      ${CMAKE_CURRENT_SOURCE_DIR}/st_code/sample_vt_device_driver.c
      )

endif()

stm32_add_linker_script(CMSIS::STM32::L4 INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/STM32L475VGTx_FLASH.ld")


# Add PnP Sample
add_executable(${PROJECT_NAME}-pnp ${PROJECT_SOURCES})
target_include_directories(${PROJECT_NAME}-pnp PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    st_code)
target_link_libraries(${PROJECT_NAME}-pnp PRIVATE
    FreeRTOS::Timers
    FreeRTOS::Heap::5
    FreeRTOS::ARM_CM4F
    FreeRTOSPlus::Utilities::backoff_algorithm
    FreeRTOSPlus::Utilities::logging
    FreeRTOSPlus::ThirdParty::mbedtls
    HAL::STM32::L4::RCC
    HAL::STM32::L4::RCCEx
    HAL::STM32::L4::QSPI
    HAL::STM32::L4::I2C
    HAL::STM32::L4::I2CEx
    HAL::STM32::L4::SPI
    HAL::STM32::L4::RTC
    HAL::STM32::L4::UART
    HAL::STM32::L4::DMA
    HAL::STM32::L4::PWR
    HAL::STM32::L4::PWREx
    HAL::STM32::L4::GPIO
    HAL::STM32::L4::CORTEX
    HAL::STM32::L4::RNG
    HAL::STM32::L4::ADC
    HAL::STM32::L4::ADCEx
    HAL::STM32::L4::TIM
    HAL::STM32::L4::TIMEx
    CMSIS::STM32::L475xx
    BSP::STM32::STM32L475E_IOT01
    BSP::STM32::L4::LSM6DSL
    BSP::STM32::L4::HTS221
    BSP::STM32::L4::LIS3MDL
    BSP::STM32::L4::LPS22HB
    STM32::NoSys
    az::iot_middleware::freertos
    SAMPLE::DEVICE
    SAMPLE::AZUREIOTPNP
    SAMPLE::TRANSPORT::MBEDTLS
    az::iot::vt)

add_custom_command(TARGET ${PROJECT_NAME}-pnp
    # Run after all other rules within the target have been executed
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}-pnp> ${PROJECT_NAME}-pnp.bin
    COMMENT "Generate Bin file"
    VERBATIM)
