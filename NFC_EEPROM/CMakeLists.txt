# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.18.2 FATAL_ERROR)

# TODO: @mbed-os-tools MBED_ROOT and MBED_CONFIG_PATH should probably come from mbedtools
set(MBED_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/.mbedbuild CACHE INTERNAL "")
set(APP_TARGET NFC_EEPROM)

add_subdirectory(${MBED_ROOT})

add_executable(${APP_TARGET})

mbed_configure_app_target(${APP_TARGET})

mbed_set_mbed_target_linker_script(${APP_TARGET})

project(${APP_TARGET})

include(${MBED_CONFIG_PATH}/mbed_config.cmake)

add_subdirectory(source/target)

target_include_directories(${APP_TARGET}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/source
)

target_sources(${APP_TARGET}
    PRIVATE
        source/main.cpp
)

list(APPEND MBED_LINK_LIBRARIES
    mbed-os
)

if("PN512" IN_LIST MBED_TARGET_LABELS)
    list(APPEND MBED_LINK_LIBRARIES
        mbed-os-connectivity-drivers-nfc-pn512
    )
endif()

target_link_libraries(${APP_TARGET}
    ${MBED_LINK_LIBRARIES}
)

mbed_generate_bin_hex(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
