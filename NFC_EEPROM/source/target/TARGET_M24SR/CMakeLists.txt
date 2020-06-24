# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_include_directories(mbed-os
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/eeprom_driver
)

target_sources(app
    PRIVATE
        EEPROMDriver.cpp

        eeprom_driver/m24sr_driver.cpp
)
