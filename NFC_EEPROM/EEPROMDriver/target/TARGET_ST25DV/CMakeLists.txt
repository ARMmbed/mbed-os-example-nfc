# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_include_directories(${APP_TARGET}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/eeprom_driver
	${CMAKE_CURRENT_SOURCE_DIR}/eeprom_driver/ST25DV
)

target_sources(${APP_TARGET}
    PRIVATE
        EEPROMDriver.cpp

        eeprom_driver/st25dv_driver.cpp
        eeprom_driver/ST25DV/st25dv.cpp
)
