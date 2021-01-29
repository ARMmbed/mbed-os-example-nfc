# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if("M24SR" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(TARGET_M24SR)
elseif("PN512" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(TARGET_PN512)
elseif("ST25DV" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(TARGET_ST25DV)
elseif("NT3H2111" IN_LIST MBED_TARGET_LABELS)
    add_subdirectory(TARGET_NT3H2111)
endif()

target_include_directories(${APP_TARGET}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(${APP_TARGET}
    PRIVATE
        NfcControllerToEEPROMAdapter.cpp
)
