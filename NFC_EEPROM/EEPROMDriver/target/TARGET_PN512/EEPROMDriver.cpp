/* mbed Microcontroller Library
 * Copyright (c) 2018-2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "EEPROMDriver.h"

#include "nfc/NFCController.h"
#include "NfcControllerToEEPROMAdapter.h"
#include "nfc/controllers/PN512Driver.h"
#include "nfc/controllers/PN512SPITransportDriver.h"
#include "target/NfcControllerToEEPROMAdapter.h"

using mbed::nfc::NFCController;
using mbed::nfc::PN512SPITransportDriver;
using mbed::nfc::PN512Driver;
using mbed::nfc::ControllerToEEPROMDriverAdapter;
using mbed::nfc::NFCEEPROMDriver;

NFCEEPROMDriver& get_eeprom_driver(events::EventQueue& queue)
{
    static uint8_t ndef_controller_buffer[1024] = { 0 };
    static uint8_t eeprom_buffer[1024] = { 0 };

    static PN512SPITransportDriver pn512_transport(ARDUINO_UNO_D11, ARDUINO_UNO_D12, ARDUINO_UNO_D13, ARDUINO_UNO_D10, ARDUINO_UNO_A1, ARDUINO_UNO_A0);
    static PN512Driver pn512_driver(&pn512_transport);

    static NFCController nfc_controller(
        &pn512_driver,
        &queue,
        ndef_controller_buffer
    );

    static ControllerToEEPROMDriverAdapter eeprom_driver(
        nfc_controller,
        eeprom_buffer
    );
    return eeprom_driver;
}

