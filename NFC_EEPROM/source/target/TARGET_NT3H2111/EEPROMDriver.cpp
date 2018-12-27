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
#include "nt3h_driver.h"

mbed::nfc::NFCEEPROMDriver& get_eeprom_driver(events::EventQueue&)
{
    static mbed::nfc::vendor::NXP::NT3HDriver eeprom_driver(D14,D15,D5);
    return eeprom_driver;
}
