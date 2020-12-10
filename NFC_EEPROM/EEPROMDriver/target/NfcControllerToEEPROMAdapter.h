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

#ifndef NFCCONTROLLER2EEPROMADAPTER_H_
#define NFCCONTROLLER2EEPROMADAPTER_H_

#include <algorithm>

#include "nfc/NFCController.h"
#include "nfc/NFCEEPROMDriver.h"
#include "nfc/NFCRemoteInitiator.h"

namespace mbed {
namespace nfc {

/**
 * Adapt an NFCController into an NFCEEPROMDriver.
 */
class ControllerToEEPROMDriverAdapter :
    public NFCEEPROMDriver,
    private NFCController::Delegate,
    private NFCRemoteInitiator::Delegate
{
public:
    ControllerToEEPROMDriverAdapter(
        NFCController &controller,
        const Span<uint8_t> &buffer
    );

    virtual ~ControllerToEEPROMDriverAdapter();

    /* ------------------------------------------------------------------------
     * Implementation of NFCEEPROMDriver
     */
    virtual void reset();

    virtual size_t read_max_size();

    virtual void start_session(bool force);

    virtual void end_session();

    virtual void read_bytes(uint32_t address, uint8_t *bytes, size_t count);

    virtual void write_bytes(uint32_t address, const uint8_t *bytes, size_t count);

    virtual void read_size();

    virtual void write_size(size_t count);

    virtual void erase_bytes(uint32_t address, size_t count);

private:
    /* ------------------------------------------------------------------------
     * Implementation of NFCRemoteInitiator::Delegate
     */
    virtual void on_connected();

    virtual void on_disconnected();

    virtual void parse_ndef_message(const Span<const uint8_t> &buffer);

    virtual size_t build_ndef_message(const Span<uint8_t> &buffer);

    /* ------------------------------------------------------------------------
     * Implementation of NFCController::Delegate
     */
    virtual void on_discovery_terminated(
        nfc_discovery_terminated_reason_t reason
    );

    virtual void on_nfc_initiator_discovered(
        const SharedPtr<NFCRemoteInitiator> &nfc_initiator
    );

    NFCController& _nfc_controller;
    Span<uint8_t> _eeprom_buffer;
    size_t _current_eeprom_size;
    bool _session_opened;
    SharedPtr<NFCRemoteInitiator> _nfc_remote_initiator;
};

}  // namespace nfc
}  // namespace mbed


#endif /* NFCCONTROLLER2EEPROMADAPTER_H_ */
