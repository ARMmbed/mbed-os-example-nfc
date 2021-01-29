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

#include "NfcControllerToEEPROMAdapter.h"

#include <algorithm>

#include "nfc/NFCController.h"
#include "nfc/NFCEEPROMDriver.h"
#include "nfc/NFCRemoteInitiator.h"

namespace mbed {
namespace nfc {

ControllerToEEPROMDriverAdapter::ControllerToEEPROMDriverAdapter(
    NFCController &controller,
    const Span<uint8_t> &buffer
) :
    _nfc_controller(controller),
    _eeprom_buffer(buffer),
    _current_eeprom_size(buffer.size()),
    _session_opened(false)
{
}

ControllerToEEPROMDriverAdapter::~ControllerToEEPROMDriverAdapter()
{
    if (_nfc_remote_initiator.get()) {
        _nfc_remote_initiator->set_delegate(NULL);
    }
    _nfc_controller.set_delegate(NULL);
    _nfc_controller.cancel_discovery();
}

void ControllerToEEPROMDriverAdapter::reset()
{
    _current_eeprom_size = _eeprom_buffer.size();
    memset(_eeprom_buffer.data(), 0, _eeprom_buffer.size());

    if (_nfc_remote_initiator.get()) {
        _nfc_remote_initiator->set_delegate(NULL);
        _nfc_remote_initiator.reset();
    }

    _nfc_controller.initialize();
    _nfc_controller.set_delegate(this);

    nfc_rf_protocols_bitmask_t protocols = { 0 };
    protocols.target_iso_dep = 1;
    _nfc_controller.configure_rf_protocols(protocols);

    _nfc_controller.start_discovery();
}

size_t ControllerToEEPROMDriverAdapter::read_max_size()
{
    return _eeprom_buffer.size();
}

void ControllerToEEPROMDriverAdapter::start_session(bool force)
{
    _session_opened = true;
    delegate()->on_session_started(true);
}

void ControllerToEEPROMDriverAdapter::end_session()
{
    _session_opened = false;
    delegate()->on_session_ended(true);
}

void ControllerToEEPROMDriverAdapter::read_bytes(
    uint32_t address, uint8_t *bytes, size_t count
) {
    if (address >= _current_eeprom_size) {
        delegate()->on_bytes_read(0);
        return;
    }

    size_t size = std::min(count, (size_t) (_current_eeprom_size - address));
    memcpy(bytes, _eeprom_buffer.data() + address, size);
    delegate()->on_bytes_read(size);
}

void ControllerToEEPROMDriverAdapter::write_bytes(
    uint32_t address, const uint8_t *bytes, size_t count
) {
    if (address >= _current_eeprom_size) {
        delegate()->on_bytes_written(0);
        return;
    }

    size_t size = std::min(count, (size_t) (_current_eeprom_size - address));
    memcpy(_eeprom_buffer.data() + address, bytes, size);
    delegate()->on_bytes_written(size);
}

void ControllerToEEPROMDriverAdapter::read_size()
{
    delegate()->on_size_read(true, _current_eeprom_size);
}

void ControllerToEEPROMDriverAdapter::write_size(size_t count)
{
    if (count > (size_t) _eeprom_buffer.size()) {
        delegate()->on_size_written(false);
    } else {
        _current_eeprom_size = count;
        delegate()->on_size_written(true);
    }
}

void ControllerToEEPROMDriverAdapter::erase_bytes(uint32_t address, size_t count)
{
    if (address >= _current_eeprom_size) {
        delegate()->on_bytes_erased(0);
        return;
    }

    size_t size = std::min(count, (size_t) (_current_eeprom_size - address));
    memset(_eeprom_buffer.data() + address, 0, size);
    delegate()->on_bytes_erased(size);
}

/* ------------------------------------------------------------------------
 * Implementation of NFCRemoteInitiator::Delegate
 */
void ControllerToEEPROMDriverAdapter::on_connected() { }

void ControllerToEEPROMDriverAdapter::on_disconnected()
{
    // reset the state of the remote initiator
    _nfc_remote_initiator->set_delegate(NULL);
    _nfc_remote_initiator.reset();

    // restart peer discovery
    _nfc_controller.start_discovery();
}

void ControllerToEEPROMDriverAdapter::parse_ndef_message(const Span<const uint8_t> &buffer)
{
    if (_session_opened) {
        return;
    }

    if (buffer.size() > _eeprom_buffer.size()) {
        return;
    }

    _current_eeprom_size = buffer.size();
    memcpy(_eeprom_buffer.data(), buffer.data(), _current_eeprom_size);
}

size_t ControllerToEEPROMDriverAdapter::build_ndef_message(const Span<uint8_t> &buffer)
{
    if (_session_opened) {
        return 0;
    }

    if ((size_t) buffer.size() < _current_eeprom_size) {
        return 0;
    }

    memcpy(buffer.data(), _eeprom_buffer.data(), _current_eeprom_size);
    return _current_eeprom_size;
}

/* ------------------------------------------------------------------------
 * Implementation of NFCController::Delegate
 */
void ControllerToEEPROMDriverAdapter::on_discovery_terminated(
    nfc_discovery_terminated_reason_t reason
) {
    if(reason != nfc_discovery_terminated_completed) {
        _nfc_controller.start_discovery();
    }
}

void ControllerToEEPROMDriverAdapter::on_nfc_initiator_discovered(
    const SharedPtr<NFCRemoteInitiator> &nfc_initiator
) {
    // setup the local remote initiator
    _nfc_remote_initiator = nfc_initiator;
    _nfc_remote_initiator->set_delegate(this);
    _nfc_remote_initiator->connect();
}

}  // namespace nfc
}  // namespace mbed

