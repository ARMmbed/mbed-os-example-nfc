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

#include "events/EventQueue.h"

#include "nfc/ndef/MessageBuilder.h"
#include "nfc/ndef/common/URI.h"

#include "m24sr_driver.h"
#include "NFCEEPROM.h"

using events::EventQueue;

using mbed::nfc::NFCEEPROM;
using mbed::nfc::NFCEEPROMDriver;
using mbed::Span;

using mbed::nfc::ndef::MessageBuilder;
using mbed::nfc::ndef::common::URI;

/* URL that will be written into the tag */
const uint8_t url_string[] = "mbed.com";

class EEPROMExample : mbed::nfc::NFCEEPROM::Delegate
{
public:
    EEPROMExample(events::EventQueue& queue, NFCEEPROMDriver& eeprom_driver) :
        _ndef_buffer(),
        _eeprom(&eeprom_driver, &queue, _ndef_buffer),
        _queue(queue)
    { }

    void run()
    {
        if (_eeprom.initialize() != NFC_OK) {
            printf("failed to initialise\r\n");
            _queue.break_dispatch();
        }

        _eeprom.set_delegate(this);

        _queue.call(&_eeprom, &NFCEEPROM::write_ndef_message);
    }

private:
    virtual void on_ndef_message_written(nfc_err_t result) {
        if (result == NFC_OK) {
            printf("message written successfully\r\n");
        } else {
            printf("failed to write (error: %d)\r\n", result);
        }

        _queue.call(&_eeprom, &NFCEEPROM::read_ndef_message);
    }

    virtual void on_ndef_message_read(nfc_err_t result) {
        if (result == NFC_OK) {
            printf("message read successfully\r\n");
        } else {
            printf("failed to read (error: %d)\r\n", result);
        }

        _queue.break_dispatch();
    }

    virtual void parse_ndef_message(const Span<const uint8_t> &buffer) {
        printf("Received an ndef message of size %d\r\n", buffer.size());
    }

    virtual size_t build_ndef_message(const Span<uint8_t> &buffer) {
        printf("Building an ndef message\r\n");

        /* create a message containing the URL */

        MessageBuilder builder(buffer);

        URI uri(URI::HTTPS_WWW, url_string);

        uri.append_as_record(builder, true);

        return builder.get_message().size();
    }

private:
    uint8_t _ndef_buffer[1024];
    NFCEEPROM _eeprom;
    EventQueue& _queue;
};

int main()
{
    mbed::nfc::vendor::ST::M24srDriver eeprom_driver;
    EventQueue queue;

    EEPROMExample example(queue, eeprom_driver);
    example.run();

    queue.dispatch_forever();

    return 0;
}

