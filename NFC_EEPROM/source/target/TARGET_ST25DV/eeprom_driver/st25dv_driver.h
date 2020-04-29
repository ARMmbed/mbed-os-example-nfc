/**
 ******************************************************************************
 * @file    st25dv_driver.h
 * @author  ST Central Labs
 * @brief   This file provides a set of functions to interface with the ST25DV
 *          device.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifndef ST25DV_H
#define ST25DV_H

#include <stdint.h>
#include <mbed.h>
#include "I2C.h"
#include "NFCEEPROMDriver.h"
#include "EventQueue.h"
#include "st25dv.h"

#include "mbed_trace.h"
#define TRACE_GROUP "ST25DV"

#if defined MBED_CONF_X_NUCLEO_NFC04A1

#define ST25DV_I2C_SDA_PIN     D14
#define ST25DV_I2C_SCL_PIN     D15
#define ST25DV_LPD_PIN         D7
#define ST25DV_GPO_PIN         D12

#else

#define ST25DV_I2C_SDA_PIN     NC
#define ST25DV_I2C_SCL_PIN     NC
#define ST25DV_LPD_PIN         NC
#define ST25DV_GPO_PIN         NC

#endif

namespace mbed {
namespace nfc {
namespace vendor {
namespace ST {

/* Error codes for Higher level */
#define NDEF_OK                     0
#define NDEF_ERROR                  1
#define NDEF_ERROR_MEMORY_TAG       2
#define NDEF_ERROR_MEMORY_INTERNAL  3
#define NDEF_ERROR_LOCKED           4
#define NDEF_ERROR_NOT_FORMATTED    5

typedef struct sCCFileInfo sCCFileInfo_t;

class ST25dvDriver : public NFCEEPROMDriver {

public:
    /** Create the driver, default pin names will be used appropriate for the board.
     *  @param i2c_data_pin I2C data pin name.
     *  @param i2c_clock_pin I2C clock pin name.
     *  @param gpo_pin I2C GPO pin name.
     */
    ST25dvDriver(PinName i2c_data_pin = ST25DV_I2C_SDA_PIN,
                PinName i2c_clock_pin = ST25DV_I2C_SCL_PIN,
                PinName lpd_pin = ST25DV_LPD_PIN,
                PinName gpo_pin = ST25DV_GPO_PIN);

    virtual ~ST25dvDriver() { }

    /** @see NFCEEPROMDriver::reset
     */
    virtual void reset() {
        tr_debug("reset\r\n");
        begin();
    }

    /** @see NFCEEPROMDriver::get_max_size
     */
    virtual size_t read_max_size() {
        return _max_mem_size;
    }

    /** @see NFCEEPROMDriver::start_session
     */
    virtual void start_session(bool force = true) {
        int ret;
        tr_debug("start_session\r\n");
        if(_is_session_started) {
          delegate()->on_session_started(true);
        }

        ret = open_session(force);
        if(ret != 0) {
          delegate()->on_session_started(false);
        } else {
          _is_session_started = true;
          delegate()->on_session_started(true);
        }
    }

    /** @see NFCEEPROMDriver::end_session
     */
    virtual void end_session() {
      int ret;
      printf("end_session\r\n");

      ret = close_session();
      printf("ret=%d\r\n", ret);
      if(ret != 0) {
        delegate()->on_session_ended(false);
      } else {
        _is_session_started = false;
        delegate()->on_session_ended(true);
      }
    }

    /** @see NFCEEPROMDriver::read_bytes
     */
    virtual void read_bytes(uint32_t address, uint8_t* bytes, size_t count) {
        int ret;
        tr_debug("read_bytes\r\n");

        if (address > _ndef_size) {
            delegate()->on_bytes_read(0);
            return;
        }

        ret = read_data(address, bytes, count);
        tr_debug("read_bytes read_data ret =%d count=%d bytes=%s\r\n", ret, count, bytes);
        if(ret != 0) {
            delegate()->on_bytes_read(0);
        } else {
            delegate()->on_bytes_read(count);
        }
    }

    /** @see NFCEEPROMDriver::write_bytes
     */
    virtual void write_bytes(uint32_t address, const uint8_t* bytes, size_t count) {
        int ret;
        tr_debug("write_bytes\r\n");
        if (address > _ndef_size) {
            delegate()->on_bytes_written(0);
            tr_error("write_bytes error (address > _ndef_size)\r\n");
            return;
        }

        ret = write_data(address, bytes, count);
        tr_debug("write_bytes write_data ret =%d count=%d bytes=%s\r\n", ret, count, bytes);
        if(ret != 0) {
            delegate()->on_bytes_written(0);
        } else {
            delegate()->on_bytes_written(count);
        }
    }

    /** @see NFCEEPROMDriver::write_size
     */
    virtual void write_size(size_t count) {
        int ret;
        tr_debug("write_size (count=%d)\r\n", count);
        if (!_is_session_started) {
            delegate()->on_size_written(false);
            return;
        }
        _ndef_size = count;

        ret = set_size(count);
        if(ret != 0) {
            delegate()->on_size_written(false);
        } else {
            delegate()->on_size_written(true);
        }
    }

    /** @see NFCEEPROMDriver::read_size
     */
    virtual void read_size() {
        int ret;
        tr_debug("read_size\r\n");
        if (!_is_session_started) {
            delegate()->on_size_read(false, 0);
            return;
        }

        ret = get_size();
        if(ret != 0) {
            delegate()->on_size_read(false, 0);
        } else {
            delegate()->on_size_read(true, _ndef_size);
        }
        tr_debug("read_size _ndef_size=%d\r\n", _ndef_size);
    }

    /** @see NFCEEPROMDriver::erase_bytes
     */
    virtual void erase_bytes(uint32_t address, size_t size) {
        tr_debug("erase_bytes\r\n");
        write_data(address, NULL, size);
    }

private:
    int begin(void);
    int read_data(uint32_t address, uint8_t* bytes, size_t count);
    int write_data(uint32_t address, const uint8_t* bytes, size_t count);
    int get_size(void);
    int set_size(size_t count);
    int open_session(bool force);
    int close_session(void);

    NFCTAG_StatusTypeDef NFCTAG_Init(void);
    NFCTAG_StatusTypeDef NFCTAG_ReadData(uint8_t * const pData, const uint16_t TarAddr, const uint16_t Size);
    NFCTAG_StatusTypeDef NFCTAG_WriteData(const uint8_t * const pData, const uint16_t TarAddr, const uint16_t Size);
    uint32_t NFCTAG_GetByteSize(void);
    NFCTAG_ExtDrvTypeDef *NFCTAG_GetExtended_Drv(void);
    uint16_t NfcType5_NDEFDetection(void);
    uint16_t NfcType5_TT5Init(void);
    uint16_t NfcType5_WriteCCFile( const uint8_t * const pCCBuffer );
    uint16_t NfcType5_ReadCCFile( uint8_t * const pCCBuffer );
    uint16_t NfcType5_ReadNDEF(uint8_t* pData);
    uint16_t NfcType5_WriteNDEF(uint16_t Length, uint8_t* pData);
    uint16_t NfcType5_GetLength(uint16_t* Length);
    uint16_t NfcType5_SetLength(uint16_t Length);

    I2C _i2c_channel;
    NFCTAG_DrvTypeDef *Nfctag_Drv;
    sCCFileInfo_t *CCFileStruct;

    DigitalOut _lpd_pin;
    DigitalIn _gpo_pin;

    uint32_t _max_mem_size;
    uint32_t _ndef_size;

    bool _is_device_inited;
    bool _is_session_started;
};

} //ST
} //vendor
} //nfc
} //mbed

#endif // ST25DV_H

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
