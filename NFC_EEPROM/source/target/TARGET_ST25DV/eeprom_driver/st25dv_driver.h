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

#if defined MBED_CONF_X_NUCLEO_NFC04A1

#define ST25DV_I2C_SDA_PIN     D14
#define ST25DV_I2C_SCL_PIN     D15
#define ST25DV_LPD_PIN         D7
#define ST25DV_GPO_PIN         D12
#define ST25DV_LED1_PIN        D5
#define ST25DV_LED2_PIN        D4
#define ST25DV_LED3_PIN        D2

#else

#define ST25DV_I2C_SDA_PIN     NC
#define ST25DV_I2C_SCL_PIN     NC
#define ST25DV_LPD_PIN         NC
#define ST25DV_GPO_PIN         NC
#define ST25DV_LED1_PIN        NC
#define ST25DV_LED2_PIN        NC
#define ST25DV_LED3_PIN        NC

#endif

namespace mbed {
namespace nfc {
namespace vendor {
namespace ST {

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

#define NFCTAG_4K_SIZE            ((uint32_t) 0x200)
#define NFCTAG_16K_SIZE           ((uint32_t) 0x800)
#define NFCTAG_64K_SIZE           ((uint32_t) 0x2000)

#define MAX_NDEF_MEM                 0x200
#define ST25DV_MAX_SIZE              NFCTAG_4K_SIZE
#define ST25DV_NDEF_MAX_SIZE         MIN(ST25DV_MAX_SIZE,MAX_NDEF_MEM)
#define NFC_DEVICE_MAX_NDEFMEMORY    ST25DV_NDEF_MAX_SIZE

/* Error codes for Higher level */
#define NDEF_OK                     0
#define NDEF_ERROR                  1
#define NDEF_ERROR_MEMORY_TAG       2
#define NDEF_ERROR_MEMORY_INTERNAL  3
#define NDEF_ERROR_LOCKED           4
#define NDEF_ERROR_NOT_FORMATTED    5

#define NDEF_MAX_SIZE               NFC_DEVICE_MAX_NDEFMEMORY
#define NDEF_RECORD_MAX_SIZE        NFC_DEVICE_MAX_NDEFMEMORY

/** @brief Memory size value indicating that this is a 8-bytes Capability Container */
#define NFCT5_EXTENDED_CCFILE             0x00
/** @brief Capability container version 1.0 */
#define NFCT5_VERSION_V1_0                0x40
/** @brief Read access condition mask for the Capability Container byte1 */
#define NFCT5_READ_ACCESS                 0x0C
/** @brief Write access condition mask for the Capability Container byte1 */
#define NFCT5_WRITE_ACCESS                0x03

/** @brief Type5 Tag NDEF message TLV-Type. */
#define NFCT5_NDEF_MSG_TLV                ((uint8_t) 0x03)
/** @brief Type5 Tag Proprietary message TLV-Type. */
#define NFCT5_PROPRIETARY_TLV             ((uint8_t) 0xFD)
/** @brief Type5 Tag Terminator TLV-Type. */
#define NFCT5_TERMINATOR_TLV              ((uint8_t) 0xFE)
/** @brief TLV-Length indicating a 4-bytes TLV (Length coded on 2 bytes). */
#define NFCT5_3_BYTES_L_TLV               ((uint8_t) 0xFF)

#define MAX_NDEF_SIZE         NFC_DEVICE_MAX_NDEFMEMORY

typedef enum
{
  TT5_NO_NDEF = 0,  /**< No data detected in the tag. */
  TT5_INITIALIZED,  /**< Capability container detected. */
  TT5_READ_WRITE,   /**< Read-Write data detected. */
  TT5_READ          /**< Read-Only data message detected. */
} TT5_State;

/** @brief Type5 Tag Capability Container Magic numbers as defined by the NFC Forum. */
typedef enum {
  NFCT5_MAGICNUMBER_E1_CCFILE = 0xE1, /**<  Complete data area can be read by 1-byte block adrdess commands. */
  NFCT5_MAGICNUMBER_E2_CCFILE = 0xE2  /**<  Last part of the data area can be only read by 2-bytes block address commands.\n
                                            The first 256 blocks can be read by 1-byte block address commands. */
} TT5_MagicNumber_t;

/**
  * @brief  Type5 Tag Capability Container structure.
  */
typedef struct
{
  TT5_MagicNumber_t MagicNumber;  /**< CCfile[0]: Magic Number should be E1h or E2h (for extended API) */
  uint8_t Version;                /**< CCfile[1]: Capability container version (b7-b4) and access conditions (b3-b0) */
  uint8_t MemorySize;             /**< CCfile[2]: Memory size, expressed in 8 bytes blocks, set to 0 if tag size is greater than 16kbits. */
  uint8_t TT5Tag;                 /**< CCfile[3]: Additionnal information on the Type5 Tag:\n
                                                  b0: supports `read multiple block` commands\n
                                                  b1: RFU\n
                                                  b2: RFU\n
                                                  b3: supports `lock block` commands\n
                                                  b4: requires the `special frame` format
                                    */
  uint8_t rsved1;                 /**< RFU */
  uint8_t rsved2;                 /**< RFU */
  uint16_t ExtMemorySize;         /**< CCfile[6],CCfile[7]: Memory size, expressed in 8 bytes blocks, when tag size is greater than 16kbits. */
  TT5_State State;                /**< Indicates if a NDEF message is present. */
  uint32_t NDEF_offset;           /**< Indicates the address of a NDEF message in the tag. */
}sCCFileInfo;

/** @brief Type5 Tag Type-Length-Value structure as defined by the NFC Forum */
typedef struct
{
  uint8_t   Type;     /**< NFC Forum message Type */
  uint8_t   Length;   /**< Message length if lesser than 255 bytes */
  uint16_t  Length16; /**< Message length if greater than or equal to 255 bytes */
} TT5_TLV_t;

class ST25dvDriver : public NFCEEPROMDriver {

public:
    /** Create the driver, default pin names will be used appropriate for the board.
     *  @param i2c_data_pin I2C data pin name.
     *  @param i2c_clock_pin I2C clock pin name.
     *  @param gpo_pin I2C GPO pin name.
     *  @param rf_disable_pin pin name for breaking the RF connection.
     */
    ST25dvDriver(PinName i2c_data_pin = ST25DV_I2C_SDA_PIN,
                PinName i2c_clock_pin = ST25DV_I2C_SCL_PIN,
                PinName led1_pin = ST25DV_LED1_PIN,
                PinName led2_pin = ST25DV_LED2_PIN,
                PinName led3_pin = ST25DV_LED3_PIN,
                PinName lpd_pin = ST25DV_LPD_PIN,
                PinName gpo_pin = ST25DV_GPO_PIN);

    virtual ~ST25dvDriver() { }

    /** @see NFCEEPROMDriver::reset
     */
    virtual void reset() {
        printf("reset\r\n");
        begin();
    }

    /** @see NFCEEPROMDriver::get_max_size
     */
    virtual size_t read_max_size() {
        return MAX_NDEF_SIZE;
    }

    /** @see NFCEEPROMDriver::start_session
     */
    virtual void start_session(bool force = true) {
        printf("start_session\r\n");
        if(_is_device_inited) {
            _is_session_started = true;
            delegate()->on_session_started(true);
        } else {
            delegate()->on_session_started(false);
        }
    }

    /** @see NFCEEPROMDriver::end_session
     */
    virtual void end_session() {
        printf("end_session\r\n");
        if(_is_session_started) {
            _is_session_started = false;
            delegate()->on_session_ended(true);
        } else {
            delegate()->on_session_ended(false);
        }
    }

    /** @see NFCEEPROMDriver::read_bytes
     */
    virtual void read_bytes(uint32_t address, uint8_t* bytes, size_t count) {
        int ret;
        printf("read_bytes\r\n");

        if (address > _ndef_size) {
            delegate()->on_bytes_read(0);
            return;
        }

        ret = read_data(address, bytes, count);
        printf("read_bytes read_data ret =%d count=%d bytes=%s\r\n", ret, count, bytes);
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
        printf("write_bytes\r\n");
        if (address > _ndef_size) {
            delegate()->on_bytes_written(0);
            printf("write_bytes error (address > _ndef_size)\r\n");
            return;
        }

        ret = write_data(address, bytes, count);
        printf("write_bytes write_data ret =%d count=%d bytes=%s\r\n", ret, count, bytes);
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
        printf("write_size (count=%d)\r\n", count);
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
        printf("read_size\r\n");
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
        printf("read_size _ndef_size=%d\r\n", _ndef_size);
    }

    /** @see NFCEEPROMDriver::erase_bytes
     */
    virtual void erase_bytes(uint32_t address, size_t size) {
        printf("erase_bytes\r\n");
        write_data(address, NULL, size);
    }

private:
    int begin();
    int read_data(uint32_t address, uint8_t* bytes, size_t count);
    int write_data(uint32_t address, const uint8_t* bytes, size_t count);
    int get_size(void);
    int set_size(size_t count);

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

    void ledOn(DigitalOut led);
    void ledOff(DigitalOut led);

    I2C _i2c_channel;
    NFCTAG_DrvTypeDef *Nfctag_Drv;
    sCCFileInfo CCFileStruct;

    DigitalOut _led1_pin;
    DigitalOut _led2_pin;
    DigitalOut _led3_pin;
    DigitalOut _lpd_pin;
    DigitalIn _gpo_pin;

    uint16_t _ndef_size;

    bool _is_device_inited;
    bool _is_session_started;
};

} //ST
} //vendor
} //nfc
} //mbed

#endif // ST25DV_H

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
