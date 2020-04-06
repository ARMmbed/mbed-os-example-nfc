/**
 ******************************************************************************
 * @file    m24sr_driver.cpp
 * @author  ST Central Labs
 * @brief   This file provides a set of functions to interface with the M24SR
 *          device.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

#include <st25dv_driver.h>
#include "Callback.h"

namespace mbed {
namespace nfc {
namespace vendor {
namespace ST {

ST25dvDriver::ST25dvDriver(PinName i2c_data_pin,
                            PinName i2c_clock_pin,
                            PinName led1_pin,
                            PinName led2_pin,
                            PinName led3_pin,
                            PinName lpd_pin,
                            PinName gpo_pin)
    : _i2c_channel(i2c_data_pin, i2c_clock_pin),
      _led1_pin(led1_pin),
      _led2_pin(led2_pin),
      _led3_pin(led3_pin),
      _lpd_pin(lpd_pin),
      _gpo_pin(gpo_pin),
      _ndef_size(MAX_NDEF_SIZE),
      _is_device_inited(false),
      _is_session_started(false) {
    /* driver requires valid pin names */
    MBED_ASSERT(i2c_data_pin != NC);
    MBED_ASSERT(i2c_clock_pin != NC);
    MBED_ASSERT(led1_pin != NC);
    MBED_ASSERT(led2_pin != NC);
    MBED_ASSERT(led3_pin != NC);
    MBED_ASSERT(lpd_pin != NC);
    MBED_ASSERT(gpo_pin != NC);
}

int ST25dvDriver::begin(void) {
    int ret = 0;
    
    // Light some leds
    ledOn(_led1_pin);
    wait_us( 300000 );
    ledOn(_led2_pin);

    /* NFCTAG Init */
    ret = NFCTAG_Init();
    if(ret != NFCTAG_OK)
        return ret;

    /* Reset MBEN Dynamic */
    NFCTAG_GetExtended_Drv()->ResetMBEN_Dyn( &_i2c_channel );

    if( NfcType5_NDEFDetection() != NDEF_OK ) {
        printf("NDEF not detected\r\n");

        CCFileStruct.MagicNumber = NFCT5_MAGICNUMBER_E1_CCFILE;
        CCFileStruct.Version = NFCT5_VERSION_V1_0;
        CCFileStruct.MemorySize = ( ST25DV_MAX_SIZE / 8 ) & 0xFF;
        CCFileStruct.TT5Tag = 0x05;

        /* Init of the Type Tag 5 component */
        ret = NfcType5_TT5Init();
        if (ret != NDEF_OK)
            return ret;
        
    } else {
        printf("NDEF detected\r\n");
    }

    ledOff( _led1_pin );
    wait_us( 300000 );
    ledOff( _led2_pin );
    wait_us( 300000 );
    ledOff( _led3_pin );
    wait_us( 300000 );
    return NFCTAG_OK;
}

int ST25dvDriver::read_data(uint32_t address, uint8_t* bytes, size_t count) {
    
    (void) address;
    (void) count;
    int ret = NfcType5_ReadNDEF((uint8_t*)bytes);

    if (ret != NDEF_OK) {   
        return ret;
    }
    return NFCTAG_OK;
}

int ST25dvDriver::write_data(uint32_t address, const uint8_t* bytes, size_t count) {

    (void) address;
    int ret = NfcType5_WriteNDEF(count, (uint8_t*)bytes);

    if (ret != NDEF_OK) {   
        return ret;
    }
    return NFCTAG_OK;
}

int ST25dvDriver::get_size(void) {
    int ret = NfcType5_GetLength(&_ndef_size);

    if (ret != NDEF_OK) {   
        return ret;
    }
    return NFCTAG_OK;
}

int ST25dvDriver::set_size(size_t count) {
    int ret = NfcType5_SetLength(count);

    if (ret != NDEF_OK) {   
        return ret;
    }
    return NFCTAG_OK;
}

NFCTAG_StatusTypeDef ST25dvDriver::NFCTAG_Init(void) {

  uint8_t nfctag_id;
  
  if( !_is_device_inited )
  {
    /* ST25DV Init */
    if( St25Dv_i2c_Drv.Init(&_i2c_channel, &_lpd_pin) != NFCTAG_OK )
    {
      return NFCTAG_ERROR;
    }


    St25Dv_i2c_Drv.ReadID(&nfctag_id, &_i2c_channel);
    
    /* Check if it is the wanted chip */
    if( (nfctag_id == I_AM_ST25DV04) || (nfctag_id == I_AM_ST25DV64) )
    {
      _is_device_inited = true;
      Nfctag_Drv = &St25Dv_i2c_Drv;
      Nfctag_Drv->pData = &St25Dv_i2c_ExtDrv;
    }
    else
    {
      Nfctag_Drv = NULL;
      _is_device_inited = false;
      return NFCTAG_ERROR;
    }
  }
  
  return NFCTAG_OK;
}


NFCTAG_StatusTypeDef ST25dvDriver::NFCTAG_ReadData(uint8_t * const pData, const uint16_t TarAddr, const uint16_t Size) {
    
  if ( Nfctag_Drv->ReadData == NULL )
  {
    return NFCTAG_ERROR;
  }
  
  return Nfctag_Drv->ReadData( pData, TarAddr, Size, &_i2c_channel );
}


NFCTAG_StatusTypeDef ST25dvDriver::NFCTAG_WriteData(const uint8_t * const pData, const uint16_t TarAddr, const uint16_t Size) {
    
  if ( Nfctag_Drv->WriteData == NULL )
  {
    return NFCTAG_ERROR;
  }
  
  return Nfctag_Drv->WriteData( pData, TarAddr, Size, &_i2c_channel );
}

uint32_t ST25dvDriver::NFCTAG_GetByteSize(void)
{
  ST25DV_MEM_SIZE mem_size;
  ((NFCTAG_ExtDrvTypeDef *)Nfctag_Drv->pData)->ReadMemSize( &mem_size, &_i2c_channel );
  return (mem_size.BlockSize+1) * (mem_size.Mem_Size+1);
}

/**
  * @brief  Give extended features for component
  * @param  None
  * @retval address of the Extended Component Structure
  */
NFCTAG_ExtDrvTypeDef* ST25dvDriver::NFCTAG_GetExtended_Drv(void)
{
  return (NFCTAG_ExtDrvTypeDef *)Nfctag_Drv->pData;
}

/**
  * @brief  This function light on selected Led
  * @param  led : Led to be lit on
  * @retval None
  */
void ST25dvDriver::ledOn(DigitalOut led) {
    printf("ledOn\r\n");
    led.write(1);
}

/**
  * @brief  This function light off selected Led
  * @param  led : Led to be lit off
  * @retval None
  */

void ST25dvDriver::ledOff(DigitalOut led) {
    led.write(0);
}

/**
  * @brief    This function detects a NDEF message in a Type 5 Tag.
  * @details  It first detects the Capability Container and then look for the NDEF TLV.
  *           The `CCfileStruct` global variable is updated accordingly with what is detected.
  * @retval NDEF_OK                 NDEF message Tag Type 5 detected.
  * @retval NDEF_ERROR_NOT_FORMATED Device is not a NFC Tag Type 5 Tag.
  */
uint16_t ST25dvDriver::NfcType5_NDEFDetection(void)
{
 uint8_t acc_buffer[8];
  TT5_TLV_t tlv_detect;
  uint16_t status;
  uint32_t memory_size;
  
  CCFileStruct.State = TT5_NO_NDEF;

  /* Read CCFile */
  status = NfcType5_ReadCCFile( acc_buffer );

  if( status != NDEF_OK )
  {
    return status;
  }
  
  /* Check Byte 0 is equal to magic number */
  if( ( acc_buffer[0] != NFCT5_MAGICNUMBER_E1_CCFILE ) && ( acc_buffer[0] != NFCT5_MAGICNUMBER_E2_CCFILE ) )
  {
   return NDEF_ERROR_NOT_FORMATTED;
  }
  /* Check Version number */
  else if( ( (acc_buffer[1]&0xFC) != 0x40 ) )
  {
    return NDEF_ERROR_NOT_FORMATTED;
  }
  
  /* Check if CCFile is on 4 Bytes or 8 Bytes */
  if( acc_buffer[2] == 0x00 )
  {
    /* Update CCFIle structure */
    CCFileStruct.MemorySize = 0x0;
    CCFileStruct.ExtMemorySize = (uint16_t)acc_buffer[6];
    CCFileStruct.ExtMemorySize = ( CCFileStruct.ExtMemorySize << 8 ) |  acc_buffer[7];
    memory_size = CCFileStruct.ExtMemorySize;
    CCFileStruct.NDEF_offset = 8;
  }
  else
  {
    /* Update CCFIle structure */
    CCFileStruct.MemorySize = acc_buffer[2];
    CCFileStruct.ExtMemorySize = 0x0;
    memory_size = CCFileStruct.MemorySize;
    CCFileStruct.NDEF_offset = 4;
  }
  
  /* Update CCFIle structure */
  CCFileStruct.MagicNumber = (TT5_MagicNumber_t)acc_buffer[0];
  CCFileStruct.Version = acc_buffer[1];
  CCFileStruct.TT5Tag = acc_buffer[3];
  
  /* Search for position of NDEF TLV in memory and tag status */
  while( ( NFCTAG_ReadData( (uint8_t *)&tlv_detect, CCFileStruct.NDEF_offset, sizeof(TT5_TLV_t) ) == NFCTAG_OK ) && ( CCFileStruct.NDEF_offset < memory_size ) )
  {
    /* Detect first NDEF Message in memory */
    if( tlv_detect.Type == NFCT5_NDEF_MSG_TLV )
    {
      if( tlv_detect.Length == 0x00 )
      {
        CCFileStruct.State = TT5_INITIALIZED;
      }
      else
      {
        if( CCFileStruct.Version & 0x3 )
        {
          CCFileStruct.State = TT5_READ;
        }
        else
        {
          CCFileStruct.State = TT5_READ_WRITE;
        }
      }
      return NDEF_OK;
    }
    /* If Proprietary NDEF jump to end of proprietary message */
    else if( tlv_detect.Type == NFCT5_PROPRIETARY_TLV )
    {
      if( tlv_detect.Length == NFCT5_3_BYTES_L_TLV )
      {
        CCFileStruct.NDEF_offset = CCFileStruct.NDEF_offset + tlv_detect.Length16;
        continue;
      }
      else
      {
        CCFileStruct.NDEF_offset = CCFileStruct.NDEF_offset + tlv_detect.Length;
        continue;
      }
    }
    /* if Terminator no NDEF detected */
    else if( tlv_detect.Type == NFCT5_TERMINATOR_TLV )
    {
     return NDEF_ERROR_NOT_FORMATTED;
    }
      
    CCFileStruct.NDEF_offset++;
  }
  
  return NDEF_ERROR_NOT_FORMATTED;
}

/**
  * @brief  This function initializes the Capability Container and an empty NDEF message in a NFC Tag.
  * @details The Capability Container content is defined by the global variable `CCFileStruct`.
  * @retval NDEF_ERROR The Tag has not been initialized.
  * @retval NDEF_OK    The Tag has been successfully initialized.
  */
uint16_t ST25dvDriver::NfcType5_TT5Init(void)
{
  NFCTAG_StatusTypeDef ret_value = NFCTAG_OK;
  uint16_t status;
  uint8_t accbuffer[8];
  uint8_t cdata;

  /* Prepare buffer to update CCFile */
  accbuffer[0] = CCFileStruct.MagicNumber;
  accbuffer[1] = CCFileStruct.Version;
  accbuffer[2] = CCFileStruct.MemorySize;
  accbuffer[3] = CCFileStruct.TT5Tag;
  CCFileStruct.NDEF_offset = 0x04;
  
  /* If extended memory prepare the length bytes */
  if( CCFileStruct.MemorySize == NFCT5_EXTENDED_CCFILE )
  {
    accbuffer[6] = (uint8_t)(CCFileStruct.ExtMemorySize >> 8);
    accbuffer[7] = (uint8_t)(CCFileStruct.ExtMemorySize & 0xFF);
    CCFileStruct.NDEF_offset = 0x08;
  }
  
  /* Update CCFile */
  status = NfcType5_WriteCCFile( accbuffer );


  if( status != NDEF_OK )
  {
    return status;
  }

  /* Update NDEF TLV for INITIALIZED state */
  /* Update T */
  cdata = NFCT5_NDEF_MSG_TLV;
  ret_value = NFCTAG_WriteData( &cdata, CCFileStruct.NDEF_offset, 1 );
  if( ret_value != NFCTAG_OK )
  {
    return NDEF_ERROR;
  }

  /* Update L */
  cdata = 0x00;
  ret_value = NFCTAG_WriteData( &cdata, (CCFileStruct.NDEF_offset + 1), 1 );
  if( ret_value != NFCTAG_OK )
  {
    return NDEF_ERROR;
  }
  
  return NDEF_OK;
}

/**
  * @brief  This functions writes the Capability Container in the NFC Tag.
  * @param  pCCBuffer Pointer on the buffer containnig the Capability Container.
  * @retval NDEF_ERROR Error when writing the Tag.
  * @retval NDEF_OK    The CC has been successfully written.
  */
uint16_t ST25dvDriver::NfcType5_WriteCCFile( const uint8_t * const pCCBuffer )
{
  NFCTAG_StatusTypeDef ret_value;
  
  /* Write first block of CCFile */
  ret_value = NFCTAG_WriteData( pCCBuffer, 0x00, 0x4 );
 
  /* If extended memory writes the next 4 bytes */
  if( (pCCBuffer[2] == 0x00) && (ret_value == NFCTAG_OK) )
  {
    ret_value = NFCTAG_WriteData( pCCBuffer + 4, 0x04, 4 );
  }

  if( ret_value != NFCTAG_OK )
  {
    return NDEF_ERROR;
  }
  
    return NDEF_OK;
}

/**
  * @brief  This functions reads the Capability Container from the NFC Tag.
  * @param  pCCBuffer Pointer on the buffer used to store the CC.
  * @retval NDEF_ERROR Error when reading the Tag.
  * @retval NDEF_OK    The CC has been successfully read.
  */
uint16_t ST25dvDriver::NfcType5_ReadCCFile( uint8_t * const pCCBuffer )
{
  NFCTAG_StatusTypeDef ret_value;
  
  /* Read 4 bytes of CC File */
  ret_value = NFCTAG_ReadData( pCCBuffer, 0x00, 4 );

  /* If extended memory reads the next 4 bytes */
  if( (pCCBuffer[2] == 0x00) && (ret_value == NFCTAG_OK) )
  {
    ret_value = NFCTAG_ReadData( pCCBuffer + 4, 0x04, 4 );
  }
  
  if( ret_value != NFCTAG_OK )
  {
    return NDEF_ERROR;
  }
  
    return NDEF_OK;
}

/**
  * @brief  This function reads the data stored in the NDEF message.
  * @param  pData Pointer on the buffer used to store the read data.
  * @retval NDEF_ERROR_MEMORY_INTERNAL  The buffer is too small for the NDEF message.
  * @retval NDEF_ERROR_NOT_FORMATED     No Capability Container detected.
  * @retval NDEF_ERROR                  Error when reading the NDEF message.
  * @retval NDEF_OK                     NDEF message successfully read.
  */
uint16_t ST25dvDriver::NfcType5_ReadNDEF( uint8_t* pData )
{
  uint16_t status = NDEF_ERROR;
  TT5_TLV_t tlv;
  uint8_t tlv_size = 0;
  uint16_t DataLength;

  /* Detect NDEF message in memory */
  status = NfcType5_NDEFDetection();
  if( status != NDEF_OK )
  {
    return status;
  }
  
  /* Read TL of Type 5 */
  status = NFCTAG_ReadData( (uint8_t*)&tlv, CCFileStruct.NDEF_offset, sizeof(TT5_TLV_t) );
  if( status != NDEF_OK )
  {
    return status;
  }
  
  /* Check if L is on 3 or 1 byte and update length in buffer */
  if( tlv.Length == NFCT5_3_BYTES_L_TLV )
  {
    tlv_size = 4;
    DataLength = ((tlv.Length16 >> 8)&0xff) | ((tlv.Length16&0xff)<<8);
  }
  else
  {
    tlv_size = 2;
    DataLength = tlv.Length;
  }
      /* If too many data to write return error */
  if( DataLength > NDEF_MAX_SIZE )
  {
    return NDEF_ERROR_MEMORY_INTERNAL;
  }
  
  /* Check CC file is in the correct mode to proceed */
  if( CCFileStruct.State ==  TT5_INITIALIZED )
  {
    return NDEF_ERROR;
  }

  if( DataLength > 0 )
  {
    /* Read NDEF */
    if( NFCTAG_ReadData( (pData), CCFileStruct.NDEF_offset + tlv_size, DataLength ) != NFCTAG_OK )
    {
      return NDEF_ERROR;
    }
  }
  
  return NDEF_OK;
}

/**
  * @brief  This function writes a NDEF message in the NFC Tag.
  * @param  Length Number of bytes to write.
  * @param  pData  Pointer on the buffer to copy.
  * @retval NDEF_ERROR_MEMORY_INTERNAL Memory size is too small for the data.
  * @retval NDEF_ERROR_NOT_FORMATED    No Capability Container detected.
  * @retval NDEF_ERROR                 Error when writing the Tag.
  * @retval NDEF_OK                    The data has been successfully written.
  */
uint16_t ST25dvDriver::NfcType5_WriteNDEF( uint16_t Length, uint8_t *pData )
{
  uint8_t tlv_size;
  uint32_t offset;
  uint8_t NfcT5_Terminator = NFCT5_TERMINATOR_TLV;
  
  if(Length >= 0xFF)
  {
    tlv_size = 4;
  } else {
    tlv_size = 2;
  }

  offset = CCFileStruct.NDEF_offset + tlv_size;

  /* Continue write TLV data  to EEPROM */
  if(NFCTAG_WriteData( pData , offset, Length ) != NFCTAG_OK )
      return NDEF_ERROR;

  offset +=Length;
  
  /* Write Terminator TLV */
  if(NFCTAG_WriteData( &NfcT5_Terminator, offset, sizeof(NfcT5_Terminator) ) != NFCTAG_OK)
    return NDEF_ERROR;
  
  return NDEF_OK;
}

uint16_t ST25dvDriver::NfcType5_SetLength(uint16_t Length)
{
  TT5_TLV_t tlv;
  uint8_t tlv_size;
  uint32_t offset;
  uint8_t NfcT5_Terminator = NFCT5_TERMINATOR_TLV;
  
  uint32_t max_length = NFCTAG_GetByteSize()        /* Memory size */
                        - ((Length >= 0xFF) ? 4 : 2)    /* - TLV length */
                        - sizeof(NfcT5_Terminator)      /* - Terminator TLV */
                        - CCFileStruct.NDEF_offset;     /* - CCfile length */

  /* If too many data to write return error */
  if( Length > max_length )
  {
    return NDEF_ERROR_MEMORY_TAG;
  }

  /* Detect NDEF message in memory */
  if( NfcType5_NDEFDetection() != NDEF_OK )
  {
    return NDEF_ERROR;
  }
  
  /* Prepare TLV */
  tlv.Type = NFCT5_NDEF_MSG_TLV;
  if(Length >= 0xFF)
  {
    tlv.Length = NFCT5_3_BYTES_L_TLV;
    tlv.Length16 = ((Length&0xff)<<8) | ((Length>>8)&0xff) ;
    tlv_size = 4;
    
  } else {
    tlv.Length = Length;
    tlv_size = 2;
  }

  offset = CCFileStruct.NDEF_offset;
  /* Start write TLV to EEPROM */
  if(NFCTAG_WriteData( (uint8_t*)&tlv, offset, tlv_size )!= NFCTAG_OK)
    return NDEF_ERROR;

  return NDEF_OK;
}

/**
  * @brief  This function reads and return the size of the NDEF message in the NFC tag.
  * @param  Length Pointer on the NDEF size to be returned.
  * @retval NDEF_ERROR_NOT_FORMATED Device is not a NFC Tag Type 5 Tag.
  * @retval NDEF_ERROR              The NDEF message size has not been read.
  * @retval NDEF_OK                 The NDEF message size has been retrieved.
  */
uint16_t ST25dvDriver::NfcType5_GetLength(uint16_t* Length)
{
  
  uint16_t status = NDEF_ERROR;
  TT5_TLV_t tlv;
  
  /* Detect NDEF message in memory */
  status = NfcType5_NDEFDetection();
  if( status != NDEF_OK )
  {
    return status;
  }
  
  /* Read TL of Type 5 */
  status = NFCTAG_ReadData( (uint8_t*)&tlv, CCFileStruct.NDEF_offset, sizeof(TT5_TLV_t) );
  if( status != NFCTAG_OK )
  {
    return NDEF_ERROR;
  }
  
  if(tlv.Length != NFCT5_3_BYTES_L_TLV)
  {
    *Length = tlv.Length;
  } else {
    *Length = ((tlv.Length16 >> 8)&0xff) | ((tlv.Length16 & 0xff) << 8);
  }
  
  return NDEF_OK;  
}

} //ST
} //vendor
} //nfc
} //mbed

mbed::nfc::NFCEEPROMDriver* greentea_nfc_EEPROM_driver_get_instance()
{
    static mbed::nfc::vendor::ST::ST25dvDriver instance;
    return &instance;
}

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/