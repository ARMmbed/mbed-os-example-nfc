/**
  ******************************************************************************
  * @file    x_nucleo_nfc04a1.c
  * @author  MMY Application Team
  * @version $Revision: 3351 $
  * @date    $Date: 2017-01-25 17:28:08 +0100 (Wed, 25 Jan 2017) $
  * @brief   This file provides nfc04a1 specific functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty  
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "st25dv.h"
#include "time.h"

/** @defgroup X_NUCLEO_NFC04A1
 * @{
 */
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
/* Global variables ----------------------------------------------------------*/
/** @defgroup X_NUCLEO_NFC04A1_Global_Variables
 * @{
 */
//uint8_t NFC04A1_Led[3] = {  1 , 2 , 3 };

/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/

NFCTAG_StatusTypeDef ST25DV_IO_Init( I2C* mi2cChannel, DigitalOut *mLPD );
NFCTAG_StatusTypeDef ST25DV_IO_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size, I2C* mi2cChannel );
NFCTAG_StatusTypeDef ST25DV_IO_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size, I2C* mi2cChannel );
NFCTAG_StatusTypeDef ST25DV_IO_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size, I2C* mi2cChannel );
NFCTAG_StatusTypeDef ST25DV_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials, I2C* mi2cChannel);
NFCTAG_StatusTypeDef NFCTAG_ConvertStatus(uint8_t ret);


/* Functions Definition ------------------------------------------------------*/
/** @defgroup X_NUCLEO_NFC04A1_Public_Functions
 * @{
 */



/**
  * @brief  Toggles the selected LED
  * @param  led : Specifies the Led to be toggled
  * @retval None
  */
void NFC04A1_LED_Toggle( DigitalOut* led)
{
	*led = !(*led);
}



/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
DigitalOut NFC04A1_GPO_ReadPin( DigitalOut *mMISO )
{
  return *mMISO;
}

/**
  * @brief  This function initialize the GPIO to manage the NFCTAG LPD pin
  * @param  None
  * @retval None
  */

void NFC04A1_LPD_Init( DigitalOut *mLPD )
{
	*mLPD = 0;
}

/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
uint8_t NFC04A1_LPD_ReadPin( DigitalOut *mLPD )
{
  return mLPD -> read();
}

/**
  * @brief  This function get the GPIO value through GPIO
  * @param  None
  * @retval HAL GPIO pin status
  */
void NFC04A1_LPD_WritePin( uint8_t LpdPinState, DigitalOut *mLPD )
{
  mLPD -> write( LpdPinState );
}

/**
  * @brief  This function select the i2cChannel1 speed to communicate with NFCTAG
  * @param  i2cChannelspeedchoice Number from 0 to 5 to select i2cChannel speed
  * @param mi2cChannel : I2C channel
  * @retval HAL GPIO pin status
  */
void NFC04A1_Selecti2cSpeed( uint8_t i2cspeedchoice, I2C* mi2cChannel)
{
  
  switch( i2cspeedchoice )
  {
    case 0:
      
      mi2cChannel -> frequency(10000);
      break;
    
    case 1:
      
      mi2cChannel -> frequency(100000);
      break;
    
    case 2:
      
    mi2cChannel -> frequency(200000);
      break;
    
    case 3:
      
    mi2cChannel -> frequency(400000);
      break;
    
    case 4:

    mi2cChannel -> frequency(800000);
      break;
    
    case 5:
      
    mi2cChannel -> frequency(1000000);
      break;
    
    default:
      
    mi2cChannel -> frequency(1000000);
      break;
  }    

}

/**
 * @}
 */

/** @defgroup X_NUCLEO_NFC04A1_Private_Functions
 * @{
 */
/******************************** LINK EEPROM COMPONENT *****************************/

/**
  * @brief  Initializes peripherals used by the i2cChannel NFCTAG driver
  * @param mi2cChannel : I2C channel
  * @param mLPD
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO_Init( I2C* mi2cChannel, DigitalOut *mLPD )
{

  
  NFC04A1_LPD_Init( mLPD );

  NFC04A1_Selecti2cSpeed(3, mi2cChannel);

  return NFCTAG_OK;
}

/**
  * @brief  Write data, at specific address, through i2c to the ST25DV
  * @param  pData: pointer to the data to write
  * @param  DevAddr : Target device address
  * @param  TarAddr : i2c data memory address to write
  * @param  Size : Size in bytes of the value to be written
  * @param mi2cChannel : I2C channel
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO_MemWrite( const uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size, I2C* mi2cChannel )
{

  uint8_t ret = 4;
  
  uint8_t Addr = DevAddr;


  uint8_t buffer[2];
  buffer[0] = (uint8_t) (TarAddr>>8);
  buffer[1] = (uint8_t) (TarAddr&0xFF);


   char * pDataChar = (char*) pData;


  ret = mi2cChannel -> write(Addr, (const char*)buffer, 2 , true);

  // Address is not OK
  	if(ret != 0)
		return NFCTAG_ConvertStatus(ret);

	ret = mi2cChannel -> write(Addr, pDataChar, Size, false);

  return NFCTAG_ConvertStatus(ret);
}


/*
  * @brief  Reads data at a specific address from the NFCTAG.
  * @param  pData: pointer to store read data
  * @param  DevAddr : Target device address
  * @param  TarAddr : i2c data memory address to read
  * @param  Size : Size in bytes of the value to be read
  * @param mi2cChannel : I2C channel
  * @retval NFCTAG enum status
  */

NFCTAG_StatusTypeDef ST25DV_IO_MemRead( uint8_t * const pData, const uint8_t DevAddr, const uint16_t TarAddr, const uint16_t Size, I2C* mi2cChannel )
{

	uint8_t ret = 4;
	uint8_t Addr = DevAddr;

    uint8_t buffer[2];
    buffer[0] = (uint8_t) (TarAddr>>8);
    buffer[1] = (uint8_t) (TarAddr&0xFF);

    ret = mi2cChannel -> write(Addr, (const char*)buffer , 2 , false);

    // Address is not OK
  	if(ret != 0)
  		return NFCTAG_ConvertStatus(ret);

	char * pDataChar = (char*) pData;

	ret = mi2cChannel -> read(DevAddr, pDataChar, Size, false );

	return NFCTAG_ConvertStatus(ret);
}

NFCTAG_StatusTypeDef NFCTAG_ConvertStatus(uint8_t ret) {
	if (ret == 0) {
		return NFCTAG_OK;
	} else if ((ret == 2) || (ret == 3)) {
		return NFCTAG_NACK;
	} else {
		return NFCTAG_ERROR;
	}
}


/**
  * @brief  Reads data at current address from the NFCTAG.
  * @param  pData: pointer to store read data
  * @param  DevAddr : Target device address
  * @param  Size : Size in bytes of the value to be read
  * @retval NFCTAG enum status
  */
NFCTAG_StatusTypeDef ST25DV_IO_Read( uint8_t * const pData, const uint8_t DevAddr, const uint16_t Size, I2C* mi2cChannel )
{
	//this has to change( send  addr then read)
	int i = 0;
	uint8_t ret = 4;

	char * pDataChar = (char*) pData;
	uint8_t ReadAddr = DevAddr | 1u;
	ret = mi2cChannel -> read(ReadAddr, pDataChar, 1, false );


// Tell slave we need to read 1byte from the current register
	while(mi2cChannel -> read( 0 ) != 0) {
		pData[i++] = mi2cChannel -> read( 0 );

	}


  return NFCTAG_ConvertStatus( ret);
}


/**
* @brief  Checks if target device is ready for communication
* @note   This function is used with Memory devices
* @param  DevAddr : Target device address
* @param  mi2cChannel : I2C channel
* @retval NFCTAG enum status
*/
NFCTAG_StatusTypeDef ST25DV_IO_IsDeviceReady( const uint8_t DevAddr, const uint32_t Trials, I2C* mi2cChannel)
{ 
  int ret = 4;
  uint32_t count = 0;

  uint8_t Addr = DevAddr;


  while ((count++ < Trials && ret) ) {
	  ret = mi2cChannel -> write(Addr, NULL, 0 , false);
  }
  return NFCTAG_ConvertStatus(ret);
}




/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
