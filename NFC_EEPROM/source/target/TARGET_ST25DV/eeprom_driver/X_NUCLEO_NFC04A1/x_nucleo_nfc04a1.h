/**
  ******************************************************************************
  * @file    x_nucleo_nfc04a1.h
  * @author  MMY Application Team
  * @version $Revision: 3351 $
  * @date    $Date: 2017-01-25 17:28:08 +0100 (Wed, 25 Jan 2017) $
  * @brief   This file contains definitions for the x_nucleo_nfc04a1.c
  *          board specific functions.
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

#include "st25dv_driver.h"
#include "mbed.h"
//#include "PinNames.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_NFC04A1_H
#define __X_NUCLEO_NFC04A1_H



/* Includes ------------------------------------------------------------------*/
//#include "ST25DV/st25dv.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup X_NUCLEO_NFC04A1
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup X_NUCLEO_NFC04A1_Exported_Types
  * @{
  */
/**
 * @brief  NFC04A1 Led enumerator definition
 */
typedef enum 
{
  GREEN_LED = 0,
  BLUE_LED ,
  YELLOW_LED
}NFC04A1_Led_E;

/**
 * @brief  NFC04A1 Ack Nack enumerator definition
 */
typedef enum 
{
  I2CANSW_ACK = 0,
  I2CANSW_NACK
}NFC04A1_I2CANSW_E;



/**
 * @brief  NFC04A1 Led structure definition
 */
 
typedef struct
{
  uint16_t          NFC04A1_LED_PIN;
}NFC04A1_Led_TypeDef;

/**
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/** @defgroup X_NUCLEO_NFC04A1_Exported_Constants
  * @{
  */
#define ST25DV_I2C_SPEED                  NFC04A1_ST25DV_I2C_SPEED_1M
#define NFC04A1_I2C_TIMEOUT               ST25DV_I2C_TIMEOUT
/**
  * @}
  */

/* External variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/** @defgroup ST25DV_NUCLEO_Exported_Functions
  * @{
  */
void NFC04A1_LED_Toggle( DigitalOut* led);
DigitalOut NFC04A1_GPO_ReadPin( DigitalOut *mMISO );
void NFC04A1_LPD_Init( DigitalOut *mLPD );
uint8_t NFC04A1_LPD_ReadPin( void );
void NFC04A1_LPD_WritePin( uint8_t LpdPinState, DigitalOut *mLPD );
void NFC04A1_SelectI2cSpeed( uint8_t i2cspeedchoice, I2C* mi2cChannel);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */ 



#endif /* __X_NUCLEO_NFC04A1_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
