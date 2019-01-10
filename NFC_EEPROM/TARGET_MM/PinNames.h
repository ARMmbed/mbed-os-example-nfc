/* 
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this list 
 *      of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form, except as embedded into a Nordic Semiconductor ASA 
 *      integrated circuit in a product or a software update for such product, must reproduce 
 *      the above copyright notice, this list of conditions and the following disclaimer in 
 *      the documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of its contributors may be 
 *      used to endorse or promote products derived from this software without specific prior 
 *      written permission.
 *
 *   4. This software, with or without modification, must only be used with a 
 *      Nordic Semiconductor ASA integrated circuit.
 *
 *   5. Any software provided in binary or object form under this license must not be reverse 
 *      engineered, decompiled, modified and/or disassembled. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"
#include "nrf_gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        PIN_INPUT,
        PIN_OUTPUT
    } PinDirection;

#define PORT_SHIFT 3

///> define macro producing for example Px_y = NRF_GPIO_PIN_MAP(x, y)
#define PinDef(port_num, pin_num) P##port_num##_##pin_num = NRF_GPIO_PIN_MAP(port_num, pin_num)

    typedef enum
    {
        PinDef(0, 0), // P0_0 = 0...
        PinDef(0, 1),
        PinDef(0, 2),
        PinDef(0, 3),
        PinDef(0, 4),
        PinDef(0, 5),
        PinDef(0, 6),
        PinDef(0, 7),
        PinDef(0, 8),
        PinDef(0, 9),
        PinDef(0, 10),
        PinDef(0, 11),
        PinDef(0, 12),
        PinDef(0, 13),
        PinDef(0, 14),
        PinDef(0, 15),
        PinDef(0, 16),
        PinDef(0, 17),
        PinDef(0, 18),
        PinDef(0, 19),
        PinDef(0, 20),
        PinDef(0, 21),
        PinDef(0, 22),
        PinDef(0, 23),
        PinDef(0, 24),
        PinDef(0, 25),
        PinDef(0, 26),
        PinDef(0, 27),
        PinDef(0, 28),
        PinDef(0, 29),
        PinDef(0, 30),
        PinDef(0, 31),

        PinDef(1, 0), //P1_1 = 32...
        PinDef(1, 1),
        PinDef(1, 2),
        PinDef(1, 3),
        PinDef(1, 4),
        PinDef(1, 5),
        PinDef(1, 6),
        PinDef(1, 7),
        PinDef(1, 8),
        PinDef(1, 9),
        PinDef(1, 10),
        PinDef(1, 11),
        PinDef(1, 12),
        PinDef(1, 13),
        PinDef(1, 14),
        PinDef(1, 15),

        // Port0
        p0 = P0_0,
        p1 = P0_1,
        p2 = P0_2,
        p3 = P0_3,
        p4 = P0_4,
        p5 = P0_5,
        p6 = P0_6,
        p7 = P0_7,
        p8 = P0_8,
        p9 = P0_9,
        p10 = P0_10,
        p11 = P0_11,
        p12 = P0_12,
        p13 = P0_13,
        p14 = P0_14,
        p15 = P0_15,
        p16 = P0_16,
        p17 = P0_17,
        p18 = P0_18,
        p19 = P0_19,
        p20 = P0_20,
        p21 = P0_21,
        p22 = P0_22,
        p23 = P0_23,
        p24 = P0_24,
        p25 = P0_25,
        p26 = P0_26,
        p27 = P0_27,
        p28 = P0_28,
        p29 = P0_29,
        p30 = P0_30,
        p31 = P0_31,

        // Port1
        p32 = P1_0,
        p33 = P1_1,
        p34 = P1_2,
        p35 = P1_3,
        p36 = P1_4,
        p37 = P1_5,
        p38 = P1_6,
        p39 = P1_7,
        p40 = P1_8,
        p41 = P1_9,
        p42 = P1_10,
        p43 = P1_11,
        p44 = P1_12,
        p45 = P1_13,
        p46 = P1_14,
        p47 = P1_15,

        // Not connected
        NC = (int)0xFFFFFFFF,

        AC21              = P0_25,             // GPIO
        AD20              = P0_24,             // GPIO
        AIN6              = NC,                // ADC
        AIN7              = NC,                // ADC

        CELL_1v8          = P0_28,             // ADC AIN4
        CELL_3v8          = P0_29,             // ADC AIN5
        CELL_EMERG_RST    = P1_4,              // GPIO
        CELL_ON           = P1_3,              // GPIO
        CELL_PWR_EN       = P1_6,              // GPIO
        CELL_VCORE_1v1    = P0_4,              // ADC CTS_OPTIONAL AIN2

        CONFIG0           = P0_13,             // GPIO
        CONFIG1           = P0_14,             // GPIO
        CONFIG2           = P0_15,             // GPIO
        CONFIG3           = P0_16,             // GPIO

        DCDC_I2C_SCL      = P1_15,             // I2C1
        DCDC_I2C_SDA      = P1_14,             // I2C1
        I2C_SCL           = DCDC_I2C_SCL,      // I2C1
        I2C_SDA           = DCDC_I2C_SDA,      // I2C1

        GPS_3v3           = P0_3,              // ADC AIN1
        GPS_NFC_I2C_SCL   = P0_27,             // I2C2
        GPS_NFC_I2C_SDA   = P0_26,             // I2C2
        GPS_PWR_EN        = P1_10,             // GPIO
        I2C_SCL2          = GPS_NFC_I2C_SCL,   // I2C2
        I2C_SDA2          = GPS_NFC_I2C_SDA,   // I2C2
        
        QSPI_CLK          = P0_19,             // QSPI
        QSPI_CS_N         = P0_17,             // QSPI
        QSPI_DIO0         = P0_20,             // QSPI
        QSPI_DIO1         = P0_21,             // QSPI
        QSPI_DIO2         = P0_22,             // QSPI
        QSPI_DIO3         = P0_23,             // QSPI

        SPI_MOSI          = QSPI_DIO0,         // Single mode SPI
        SPI_MISO          = QSPI_DIO1,         // Single mode SPI
        SPI_CLK           = QSPI_CLK,          // Single mode SPI
        SPI_CS            = QSPI_CS_N,         // Single mode SPI

        SPI_SCK           = SPI_CLK,           // mbed-os/features/storage/system_storage/SystemStorage.cpp

        RESET_N           = P0_18,             // NRF

        // Note: some button and led pins are defined due
        // to hello world type examples that require them

        /**** LED ****/
        STATUS_LED        = P1_5,              // GPIO
        LED1              = STATUS_LED,        // GPIO
        LED2              = STATUS_LED,        // GPIO
        LED3              = STATUS_LED,        // GPIO
        LED4              = STATUS_LED,        // GPIO
        LED_RED           = STATUS_LED,        // GPIO
        LED_GREEN         = STATUS_LED,        // GPIO
        LED_BLUE          = STATUS_LED,        // GPIO

        /**** BUTTON ****/
        BUTTON1           = NC,

        /**** WiFi ESP8266 ****/
        PROG_WIFI_N       = P1_12,             // GPIO
        WIFI_3v3          = P0_2,              // ADC AIN0
        WIFI_CELLULAR_CTS = P0_7,              // UART1 CTS_TRACECLK
        WIFI_CELLULAR_RTS = P0_5,              // UART1 RTS_AIN3
        WIFI_CELLULAR_RX  = P0_8,              // UART1 RxD
        WIFI_CELLULAR_TX  = P0_6,              // UART1 TxD
        WIFI_N            = P1_7,              // GPIO
        WIFI_PWR_EN       = P1_8,              // GPIO
        WIFI_SLEEP_WAKEUP = P1_11,             // GPIO

        /**** NFC ****/
        NFC_FIELD_DETECT  = P1_13,             // GPIO
        uC_NFC_A          = P0_9,              // RF
        uC_NFC_B          = P0_10,             // RF
        NFC1              = uC_NFC_A,          // RF
        NFC2              = uC_NFC_B,          // RF

        /**** Debug UART2 ****/
        MAIN_TX           = P1_1,              // UART2
        MAIN_RX           = P1_2,              // UART2

        /*
        USBRX/USBTX needed due to GreenteaSerial::GreenteaSerial() mbed-os/features/frameworks/greentea-client/source/greentea_serial.cpp
        */     
        USBRX            = MAIN_RX,           // UART2
        USBTX            = MAIN_TX,           // UART2

        RX_PIN_NUMBER     = MAIN_RX,           // UART2
        TX_PIN_NUMBER     = MAIN_TX,           // UART2
        CTS_PIN_NUMBER    = NC,                // UART2
        RTS_PIN_NUMBER    = NC,                // UART2

        STDIO_UART_TX     = TX_PIN_NUMBER,     // UART2
        STDIO_UART_RX     = RX_PIN_NUMBER,     // UART2
        STDIO_UART_CTS    = CTS_PIN_NUMBER,    // UART2
        STDIO_UART_RTS    = RTS_PIN_NUMBER,    // UART2

        // SPI_PSELMOSI0 = P1_13,
        // SPI_PSELMISO0 = P1_14,
        // SPI_PSELSS0 = P1_12,
        // SPI_PSELSCK0 = P1_15,

        // SPI_PSELMOSI1 = P1_2,
        // SPI_PSELMISO1 = P1_3,
        // SPI_PSELSS1 = P1_1,
        // SPI_PSELSCK1 = P1_4,

        // SPIS_PSELMOSI = P1_2,
        // SPIS_PSELMISO = P1_3,
        // SPIS_PSELSS = P1_1,
        // SPIS_PSELSCK = P1_4,

        /**** I2C Instance 0 ****/
        I2C_SDA0          = I2C_SDA,
        I2C_SCL0          = I2C_SCL,

        // D0 = P1_1,
        // D1 = P1_2,
        // D2 = P1_3,
        // D3 = P1_4,
        // D4 = P1_5,
        // D5 = P1_6,
        // D6 = P1_7,
        // D7 = P1_8,

        // D8 = P1_10,
        // D9 = P1_11,
        // D10 = P1_12,
        // D11 = P1_13,
        // D12 = P1_14,
        // D13 = P1_15,

        // D14 = p26,
        // D15 = p27,

        // A0 = p3,
        // A1 = p4,
        // A2 = p28,
        // A3 = p29,
        // A4 = p30,
        // A5 = p31,

        /**** QSPI pins ****/
        QSPI1_IO0         = QSPI_DIO0,
        QSPI1_IO1         = QSPI_DIO1,
        QSPI1_IO2         = QSPI_DIO2,
        QSPI1_IO3         = QSPI_DIO3,
        QSPI1_SCK         = QSPI_CLK,
        QSPI1_CSN         = QSPI_CS_N,

        /**** QSPI FLASH pins ****/
        QSPI_FLASH1_IO0   = QSPI1_IO0,
        QSPI_FLASH1_IO1   = QSPI1_IO1,
        QSPI_FLASH1_IO2   = QSPI1_IO2,
        QSPI_FLASH1_IO3   = QSPI1_IO3,
        QSPI_FLASH1_SCK   = QSPI1_SCK,
        QSPI_FLASH1_CSN   = QSPI1_CSN
    } PinName;

    typedef enum
    {
        PullNone = 0,
        PullDown = 1,
        PullUp = 3,
        PullDefault = PullUp
    } PinMode;

#ifdef __cplusplus
}
#endif

#endif


