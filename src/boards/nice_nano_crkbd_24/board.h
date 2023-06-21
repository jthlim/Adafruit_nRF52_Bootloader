/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Nick Winans
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _NICENANO_H
#define _NICENANO_H

#define _PINNUM(port, pin)    ((port)*32 + (pin))

#define UICR_REGOUT0_VALUE UICR_REGOUT0_VOUT_3V3

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER       1
#define LED_PRIMARY_PIN   _PINNUM(0, 15) // Blue
#define LED_STATE_ON      1

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER    2  // none connected at all
#define BUTTON_1          _PINNUM(0, 18)  // unusable: RESET
#define BUTTON_2          _PINNUM(0, 19)  // no connection
#define BUTTON_PULL       NRF_GPIO_PIN_PULLUP

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER  "Nice Keyboards"
#define BLEDIS_MODEL         "nice!nano"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
#define USB_DESC_VID           0x239A
#define USB_DESC_UF2_PID       0x00B3
#define USB_DESC_CDC_ONLY_PID  0x00B3

#define UF2_PRODUCT_NAME  "nice!nano"
#define UF2_VOLUME_LABEL  "NICENANO"
#define UF2_BOARD_ID      "nRF52840-nicenano"
#define UF2_INDEX_URL     "https://nicekeyboards.com/docs/nice-nano"

#define JAVELIN_QSPI_SCK_PIN 20
#define JAVELIN_QSPI_CSN_PIN 36
#define JAVELIN_QSPI_IO0_PIN 17
#define JAVELIN_QSPI_IO1_PIN 38
#define JAVELIN_QSPI_IO2_PIN NRF_QSPI_PIN_NOT_CONNECTED
#define JAVELIN_QSPI_IO3_PIN NRF_QSPI_PIN_NOT_CONNECTED
#define JAVELIN_QSPI_READOC NRF_QSPI_READOC_READ2IO
#define JAVELIN_QSPI_WRITEOC NRF_QSPI_WRITEOC_PP
#define JAVELIN_QSPI_SCK_FREQ NRF_QSPI_FREQ_32MDIV1
#define JAVELIN_QSPI_POWER_PIN 10
#define JAVELIN_QSPI_32_BIT_ADDRESSING 0
#define JAVELIN_QSPI_32_BIT_ADDRESSING_OPCODE 0xb7

#endif // _NICENANO_H
