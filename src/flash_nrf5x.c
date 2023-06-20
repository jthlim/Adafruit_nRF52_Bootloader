/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
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

#include "flash_nrf5x.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "nrf_qspi.h"
#include "nrf_sdm.h"
#include <string.h>

#define FLASH_PAGE_SIZE 4096
#define FLASH_CACHE_INVALID_ADDR 0xffffffff

static bool IsXip(uint32_t addr) { return addr >= 0x12000000; }
static uint32_t GetXipOffset(uint32_t addr) { return addr - 0x12000000; }

static uint32_t _fl_addr = FLASH_CACHE_INVALID_ADDR;
static uint8_t _fl_buf[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

void QspiWaitForCommandCompletion() {
  while (!NRF_QSPI->EVENTS_READY) {
  }
  NRF_QSPI->EVENTS_READY = 0;
}

void QspiWaitForFlashBusyCompletion() {
  const int READ_STATUS_REGISTER_OPCODE = 5;
  do {
    NRF_QSPI->CINSTRDAT0 = 0;
    NRF_QSPI->CINSTRCONF =
        (READ_STATUS_REGISTER_OPCODE << QSPI_CINSTRCONF_OPCODE_Pos) |
        (QSPI_CINSTRCONF_LENGTH_2B << QSPI_CINSTRCONF_LENGTH_Pos) |
        QSPI_CINSTRCONF_WIPWAIT_Msk;
    QspiWaitForCommandCompletion();
  } while (NRF_QSPI->CINSTRDAT0 & 1);
}

void QspiErase(size_t flashOffset, size_t length) {
  while (length) {
    NRF_QSPI->ERASE.PTR = flashOffset;
    if (length >= 0x10000 && (flashOffset & 0xffff) == 0) {
      NRF_QSPI->ERASE.LEN = QSPI_ERASE_LEN_LEN_64KB;
      flashOffset += 0x10000;
      length -= 0x10000;
    } else {
      NRF_QSPI->ERASE.LEN = QSPI_ERASE_LEN_LEN_4KB;
      flashOffset += 0x1000;
      length -= 0x1000;
    }
    NRF_QSPI->TASKS_ERASESTART = 1;
    QspiWaitForCommandCompletion();
    QspiWaitForFlashBusyCompletion();
  }
}

void QspiWrite(size_t flashOffset, const void *data, size_t length) {
  NRF_QSPI->WRITE.DST = flashOffset;
  NRF_QSPI->WRITE.SRC = (size_t)data;
  NRF_QSPI->WRITE.CNT = length;
  NRF_QSPI->TASKS_WRITESTART = 1;

  QspiWaitForCommandCompletion();
  QspiWaitForFlashBusyCompletion();
}

void QspiInitializePin(int pin) {
  if (pin == NRF_QSPI_PIN_NOT_CONNECTED) {
    return;
  }

  nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
}

void StartQspi() {
  nrf_gpio_cfg_output(JAVELIN_QSPI_SCK_PIN);
  nrf_gpio_pin_clear(JAVELIN_QSPI_SCK_PIN);

  NRF_QSPI->PSEL.SCK = JAVELIN_QSPI_SCK_PIN;
  NRF_QSPI->PSEL.CSN = JAVELIN_QSPI_CSN_PIN;
  NRF_QSPI->PSEL.IO0 = JAVELIN_QSPI_IO0_PIN;
  NRF_QSPI->PSEL.IO1 = JAVELIN_QSPI_IO1_PIN;
  NRF_QSPI->PSEL.IO2 = JAVELIN_QSPI_IO2_PIN;
  NRF_QSPI->PSEL.IO3 = JAVELIN_QSPI_IO3_PIN;

  QspiInitializePin(JAVELIN_QSPI_SCK_PIN);
  QspiInitializePin(JAVELIN_QSPI_CSN_PIN);
  QspiInitializePin(JAVELIN_QSPI_IO0_PIN);
  QspiInitializePin(JAVELIN_QSPI_IO1_PIN);
  QspiInitializePin(JAVELIN_QSPI_IO2_PIN);
  QspiInitializePin(JAVELIN_QSPI_IO3_PIN);

  NRF_QSPI->XIPOFFSET = 0;
  NRF_QSPI->IFCONFIG0 =
      (QSPI_IFCONFIG0_DPMENABLE_Enable << QSPI_IFCONFIG0_DPMENABLE_Pos) |
#if JAVELIN_QSPI_32_BIT_ADDRESSING
      (QSPI_IFCONFIG0_ADDRMODE_32BIT << QSPI_IFCONFIG0_ADDRMODE_Pos) |
#endif
      (JAVELIN_QSPI_WRITEOC << QSPI_IFCONFIG0_WRITEOC_Pos) |
      JAVELIN_QSPI_READOC;

  NRF_QSPI->IFCONFIG1 =
      (JAVELIN_QSPI_SCK_FREQ << QSPI_IFCONFIG1_SCKFREQ_Pos) |
      (QSPI_IFCONFIG1_SPIMODE_MODE0 << QSPI_IFCONFIG1_SPIMODE_Pos) |
      (QSPI_IFCONFIG0_DPMENABLE_Disable << QSPI_IFCONFIG0_DPMENABLE_Pos) |
      (1 << QSPI_IFCONFIG1_SCKDELAY_Pos);

#if JAVELIN_QSPI_32_BIT_ADDRESSING
  NRF_QSPI->ADDRCONF =
      (QSPI_ADDRCONF_WREN_Enable << QSPI_ADDRCONF_WREN_Pos) |
      (QSPI_ADDRCONF_WIPWAIT_Enable << QSPI_ADDRCONF_WIPWAIT_Pos) |
      (QSPI_ADDRCONF_MODE_Opcode << QSPI_ADDRCONF_MODE_Pos) |
      (0xb7 << QSPI_ADDRCONF_OPCODE_Pos);
#endif

  // NRF_QSPI->DPMDUR = 0x10001;

  NRF_QSPI->ENABLE = (QSPI_ENABLE_ENABLE_Enabled << QSPI_ENABLE_ENABLE_Pos);

  NRF_QSPI->EVENTS_READY = 0;
  NRF_QSPI->TASKS_ACTIVATE = 1;

  QspiWaitForCommandCompletion();
}

void QspiConfigureMemory() {
  const int QSPI_STD_CMD_RSTEN = 0x66;
  const int QSPI_STD_CMD_RST = 0x99;

  NRF_QSPI->CINSTRCONF =
      (QSPI_STD_CMD_RSTEN << QSPI_CINSTRCONF_OPCODE_Pos) |
      (QSPI_CINSTRCONF_LENGTH_1B << QSPI_CINSTRCONF_LENGTH_Pos) |
      QSPI_CINSTRCONF_WIPWAIT_Msk | QSPI_CINSTRCONF_WREN_Msk;
  QspiWaitForCommandCompletion();

  NRF_QSPI->CINSTRCONF =
      (QSPI_STD_CMD_RST << QSPI_CINSTRCONF_OPCODE_Pos) |
      (QSPI_CINSTRCONF_LENGTH_1B << QSPI_CINSTRCONF_LENGTH_Pos) |
      QSPI_CINSTRCONF_WIPWAIT_Msk | QSPI_CINSTRCONF_WREN_Msk;
  QspiWaitForCommandCompletion();

#if JAVELIN_QSPI_32_BIT_ADDRESSING &&                                          \
    defined(JAVELIN_QSPI_32_BIT_ADDRESSING_OPCODE)
  // Enable 4 byte mode.
  NRF_QSPI->CINSTRCONF =
      (JAVELIN_QSPI_32_BIT_ADDRESSING_OPCODE << QSPI_CINSTRCONF_OPCODE_Pos) |
      (QSPI_CINSTRCONF_LENGTH_1B << QSPI_CINSTRCONF_LENGTH_Pos) |
      QSPI_CINSTRCONF_WIPWAIT_Msk | QSPI_CINSTRCONF_WREN_Msk;
  QspiWaitForCommandCompletion();
#endif
}

void QspiActivate() {
  StartQspi();
  QspiConfigureMemory();

  // sd_clock_hfclk_request();
  // uint32_t isHfclkRunning = 0;
  // do {
  //   sd_clock_hfclk_is_running(&isHfclkRunning);
  // } while (!isHfclkRunning);
}

void flash_nrf5x_flush (bool need_erase)
{
  if ( _fl_addr == FLASH_CACHE_INVALID_ADDR ) return;

  // skip the write if contents matches
  if (memcmp(_fl_buf, (void *)_fl_addr, FLASH_PAGE_SIZE) != 0) {
    if (IsXip(_fl_addr)) {
      QspiErase(GetXipOffset(_fl_addr), 4096);
      QspiWrite(GetXipOffset(_fl_addr), _fl_buf, FLASH_PAGE_SIZE);
    } else {
      // - nRF52832 dfu via uart can miss incoming byte when erasing because cpu
      // is blocked for > 2ms. Since dfu_prepare_func_app_erase() already erase
      // the page for us, we can skip it here.
      // - nRF52840 dfu serial/uf2 are USB-based which are DMA and should have
      // no problems.
      //
      // Note: MSC uf2 does not erase page in advance like dfu serial
      if (need_erase) {
        PRINTF("Erase and ");
        nrfx_nvmc_page_erase(_fl_addr);
      }

      PRINTF("Write 0x%08lX\r\n", _fl_addr);
      nrfx_nvmc_words_write(_fl_addr, (uint32_t *)_fl_buf, FLASH_PAGE_SIZE / 4);
    }
  }

  _fl_addr = FLASH_CACHE_INVALID_ADDR;
}

void flash_nrf5x_write (uint32_t dst, void const *src, int len, bool need_erase)
{
  uint32_t newAddr = dst & ~(FLASH_PAGE_SIZE - 1);

  if ( newAddr != _fl_addr )
  {
    flash_nrf5x_flush(need_erase);
    _fl_addr = newAddr;
    memcpy(_fl_buf, (void *) newAddr, FLASH_PAGE_SIZE);
  }
  memcpy(_fl_buf + (dst & (FLASH_PAGE_SIZE - 1)), src, len);
}

void flash_init_qspi() {
  nrf_gpio_cfg(JAVELIN_QSPI_POWER_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_set(JAVELIN_QSPI_POWER_PIN);
  nrf_gpio_cfg(JAVELIN_QSPI_CSN_PIN, NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_set(JAVELIN_QSPI_CSN_PIN);

  QspiActivate();
}

//---------------------------------------------------------------------------
