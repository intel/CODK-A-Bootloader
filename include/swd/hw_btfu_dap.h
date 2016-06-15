/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HW_BTFU_DAP_H
#define HW_BTFU_DAP_H

#include <utils.h>
#include <gpio/gpio.h>
#include <gpio/soc_gpio.h>

//-----------------------------------------------------------------------------
//Definitions
//-----------------------------------------------------------------------------
typedef enum btfu_dap_error_tag {
	SWD_ERROR_OK = 0,	//successful
	SWD_ERROR_WAIT = 1,	//wait condition - retry it
	SWD_ERROR_FAULT = 2,
	SWD_ERROR_PROTOCOL = 3,
	SWD_ERROR_PARITY = 4,
	SWD_ERROR_MCU_LOCKED = 5,
	SWD_ERROR_INVALID_IDR = 6,
	SWD_ERROR_INVALID_IDCODE = 7,
	SWD_ERROR_FLASH_WRITE_FAILED = 8,
	SWD_ERROR_UNLOCK_FAILED = 9,
	SWD_ERROR_TIMEOUT_WAITING_RESET = 10,
	SWD_ERROR_TARGET_NOT_HALTED = 11,
	SWD_ERROR_DEVICE_ERASE_FAILED = 12,
	SWD_ERROR_TIMEOUT_HALT = 13,
	SWD_ERROR_DEBUG_POWER = 14,
	SWD_ERROR_CONFIG_WRITE_FAILED = 15,
	SWD_ERROR_WRITE_VERIFY_FAILED = 16,
	SWD_ERROR_CONFIG_GPIO = 17,
	SWD_ERROR_FORCE_U32B = 0x80000000,	// Allow better optimization

} btfu_dap_error_t;

/* ACK responses */
#define ACK_OK     1
#define ACK_WAIT   2
#define ACK_FAULT  4

/* Address of DP read registers */
#define DP_IDCODE  0
#define DP_CTRL    1
#define DP_RESEND  2
#define DP_RDBUFF  3

/* Adressses of DP write registers */
#define DP_ABORT   0
#define DP_STAT    1
#define DP_SELECT  2

/* AHB-AP registers */
#define AP_CSW 0		/* 0x0 */
#define AP_TAR 1		/* 0x4 */
#define AP_DRW 3		/* 0xC */
#define AP_IDR 3		/* In bank 0xf -address is 0xFC */

/* TAR Wrap value usually 1kB for this processor */
#define NRF51_TAR_WRAP    0x3FF	// Nordic

/* Valid values for the DP IDCODE register */
#define NRF51_DPID_1      0x0BB11477	// Nordic

/* Valid values for the APB-AP IDR register */
#define NRF51_APB_AP_ID_1 0x04770021

/* Bit fields for the CSW register */
#define AP_CSW_32BIT_TRANSFER   0x02
#define AP_CSW_AUTO_INCREMENT   0x10	//only used in some cases
#define AP_CSW_DBG_SW_ENABLE    (1 << 31)
#define AP_CSW_DEFAULT (AP_CSW_32BIT_TRANSFER | AP_CSW_DBG_SW_ENABLE)

/* Error bits in CTRL/STAT */
#define DP_CTRL_ORUNDETECT    (1 << 0)
#define DP_CTRL_STICKYORUN    (1 << 1)
#define DP_CTRL_TRNMODE0      (1 << 2)
#define DP_CTRL_TRNMODE1      (1 << 3)
#define DP_CTRL_STICKYCMP     (1 << 4)
#define DP_CTRL_STICKYERR     (1 << 5)
#define DP_CTRL_READOK        (1 << 6)
#define DP_CTRL_WDATAERR      (1 << 7)

/* Powerup request and acknowledge bits in CTRL/STAT */
#define DP_CTRL_CDBGPWRUPREQ  (1 << 28)
#define DP_CTRL_CDBGPWRUPACK  (1 << 29)
#define DP_CTRL_CSYSPWRUPREQ  (1 << 30)
#define DP_CTRL_CSYSPWRUPACK  (1 << 31)

/* Error clear bits in ABORT */
/* From http://infocenter.arm.com/help/topic/com.arm.doc.ddi0314h/DDI0314H_coresight_components_trm.pdf */
#define DP_ABORT_DAPABORT     (1 << 0)
#define DP_ABORT_STKCMPCLR    (1 << 1)
#define DP_ABORT_STKERRCLR    (1 << 2)
#define DP_ABORT_WDERRCLR     (1 << 3)
#define DP_ABORT_ORUNERRCLR   (1 << 4)

/* Number of times to retry an SWD operation when receiving
 * a WAIT response */
#define SWD_RETRY_COUNT 200

/* Number of times to retry the connection sequence */
#define CONNECT_RETRY_COUNT 3

/* Number of times to retry reading the AHB-IDR register when connecting */
#define AHB_IDR_RETRY_COUNT 20

/* Number of times to retry reading the CTLR/STAT
 * register while waiting for powerup acknowledge */
#define PWRUP_RETRY_COUNT 250

/* Number of times to retry reading status registers while
 * waiting for a debug event (such as a halt or soft reset) */
#define DEBUG_EVENT_RETRY_COUNT 200

#define NRF_SWD_PORT SOC_GPIO_32

/*
 * SWD PIN
*/
#define NRF_SWCLK_PIN  27
#define NRF_SWDIO_PIN   6

//-----------------------------------------------------------------------------
//Macros
//-----------------------------------------------------------------------------
/* JTAG to SWD bit sequence, transmitted LSB first */
#define JTAG2SWD 0xE79E

#define SWDIO_OUT(bit) \
  do {                 \
   if (bit)            \
    SWDIO_SET();       \
   else SWDIO_CLR();   \
  } while(0)

#define SWDIO_IN()                                                     \
    (!!(MMIO_REG_VAL_FROM_BASE(SOC_GPIO_BASE_ADDR, SOC_GPIO_EXT_PORTA) \
        & (1 << NRF_SWDIO_PIN)))

#define SWDIO_SET()                                                       \
  do {                                                                    \
    MMIO_REG_VAL_FROM_BASE((SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DR), 0) \
    |= (1 << NRF_SWDIO_PIN);                                              \
  } while(0)

#define SWDIO_CLR()                                                       \
  do {                                                                    \
    MMIO_REG_VAL_FROM_BASE((SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DR), 0) \
    &= ~(1 << NRF_SWDIO_PIN);                                             \
  } while(0)

#define SWCLK_SET()                                                       \
  do {                                                                    \
    MMIO_REG_VAL_FROM_BASE((SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DR), 0) \
    |= (1 << NRF_SWCLK_PIN);                                              \
  } while(0)

#define SWCLK_CLR()                                                       \
  do {                                                                    \
    MMIO_REG_VAL_FROM_BASE((SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DR), 0) \
    &= ~(1 << NRF_SWCLK_PIN);                                             \
  } while(0)

#define SWCLK_CYCLE()   \
  do {                  \
    SWCLK_CLR();        \
    SWCLK_SET();        \
  } while(0)

#define SWDIO_CYCLE()   \
  do {                  \
    SWDIO_SET();        \
    SWDIO_CLR();        \
  } while(0)

#define WRITE_BIT(bit)  \
  do {                  \
    SWDIO_OUT(bit);     \
    SWCLK_CLR();        \
    SWCLK_SET();        \
  } while(0)

#define READ_A_BIT(bit) \
  do {                  \
    SWCLK_CLR();        \
    bit = SWDIO_IN();   \
    SWCLK_SET();        \
  } while(0)

//-----------------------------------------------------------------------------
//Functions
//-----------------------------------------------------------------------------
extern btfu_dap_error_t hw_BtfuDapInit(void);
extern void hw_BtfuDapHibernate(void);
extern void hw_BtfuDapHardReset(void);
extern void hw_BtfuDapHardResetHold(void);
extern void hw_BtfuDapHardResetRelease(void);

extern btfu_dap_error_t hw_BtfuDapInitDp(void);
extern btfu_dap_error_t hw_BtfuDapReadApId(void);
extern btfu_dap_error_t hw_BtfuDapInitAhbAp(void);

extern uint32_t hw_BtfuDapReadMem(uint32_t addr);
extern void hw_BtfuDapWriteMem(uint32_t addr, uint32_t data);

extern btfu_dap_error_t hw_BtfuDapReadAP(int reg, uint32_t * data);
extern btfu_dap_error_t hw_BtfuDapWriteAP(int reg, uint32_t data);
#endif // !HW_BTFU_DAP_H
