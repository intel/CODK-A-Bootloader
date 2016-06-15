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

#ifndef SCSS_REGISTERS_H_
#define SCSS_REGISTERS_H_

#include <stdint.h>

/* MMIO Register Access Macros. */
#define MMIO_REG_VAL(addr) (*((volatile uint32_t *)addr))
#define MMIO_REG_ADDR(addr) ((volatile uint32_t *)addr)
#define MMIO_REG_VAL_FROM_BASE(base, offset) \
		(*((volatile uint32_t *)(base+offset)))

#define CALC_PIN_MUX_SELECT_VAL(pin_offset, mode) (mode << pin_offset * 2)

/*  Pin Muxing */
/*  TODO: DB - Change these to offsets from SCSS */

#define QRK_PMUX_SLEW_RATE_0 (0xB0800910)
#define QRK_PMUX_SLEW_RATE_1 (0xB0800914)
#define QRK_PMUX_SLEW_RATE_2 (0xB0800918)
#define QRK_PMUX_SLEW_RATE_3 (0xB080091C)
#define PIN_MUX_SLEW_4mA_driver (0xFFFFFFFF)

/* USB PLL register */
#define USB_PLL_CFG0 (0xB0800014)
#define USB_PLL_PDLD (1) /* USB PLL enable bit */
#define USB_PLL_LOCK (1<<14) /* USB PLL locked */
#define USB_PLL_CFG0_DEFAULT (0x00003104) /* Default USB PLL setup value */

/* USB clock enable bit */
#define CCU_USB_CLK_EN (0x2)

#define QRK_PMUX_SELECT_0               0XB0800930
#define QRK_PMUX_SELECT_1               0XB0800934
#define QRK_PMUX_SELECT_2               0XB0800938
#define QRK_PMUX_SELECT_3               0XB080093C
#define QRK_PMUX_SELECT_4               0XB0800940
#define QRK_PMUX_SELECT_5               0XB0800948

#define QRK_PMUX_SEL_MODEA              0
#define QRK_PMUX_SEL_MODEB              1
#define QRK_PMUX_SEL_MODEC              2
#define QRK_PMUX_SEL_MODED              3

#define SCSS_REGISTER_BASE              0xB0800000
#define SCSS_REG_VAL(offset) \
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, offset)

#define SCSS_OSC0_CFG0                  0x0 /* Hybrid Oscillator Config 0 */
#define SCSS_OSC0_STAT1                 0x4 /* Hybrid Oscillator Status 1 */
#define SCSS_OSC0_CFG1                  0x8 /* Hybrid Oscillator Config 1 */

#define OSC0_STAT1_LOCK_INTERNAL        (1 << 0)
#define OSC0_STAT1_LOCK_XTAL            (1 << 1)

#define OSC0_CFG1_XTAL_OSC_EN_MASK      (1 << 0)
#define OSC0_CFG1_INTERNAL_OSC_EN_MASK  (1 << 1)
#define OSC0_CFG1_XTAL_OSC_OUT_MASK     (1 << 3)

#define OSC0_CFG1_INTERNAL_OSC_TRIM_BIT 20

#define OSC0_CFG1_XTAL_OSC_TRIM_5_55_PF (0x7 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_6_18_PF (0x6 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_6_82_PF (0x5 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_7_45_PF (0x4 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_8_08_PF (0x3 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_8_71_PF (0x2 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_9_34_PF (0x1 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_10_PF   (0x0 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_10_61_PF (0xF << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_11_24_PF (0xE << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_11_88_PF (0xD << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_12_51_PF (0xC << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_13_14_PF (0xB << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_13_77_PF (0xA << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_14_04_PF (0x9 << 16)
#define OSC0_CFG1_XTAL_OSC_TRIM_15_03_PF (0x8 << 16)


#define SCSS_CCU_SYS_CLK_CTL_OFFSET     0x38
#define SCSS_CCU_RTC_CLK_DIV_EN         (1 << 2)
#define SCSS_RTC_CLK_DIV_1_SECOND       0x78

/* Interrupt masking */
#define QRK_INT_UNMASK_IA               (~0x00000001)

#define SCSS_INT_GPIO_MASK_OFFSET       0X46C
#define SCSS_INT_PWM_TIMER_MASK_OFFSET  0X470
#define SCSS_INT_USB_MASK_OFFSET        0x474
#define SCSS_INT_RTC_MASK_OFFSET        0x478
#define SCSS_INT_WATCHDOG_MASK_OFFSET   0x47C
#define SCSS_INT_MPR_MASK_OFFSET        0x4BC
#define SCSS_INT_MAILBOX_MASK           0x4A0
#define QRK_INT_PWM_UNMASK_QRK          (~0x1)
#define QRK_INT_WDT_UNMASK_QRK          (~0x1)
#define QRK_INT_RTC_UNMASK_QRK          (~0x1)
#define QRK_INT_MPR_UNMASK_QRK          (~0x1)
#define QRK_INT_USB_UNMASK_QRK          (~0x1)
#define QRK_INT_MPR_UNMASK_QRK_HLT      (~0x10000)
#define QRK_SCSS_P_STS                  0x560
#define QRK_SCSS_P_STS_HIR_PRBE_MODE_EN (1 << 26)
#define QRK_SCSS_PERIPH_CFG0_OFFSET     0x804
#define QRK_SCSS_PERIPH_CFG0_WDT_ENABLE (1 << 1)
/* Identification */
#define SCSS_ID                         0x0128
/* Revision */
#define SCSS_REV                        0x012C
/* Sensor Subsystem */
#define SCSS_SS_CFG                     0x0600
#define SCSS_SS_STS                     0x0604
#define ARC_HALT_REQ_A                  (1 << 25)
#define ARC_RUN_REQ_A                   (1 << 24)
/* Always On Counter */
#define SCSS_AONC_CNT                   0x0700
#define SCSS_AONC_CFG                   0x0704
#define AONC_CNT_DIS                    0
#define AONC_CNT_EN                     1
/* Mailbox Channel Status */
#define SCSS_MBOX_CHALL_STS             0x0AC0

/* Power management */
#define SLP_CFG_BASE                    (SCSS_REGISTER_BASE + 0x550)
#define SLP_CFG_IO_STATE_RET_HOLD       (1<<10)
#define SLP_CFG_IO_STATE_RET_EN         (1<<9)
#define SLP_CFG_LPMODE_EN               (1<<8)
#define SLP_CFG_RTC_DIS                 (1<<7)
#define SLP_CFG_VRET_EN                 (1<<5)
#define SLP_CFG_VRET_SEL_135            0x0000000b
#define SLP_CFG_VRET_SEL_140            0x0000000c
#define SLP_CFG_VRET_SEL_150            0x0000000d
#define SLP_CFG_VRET_SEL_160            0x0000000e
#define SLP_CFG_VRET_SEL_170            0x0000000f
#define SLP_CFG_VRET_SEL_180            0x00000010
#define PM1C_BASE                       (SCSS_REGISTER_BASE + 0x518)
#define PM1C_SLPEN_BIT                  13
#define PM1C_SLPEN                      (1<<PM1C_SLPEN_BIT)
#define SCSS_RSTC                       0x570
#define RSTC_WARM_RESET                 (1 << 1)
#define RSTC_COLD_RESET                 (1 << 3)

/* Clock Gating */
/* TODO: DB - Change these to offsets from SCSS */
#define QRK_CLKGATE_CTRL                0xB0800018
#define QRK_CLKGATE_CTRL_WDT_ENABLE     (1 << 10)
#define QRK_CLKGATE_CTRL_RTC_ENABLE     (1 << 11)
#define QRK_CLKGATE_CTRL_PWM_ENABLE     (1 << 12)

#define PERIPH_CLK_GATE_CTRL            (SCSS_REGISTER_BASE + 0x018)

/* PWM */
#define QRK_PWM_BASE_ADDR               0xB0000800
#define QRK_PWM_N_REGS_LEN              0x14
#define QRK_PWM_N_LCNT2_LEN             0x04

/* PWM register offsets */
#define QRK_PWM_N_LOAD_COUNT1           0x00
#define QRK_PWM_N_CURRENT_VALUE         0x04
#define QRK_PWM_N_CONTROL               0x08
#define QRK_PWM_N_EOI                   0x0C
#define QRK_PWM_N_INT_STATUS            0x10
#define QRK_PWM_N_LOAD_COUNT2           0xB0

#define QRK_PWMS_INT_STATUS             0xA0
#define QRK_PWMS_EOI                    0xA4
#define QRK_PWMS_RAW_INT_STATUS         0xA8
#define QRK_PWMS_COMP_VERSION           0xAC

#define QRK_PWM_CONTROL_ENABLE          (1 << 0)
#define QRK_PWM_CONTROL_MODE_PERIODIC   (1 << 1)
#define QRK_PWM_CONTROL_INT_MASK        (1 << 2)
#define QRK_PWM_CONTROL_PWM_OUT         (1 << 3)

/* soc GPIOs */
#define SOC_GPIO_BASE_ADDR              0xB0000C00
#define SOC_GPIO_AON_BASE_ADDR          (SCSS_REGISTER_BASE + 0xB00)
#define SOC_GPIO_SWPORTA_DR             0x00
#define SOC_GPIO_SWPORTA_DDR            0x04
#define SOC_GPIO_SWPORTA_CTL            0x08
#define SOC_GPIO_INTEN                  0x30
#define SOC_GPIO_INTMASK                0x34
#define SOC_GPIO_INTTYPE_LEVEL          0x38
#define SOC_GPIO_INTPOLARITY            0x3c
#define SOC_GPIO_INTSTATUS              0x40
#define SOC_GPIO_RAW_INTSTATUS          0x44
#define SOC_GPIO_DEBOUNCE               0x48
#define SOC_GPIO_PORTA_EOI              0x4c
#define SOC_GPIO_EXT_PORTA              0x50
#define SOC_GPIO_LS_SYNC                0x60
#define SOC_GPIO_INT_BOTHEDGE           0x68
#define SOC_GPIO_CONFIG_REG2            0x70
#define SOC_GPIO_CONFIG_REG1            0x74

/* Watchdog Timer */
#define QRK_WDT_CRR_VAL                 0x76

#define QRK_WDT_BASE_ADDR               0xB0000000
#define QRK_WDT_CR                      0x00
#define QRK_WDT_TORR                    0x04
#define QRK_WDT_CCVR                    0x08
#define QRK_WDT_CRR                     0x0C
#define QRK_WDT_STAT                    0x10
#define QRK_WDT_EOI                     0x14
#define QRK_WDT_COMP_PARAM_5            0xE4
#define QRK_WDT_COMP_PARAM_4            0xE8
#define QRK_WDT_COMP_PARAM_3            0xEC
#define QRK_WDT_COMP_PARAM_2            0xF0
#define QRK_WDT_COMP_PARAM_1            0xF4
#define QRK_WDT_COMP_VERSION            0xF8
#define QRK_WDT_COMP_TYPE               0xFC

#define QRK_WDT_CR_ENABLE               (1 << 0)
#define QRK_WDT_CR_INT_ENABLE           (1 << 1) /* interrupt mode enable - mode1*/

/*  RTC */
#define QRK_RTC_BASE_ADDR               0xB0000400
#define QRK_RTC_CCVR                    0x00
#define QRK_RTC_CMR                     0x04
#define QRK_RTC_CLR                     0x08
#define QRK_RTC_CCR                     0x0C
#define QRK_RTC_STAT                    0x10
#define QRK_RTC_RSTAT                   0x14
#define QRK_RTC_EOI                     0x18
#define QRK_RTC_COMP_VERSION            0x1C

#define QRK_RTC_INTERRUPT_ENABLE        (1 << 0)
#define QRK_RTC_INTERRUPT_MASK          (1 << 1)
#define QRK_RTC_ENABLE                  (1 << 2)
#define QRK_RTC_WRAP_ENABLE             (1 << 3)

/*  MPR */
#define QRK_MPR_BASE_ADDR               0xB0400000
#define QRK_MPR_REGS_LEN                0x04
#define QRK_MPR_MAX_NUM                 3           /* 4 MPRs 0-3) */
#define QRK_MPR_CFG_LOCK                (1 << 31)
#define QRK_MPR_CFG_EN                  (1 << 30)
#define QRK_MPR_CFG_HOST_WRITE_EN       (1 << 20)
#define QRK_MPR_CFG_SS_WRITE_EN         (1 << 21)
#define QRK_MPR_CFG_OTHER_WRITE_EN      (1 << 22)
#define QRK_MPR_CFG_HOST_READ_EN        (1 << 24)
#define QRK_MPR_CFG_SS_READ_EN          (1 << 25)
#define QRK_MPR_CFG_OTHER_READ_EN       (1 << 26)
#define QRK_MPR0_CFG                    0x00
#define QRK_MPR1_CFG                    0x04
#define QRK_MPR2_CFG                    0x08
#define QRK_MPR3_CFG                    0x0C
#define QRK_MPR_VDATA                   0x10
#define QRK_MPR_VSTS                    0x14

/* I2C */
#define SOC_I2C_0_BASE                  (0xb0002800)
#define SOC_I2C_1_BASE                  (0xb0002c00)

/* SPI */
#define SOC_MST_SPI0_REGISTER_BASE      (0xB0001000)
#define SOC_MST_SPI1_REGISTER_BASE      (0xB0001400)
#define SOC_SLV_SPI_REGISTER_BASE       (0xB0001800)

/* USB */
#define SOC_USB_BASE_ADDR               (0xB0500000)

/* Mailbox Interrupt*/
#define IO_REG_MAILBOX_INT_MASK         (SCSS_REGISTER_BASE + 0x4A0)

/* Mailbox Base Address */
#define IO_REG_MAILBOX_BASE             (SCSS_REGISTER_BASE + 0xA00)

/* Mailbox offsets */
#define MAILBOX_CTRL_OFFSET             (0x00)
#define MAILBOX_DATA0_OFFSET            (0x04)
#define MAILBOX_DATA1_OFFSET            (0x08)
#define MAILBOX_DATA2_OFFSET            (0x0C)
#define MAILBOX_DATA3_OFFSET            (0x10)
#define MAILBOX_STATUS_OFFSET           (0x14)

/* Mailbox addresses for a given index */
#define MBX_CTRL(_x_) MMIO_REG_VAL( \
	(IO_REG_MAILBOX_BASE + (0x18*(_x_)) + MAILBOX_CTRL_OFFSET))
#define MBX_DAT0(_x_) MMIO_REG_VAL( \
	(IO_REG_MAILBOX_BASE + (0x18*(_x_)) + MAILBOX_DATA0_OFFSET))
#define MBX_DAT1(_x_) MMIO_REG_VAL( \
	(IO_REG_MAILBOX_BASE + (0x18*(_x_)) + MAILBOX_DATA1_OFFSET))
#define MBX_DAT2(_x_) MMIO_REG_VAL( \
	(IO_REG_MAILBOX_BASE + (0x18*(_x_)) + MAILBOX_DATA2_OFFSET))
#define MBX_DAT3(_x_) MMIO_REG_VAL( \
	(IO_REG_MAILBOX_BASE + (0x18*(_x_)) + MAILBOX_DATA3_OFFSET))
#define MBX_STS(_x_) MMIO_REG_VAL( \
	(IO_REG_MAILBOX_BASE + (0x18*(_x_)) + MAILBOX_STATUS_OFFSET))

#define MBX_CHALL_STS SCSS_REG_VAL(SCSS_MBOX_CHALL_STS)

/* DMAC Interrupts Mask Register Offsets */
#define INT_DMA_CHANNEL_0_MASK_REG      (0x480)
#define INT_DMA_CHANNEL_1_MASK_REG      (0x484)
#define INT_DMA_CHANNEL_2_MASK_REG      (0x488)
#define INT_DMA_CHANNEL_3_MASK_REG      (0x48C)
#define INT_DMA_CHANNEL_4_MASK_REG      (0x490)
#define INT_DMA_CHANNEL_5_MASK_REG      (0x494)
#define INT_DMA_CHANNEL_6_MASK_REG      (0x498)
#define INT_DMA_CHANNEL_7_MASK_REG      (0x49C)
#define INT_DMA_ERROR_MASK_REG          (0x4B8)


/* Pin Muxing */
#define MUX_BASE    QRK_PMUX_SELECT_0
/* Read current Mux reg, Zero pin bits, OR new mode into these bits, write reg - thereby preserving other pin mode settings */
#define SET_MUX_REG( mux_reg, mode, pin ) MMIO_REG_VAL(mux_reg) = ( MMIO_REG_VAL(mux_reg) & ~( 3 << (pin*2) ) ) | ( mode << (pin*2) )
/* Calculate mux register address from pin number and calculate pin number within that register - call SET_MUX_REG */
#define SET_PIN_MODE( pin_no, mode) SET_MUX_REG( (((pin_no/16)*4 )+ MUX_BASE), mode, pin_no % 16)
/* Determine Pin mode */
#define GET_PIN_MODE_MUXING(mux_reg, pin)   ((MMIO_REG_VAL(mux_reg) >> (pin*2)) & 0x3)
#define GET_PIN_MODE(pin_no) GET_PIN_MODE_MUXING(((((pin_no/16)*4 )+ MUX_BASE)), pin_no % 16)

#define AHB_CTRL_REG    (SCSS_REGISTER_BASE + 0x034)

/* Typical value used to unmask single source interrupts in SCSS. */
#define ENABLE_SSS_INTERRUPTS           ~(0x00000001 << 8)
#define DISABLE_SSS_INTERRUPTS          (0x00000001 << 8)


/* Latest Masks */
#define INT_SS_ADC_ERR_MASK             (0x400) /*  Sensor Subsystem Interrupt Routing Mask 0 */
#define INT_SS_ADC_IRQ_MASK             (0x404) /*  Sensor Subsystem Interrupt Routing Mask 1 */
#define INT_SS_GPIO_0_INTR_MASK         (0x408) /*  Sensor Subsystem Interrupt Routing Mask 2 */
#define INT_SS_GPIO_1_INTR_MASK         (0x40C) /*  Sensor Subsystem Interrupt Routing Mask 3 */
#define INT_SS_I2C_0_ERR_MASK           (0x410) /*  Sensor Subsystem Interrupt Routing Mask 4 */
#define INT_SS_I2C_0_RX_AVAIL_MASK      (0x414) /*  Sensor Subsystem Interrupt Routing Mask 5 */
#define INT_SS_I2C_0_TX_REQ_MASK        (0x418) /*  Sensor Subsystem Interrupt Routing Mask 6 */
#define INT_SS_I2C_0_STOP_DETECTED_MASK (0x41C) /*  Sensor Subsystem Interrupt Routing Mask 7 */
#define INT_SS_I2C_1_ERR_MASK           (0x420) /*  Sensor Subsystem Interrupt Routing Mask 8 */
#define INT_SS_I2C_1_RX_AVAIL_MASK      (0x424) /*  Sensor Subsystem Interrupt Routing Mask 9 */
#define INT_SS_I2C_1_TX_REQ_MASK        (0x428) /*  Sensor Subsystem Interrupt Routing Mask 10 */
#define INT_SS_I2C_1_STOP_DETECTED_MASK (0x42C) /*  Sensor Subsystem Interrupt Routing Mask 11 */
#define INT_SS_SPI_0_ERR_INT_MASK       (0x430) /*  Sensor Subsystem Interrupt Routing Mask 12 */
#define INT_SS_SPI_0_RX_AVAIL_MASK      (0x434) /*  Sensor Subsystem Interrupt Routing Mask 13 */
#define INT_SS_SPI_0_TX_REQ_MASK        (0x438) /*  Sensor Subsystem Interrupt Routing Mask 14 */
#define INT_SS_SPI_1_ERR_INT_MASK       (0x43C) /*  Sensor Subsystem Interrupt Routing Mask 15 */
#define INT_SS_SPI_1_RX_AVAIL_MASK      (0x440) /*  Sensor Subsystem Interrupt Routing Mask 16 */
#define INT_SS_SPI_1_TX_REQ_MASK        (0x444) /*  Sensor Subsystem Interrupt Routing Mask 17 */

#define INT_I2C_MST_0_MASK              (0x448) /*  Host Processor Interrupt Routing Mask 0 */
#define INT_I2C_MST_1_MASK              (0x44C) /*  Host Processor Interrupt Routing Mask 1 */
#define INT_SPI_MST_0_MASK              (0x454) /*  Host Processor Interrupt Routing Mask 2 */
#define INT_SPI_MST_1_MASK              (0x458) /*  Host Processor Interrupt Routing Mask 3 */
#define INT_SPI_SLV_MASK                (0x45C) /*  Host Processor Interrupt Routing Mask 4 */
#define INT_UART_0_MASK                 (0x460) /*  Host Processor Interrupt Routing Mask 5 */
#define INT_UART_1_MASK                 (0x464) /*  Host Processor Interrupt Routing Mask 6 */
#define INT_I2S_MASK                    (0x468) /*  Host Processor Interrupt Routing Mask 7 */
#define INT_GPIO_MASK                   (0x46C) /*  Host Processor Interrupt Routing Mask 8 */
#define INT_PWM_TIMER_MASK              (0x470) /*  Host Processor Interrupt Routing Mask 9 */
#define INT_USB_MASK                    (0x474) /*  Host Processor Interrupt Routing Mask 10 */
#define INT_RTC_MASK                    (0x478) /*  Host Processor Interrupt Routing Mask 11 */
#define INT_WATCHDOG_MASK               (0x47C) /*  Host Processor Interrupt Routing Mask 12 */
#define INT_DMA_CHANNEL_0_MASK          (0x480) /*  Host Processor Interrupt Routing Mask 13 */
#define INT_DMA_CHANNEL_1_MASK          (0x484) /*  Host Processor Interrupt Routing Mask 14 */
#define INT_DMA_CHANNEL_2_MASK          (0x488) /*  Host Processor Interrupt Routing Mask 15 */
#define INT_DMA_CHANNEL_3_MASK          (0x48C) /*  Host Processor Interrupt Routing Mask 16 */
#define INT_DMA_CHANNEL_4_MASK          (0x490) /*  Host Processor Interrupt Routing Mask 17 */
#define INT_DMA_CHANNEL_5_MASK          (0x494) /*  Host Processor Interrupt Routing Mask 18 */
#define INT_DMA_CHANNEL_6_MASK          (0x498) /*  Host Processor Interrupt Routing Mask 19 */
#define INT_DMA_CHANNEL_7_MASK          (0x49C) /*  Host Processor Interrupt Routing Mask 20 */
#define INT_MAILBOX_MASK                (0x4A0) /*  Host Processor Interrupt Routing Mask 21 */
#define INT_COMPARATORS_SS_HALT_MASK    (0x4A4) /*  Host Processor Interrupt Routing Mask 22 */
#define INT_COMPARATORS_HOST_HALT_MASK  (0x4A8) /*  Host Processor Interrupt Routing Mask 23 */
#define INT_COMPARATORS_SS_MASK         (0x4AC) /*  Host Processor Interrupt Routing Mask 24 */
#define INT_COMPARATORS_HOST_MASK       (0x4B0) /*  Host Processor Interrupt Routing Mask 25 */
#define INT_SYSTEM_PMU_MASK             (0x4B4) /*  Host Processor Interrupt Routing Mask 26 */
#define INT_DMA_ERROR_MASK              (0x4B8) /*  Host Processor Interrupt Routing Mask 27 */
#define INT_SRAM_CONTROLLER_MASK        (0x4BC) /*  Host Processor Interrupt Routing Mask 28 */
#define INT_FLASH_CONTROLLER_0_MASK     (0x4C0) /*  Host Processor Interrupt Routing Mask 29 */
#define INT_FLASH_CONTROLLER_1_MASK     (0x4C4) /*  Host Processor Interrupt Routing Mask 30 */
#define INT_AON_TIMER_MASK              (0x4C8) /*  Host Processor Interrupt Routing Mask 31 */
#define INT_ADC_PWR_MASK                (0x4CC) /*  Host Processor Interrupt Routing Mask 32 */
#define INT_ADC_CALIB_MASK              (0x4D0) /*  Host Processor Interrupt Routing Mask 33 */
#define INT_AON_GPIO_MASK               (0x4D4) /*  Host Processor Interrupt Routing Mask 34 */
#define LOCK_INT_MASK_REG               (0x4D8) /*  Interrupt Mask Lock Register */

/* sticky registers */
#define SCSS_GPS0                       0x100
#define SCSS_GPS1                       0x104
#define SCSS_GPS2                       0x108
#define SCSS_GPS3                       0x10C

/* Reset Status */
#define RSTS                            0x574
#define SW_WRST_MASK                    (1 << 0)
#define HOST_WDG_WRST                   (1 << 1)
#define SS_WDG_WRS                      (1 << 2)
#define HOST_HALT_WRST                  (1 << 3)
#define SS_HALT_WRST                    (1 << 4)

/* SCSS Mailbox Interrupt masking/unmasking */
/* First byte of the register is for the Quark */
#define SCSS_INT_MAILBOX_START_CHANNEL  0x00

#define SOC_MBX_INT_UNMASK(channel) SCSS_REG_VAL(SCSS_INT_MAILBOX_MASK) &= \
		~( 1 << ( channel + SCSS_INT_MAILBOX_START_CHANNEL ) )
#define SOC_MBX_INT_MASK(channel) SCSS_REG_VAL(SCSS_INT_MAILBOX_MASK) |=  \
		( 1 << ( channel + SCSS_INT_MAILBOX_START_CHANNEL ) )

/* Interrupt vector mapping */
#define SOC_I2C0_INTERRUPT              0x0
#define SOC_I2C1_INTERRUPT              0x1
#define SOC_SPIM0_INTERRUPT             0x2
#define SOC_SPIM1_INTERRUPT             0x3
#define SOC_SPIS0_INTERRUPT             0x4
#define SOC_UART0_INTERRUPT             0x5
#define SOC_UART1_INTERRUPT             0x6
#define SOC_GPIO_INTERRUPT              0x8
#define SOC_PWM_INTERRUPT               0x9
#define SOC_USB_INTERRUPT               0xa
#define SOC_RTC_INTERRUPT               0xb
#define SOC_WDT_INTERRUPT               0xc
#define SOC_MBOX_INTERRUPT              0x15
#define SOC_MPR_INTERRUPT               0x19
#define SOC_GPIO_AON_INTERRUPT          0x1F

#define SOC_UNMASK_INTERRUPTS(_driver_) (MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, _driver_) &= QRK_INT_UNMASK_IA)
#define SOC_MASK_INTERRUPTS(_driver_)   (MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, _driver_) |= ~(QRK_INT_UNMASK_IA))

#endif /* SCSS_REGISTERS_H_ */
