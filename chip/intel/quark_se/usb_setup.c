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

/* SoC-specific USB initialazation */

#include "gpio/gpio.h"
#include "scss_registers.h"
#include "printk.h"
#include "utils.h"

#include "usb/usb_api.h"

#define VENABLE_USB_REGULATOR	28

/**
 * Enables USB driver clock and regulator.
 */
static void enable_usb()
{
	gpio_cfg_data_t config;

	/* Setup and turn on USB PLL */
	MMIO_REG_VAL(USB_PLL_CFG0) = USB_PLL_CFG0_DEFAULT | USB_PLL_PDLD;

	/*Wait for the PLL lock */
	int count = 500;
	while(count-- && (0 == (MMIO_REG_VAL(USB_PLL_CFG0) & USB_PLL_LOCK))){}

	/* Turn on the clock gate */
	MMIO_REG_VAL(AHB_CTRL_REG) |= CCU_USB_CLK_EN;

	pr_info("USB PLL Configured count: %d\n", count);

	/* activate regulator after USB clocks are stabilized */
	config.gpio_type = GPIO_OUTPUT;
	soc_gpio_set_config(SOC_GPIO_32, VENABLE_USB_REGULATOR, &config);
	soc_gpio_write(SOC_GPIO_32, VENABLE_USB_REGULATOR, 1);
}

void platform_usb_init(void)
{
	/* platform specific init of USB core */
	MMIO_REG_VAL(QRK_PMUX_SLEW_RATE_0) = PIN_MUX_SLEW_4mA_driver;
	MMIO_REG_VAL(QRK_PMUX_SLEW_RATE_1) = PIN_MUX_SLEW_4mA_driver;
	MMIO_REG_VAL(QRK_PMUX_SLEW_RATE_2) = PIN_MUX_SLEW_4mA_driver;
	MMIO_REG_VAL(QRK_PMUX_SLEW_RATE_3) = PIN_MUX_SLEW_4mA_driver;

	enable_usb();

	/* Init the USB driver */
	SCSS_REG_VAL(SCSS_INT_USB_MASK_OFFSET) = QRK_INT_USB_UNMASK_QRK;
}

void platform_usb_release()
{
	/* Disable clock */
	MMIO_REG_VAL(AHB_CTRL_REG) &= ~CCU_USB_CLK_EN;

	/* Stopping USB PLL */
	MMIO_REG_VAL(USB_PLL_CFG0) &= ~USB_PLL_PDLD;

	/* Disable regulator */
	soc_gpio_write(SOC_GPIO_32, 28, 0);
}

void poll_usb(void)
{
	static uint32_t date = 0;
	uint32_t new_date;
	uint32_t elapsed;

	new_date = get_32k_time();
	if (new_date > date) {
		elapsed = new_date - date;
	} else {
		elapsed = new_date + (INT32_MAX - date);
	}
	if (elapsed > 32) {
		usb_ISR();
		date = new_date;
	}
}
