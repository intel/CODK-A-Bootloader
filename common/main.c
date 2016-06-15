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

#include <bootlogic.h>
#include <usb/usb_api.h>
#include <printk.h>
#include <panic_dump.h>

int main(void)
{
	cpu_init();

#if defined(CONFIG_INTERRUPT_MODE)
	interrupt_init();
#endif

	soc_init();
	board_early_init();
	gpio_init();

#if defined(CONFIG_TIMER)
	timer_init();
#endif

#if defined(CONFIG_USB)
	usb_init();
#endif

#if defined(CONFIG_SERIAL)
	serial_init();
#endif


#if defined(CONFIG_DNX)
	dnx();
#endif

#if defined(CONFIG_HARDWARE_CHARGING)
	hardware_charging_init();
	hardware_charging();
#endif

#if defined(CONFIG_PANIC)
	panic_dump();
#endif

	early_fixup();
	early_sign_of_life();

#if defined(CONFIG_RTC)
	rtc_init();
#endif

#if defined(CONFIG_WATCHDOG)
	watchdog_init();
#endif

#if defined(CONFIG_SPI)
	spi_init();
#endif

#if defined(CONFIG_I2C)
	i2c_init();
#endif

	board_misc_init();

	late_fixup();
	late_sign_of_life();
	board_late_init();
	boot_logic();

#if defined(CONFIG_GPIO_TST)
	gpio_tst();
#endif

	while (1) ;

	return 0;
}
