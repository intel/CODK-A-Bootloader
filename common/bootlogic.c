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
#include <utils.h>
#include <partition.h>

__weak void cpu_init(void)
{
}

__weak void soc_init(void)
{
}

__weak void dnx(void)
{
}


__weak void early_fixup(void)
{
}

__weak void early_sign_of_life(void)
{
}

__weak void board_early_init(void)
{
}

__weak void timer_init(void)
{
}

__weak void rtc_init(void)
{
}

__weak void serial_init(void)
{
}

__weak void gpio_init(void)
{
}

__weak void watchdog_init(void)
{
}

__weak void spi_early_init(void)
{
}

__weak void i2c_early_init(void)
{
}

__weak void board_misc_init(void)
{
}

__weak void interrupt_init(void)
{
}

__weak void init_flash(void)
{
}

__weak void late_fixup(void)
{
}

__weak void late_sign_of_life(void)
{
}

__weak void board_late_init(void)
{
}

/*
 * Boot logic
 */
__weak void boot_logic(void)
{
	enum wake_sources wake_source;
	enum reset_reasons reset_reason;
	enum boot_targets boot_target;

	override_boot_logic();
	wake_source = get_wake_source();
	reset_reason = get_reset_reason();
	boot_target = get_boot_target();
	charger_connected(&boot_target);
	override_boot_target(wake_source, reset_reason, &boot_target);
	boot(wake_source, reset_reason, boot_target);
}

__weak void override_boot_logic(void)
{
}

__weak enum wake_sources get_wake_source(void)
{
	return WAKE_UNKNOWN;
}

__weak enum reset_reasons get_reset_reason(void)
{
	return RESET_UNKNOWN;
}

__weak enum boot_targets get_boot_target(void)
{
	return TARGET_MAIN;
}

__weak void set_boot_target(enum boot_targets boot_target)
{
}

__weak void override_boot_target(enum wake_sources wake_source,
				 enum reset_reasons reset_reason,
				 enum boot_targets *boot_target)
{
}

__weak enum boot_flags get_boot_flags(void)
{
	return BOOT_NORMAL;
}

__weak void set_boot_flags(enum boot_flags boot_flags)
{
}

__weak void mpu_init(enum boot_targets boot_target)
{
}

__weak void boot(enum wake_sources wake_source, enum reset_reasons reset_reason,
		 enum boot_targets boot_target)
{
	void (*do_boot[]) (enum wake_sources, enum reset_reasons,
			   enum boot_targets) = {
	[TARGET_MAIN] = boot_main,
		    [TARGET_CHARGING] = boot_charging,
		    [TARGET_WIRELESS_CHARGING] = boot_charging,
		    [TARGET_RECOVERY] = boot_recovery,
		    [TARGET_FLASHING] = boot_flashing,
		    [TARGET_FACTORY] = boot_factory,
		    [TARGET_OTA] = boot_ota,
		    [TARGET_DTM] = boot_dtm,
		    [TARGET_CERTIFICATION] = boot_default,
		    [TARGET_RESERVED_0] = boot_default,
		    [TARGET_APP_1] = boot_default,
		    [TARGET_APP_2] = boot_default,
		    [TARGET_RESERVED_1] = boot_default,
		    [TARGET_RESERVED_2] = boot_default,
		    [TARGET_RESERVED_3] = boot_default,
		    [TARGET_RESERVED_4] = boot_default};

#if defined(CONFIG_MPU)
	mpu_init();
#endif

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_WATCHDOG_TIMEOUT)
	watchdog_start(CONFIG_WATCHDOG_TIMEOUT);
	WATCHDOG_RESET();
#endif

	if (boot_target < ARRAY_SIZE(do_boot))
		do_boot[boot_target] (wake_source, reset_reason, boot_target);
	boot_default(wake_source, reset_reason, boot_target);
}

__weak void watchdog_start(uint16_t period_sec)
{
}

__weak void watchdog_reset(void)
{
}

__weak void load_binary(uint32_t address)
{
}

__weak void start_binary(uint32_t address)
{

}

__weak void charger_connected(enum boot_targets *boot_target)
{
	if (is_conductive_charger_connected(boot_target))
		set_boot_target(*boot_target);
	else if (is_wireless_charger_connected(boot_target))
		set_boot_target(*boot_target);
}

__weak uint8_t is_wireless_charger_connected(enum boot_targets *boot_target)
{
	return 0;
}

__weak uint8_t is_conductive_charger_connected(enum boot_targets * boot_target)
{
	return 0;
}

__weak void boot_main(enum wake_sources wake_source,
		      enum reset_reasons reset_reason,
		      enum boot_targets boot_target)
{
		((void (*)(void))(QUARK_FLASH_START_ADDR + VERSION_HEADER_SIZE))();
}

__weak void boot_default(enum wake_sources wake_source,
			 enum reset_reasons reset_reason,
			 enum boot_targets boot_target)
{
	boot_main(wake_source, reset_reason, boot_target);
}

__weak void boot_charging(enum wake_sources wake_source,
			  enum reset_reasons reset_reason,
			  enum boot_targets boot_target)
{
}

__weak void boot_recovery(enum wake_sources wake_source,
			  enum reset_reasons reset_reason,
			  enum boot_targets boot_target)
{
}

__weak void boot_flashing(enum wake_sources wake_source,
			  enum reset_reasons reset_reason,
			  enum boot_targets boot_target)
{
}

__weak void boot_factory(enum wake_sources wake_source,
			 enum reset_reasons reset_reason,
			 enum boot_targets boot_target)
{
}

__weak void boot_ota(enum wake_sources wake_source,
		     enum reset_reasons reset_reason,
		     enum boot_targets boot_target)
{
}

__weak void boot_dtm(enum wake_sources wake_source,
		     enum reset_reasons reset_reason,
		     enum boot_targets boot_target)
{
}
