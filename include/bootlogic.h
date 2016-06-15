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

#ifndef __BOOTLOGIC_H__
#define __BOOTLOGIC_H__

#include <stdint.h>

enum wake_sources {
	WAKE_UNKNOWN,
	WAKE_PWR_BUTTON,
	WAKE_BATTERY_INSERTION,
	WAKE_WIRED_PS_INSERTION,
	WAKE_WIRELESS_PS_INSERTION,
	WAKE_USB_INSERTION,
	WAKE_POWER_BUTTON,
};

enum reset_reasons {
	RESET_HW,                    /*  Initiated via HW ON/OFF POWER                   */
	RESET_DEBUG,
	RESET_SW,                    /*  Initiated via Reset Control register            */
	RESET_HOST_HW_WATCHDOG,      /*  Triggered by Quark Watchdog expiring         */
	RESET_SS_HW_WATCHDOG,        /*  Triggered by Arc Watchdog expiring              */
	RESET_INT_QRK,               /*  Triggered by event interrupt routed to Quark */
	RESET_INT_SS,                /*  Triggered by event interrupt routed to Arc      */
	RESET_UNKNOWN,
	RESET_REASON_SIZE,
};

enum boot_targets {
	TARGET_MAIN = 0x0,
	TARGET_CHARGING,
	TARGET_WIRELESS_CHARGING,
	TARGET_RECOVERY,
	TARGET_FLASHING,
	TARGET_FACTORY,
	TARGET_OTA,
	TARGET_DTM,
	TARGET_CERTIFICATION,
	TARGET_RESERVED_0,
	TARGET_APP_1,
	TARGET_APP_2,
	TARGET_RESERVED_1,
	TARGET_RESERVED_2,
	TARGET_RESERVED_3,
	TARGET_RESERVED_4
};

enum boot_flags {
	BOOT_NORMAL = 0x0,
	BOOT_PANIC,
	BOOT_WATCHDOG,
	BOOT_OTA_ONGOING,
	BOOT_OTA_COMPLETE,
	BOOT_OTA_FAILURE,
	BOOT_RESERVED
};


enum shutdown_types {
	SHUTDOWN = 0x0,
	CRITICAL_SHUTDOWN,
	THERMAL_SHUTDOWN,
	BATTERY_SHUTDOWN,
	REBOOT,
	SHUTDOWN_INVALID,
	MAX_SHUTDOWN_REASONS = 0xFF
};

/*
 * Hardware init
 */
void cpu_init(void);
void soc_init(void);
void dnx(void);
void early_fixup(void);
void early_sign_of_life(void);
void board_early_init(void);
void timer_init(void);
void rtc_init(void);
void serial_init(void);
void gpio_init(void);
void watchdog_init(void);
void spi_early_init(void);
void i2c_early_init(void);
void board_misc_init(void);
void interrupt_init(void);
void init_flash(void);
void late_fixup(void);
void late_sign_of_life(void);
void board_late_init(void);

/*
 * Boot logic
 */
void boot_logic(void);
void override_boot_logic(void);
enum wake_sources get_wake_source(void);
enum reset_reasons get_reset_reason(void);
enum boot_targets get_boot_target(void);
void set_boot_target(enum boot_targets);
void override_boot_target(enum wake_sources, enum reset_reasons,
			  enum boot_targets *);
enum boot_flags get_boot_flags(void);
void set_boot_flags(enum boot_flags boot_flags);
void boot(enum wake_sources, enum reset_reasons, enum boot_targets);
void watchdog_start(uint16_t period_sec);
void watchdog_reset(void);
void load_binary(uint32_t address);
void start_binary(uint32_t address);

void charger_connected(enum boot_targets *);
uint8_t is_wireless_charger_connected(enum boot_targets *);
uint8_t is_conductive_charger_connected(enum boot_targets *);

void boot_main(enum wake_sources wake_source,
	       enum reset_reasons reset_reason, enum boot_targets boot_target);

void boot_default(enum wake_sources wake_source,
		  enum reset_reasons reset_reason,
		  enum boot_targets boot_target);

void boot_charging(enum wake_sources wake_source,
		   enum reset_reasons reset_reason,
		   enum boot_targets boot_target);

void boot_recovery(enum wake_sources wake_source,
		   enum reset_reasons reset_reason,
		   enum boot_targets boot_target);

void boot_flashing(enum wake_sources wake_source,
		   enum reset_reasons reset_reason,
		   enum boot_targets boot_target);

void boot_factory(enum wake_sources wake_source,
		  enum reset_reasons reset_reason,
		  enum boot_targets boot_target);

void boot_ota(enum wake_sources wake_source,
	      enum reset_reasons reset_reason, enum boot_targets boot_target);

void boot_dtm(enum wake_sources wake_source,
	      enum reset_reasons reset_reason, enum boot_targets boot_target);
#endif /* __BOOTLOGIC_H__ */
