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

#include "bootlogic.h"
#include "reboot.h"
#include "utils.h"

/** Mapping of General Purpose Sticky Scratchpad 0 */
/**
    31                                                                  0
    *----------------*----------------*----------------*----------------*
    |    BOOT FLAGS  |  RESET REASONS |  BOOT TARGETS  | WAKE SOURCES   | SCSS_GPS0
    *----------------*----------------*----------------*----------------*
**/

#define RESET_REASON_POS            0x10
#define RESET_REASON_MASK           (0xFF << RESET_REASON_POS)

#define BOOT_TARGETS_POS            0x08
#define BOOT_TARGETS_MASK           (0xFF << BOOT_TARGETS_POS)

#define WAKE_SOURCES_POS            0x00
#define WAKE_SOURCES_MASK           (0xFF << WAKE_SOURCES_POS)

#define BOOT_FLAG_POS               0x18
#define BOOT_FLAG_MASK              (0xFF << BOOT_FLAG_POS)

#define GET_REBOOT_REG(addr,mask,pos) ((MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,addr) & mask)>> pos)

#define SET_REBOOT_REG(addr,mask,pos,val)\
    do {\
           uint32_t reg = MMIO_REG_VAL_FROM_BASE (SCSS_REGISTER_BASE,addr) ; \
           reg &= ~mask; \
           reg |= val << pos; \
           MMIO_REG_VAL_FROM_BASE (SCSS_REGISTER_BASE,addr) = (MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,addr) & ~mask) | reg; \
       } while(0)


/* Private function prototypes */
enum reset_reasons get_hard_reset_reason(void);

enum reset_reasons get_hard_reset_reason(void)
{
    uint32_t reset_reason_reg = MMIO_REG_VAL_FROM_BASE (SCSS_REGISTER_BASE,RSTS);

    /*
     * Priority: RESET_HOST_HW_WATCHDOG >> RESET_SS_HW_WATCHDOG >>
     * RESET_INT_QRK >> RESET_INT_SS >> RESET_SW
     */

    if (reset_reason_reg & (HOST_WDG_WRST  ))
        return RESET_HOST_HW_WATCHDOG;
    if (reset_reason_reg & (SS_WDG_WRS))
        return RESET_SS_HW_WATCHDOG;
    if (reset_reason_reg & (HOST_HALT_WRST))
        return RESET_INT_QRK;
    if (reset_reason_reg & (SS_HALT_WRST))
        return RESET_INT_SS;
    if (reset_reason_reg & (SW_WRST_MASK))
        return RESET_SW;
    return (RESET_HW);
}

enum reset_reasons get_reset_reason(void)
{
    /* get reset reason from RSTS hardware register */
    enum reset_reasons reset_reason_val = get_hard_reset_reason();

    SET_REBOOT_REG(SCSS_GPS0,RESET_REASON_MASK,RESET_REASON_POS,reset_reason_val);

    return reset_reason_val;
}

enum boot_targets get_boot_target(void)
{
    return GET_REBOOT_REG(SCSS_GPS0,BOOT_TARGETS_MASK,BOOT_TARGETS_POS);
}

void set_boot_target(enum boot_targets boot_target)
{
    SET_REBOOT_REG(SCSS_GPS0,BOOT_TARGETS_MASK,BOOT_TARGETS_POS,boot_target);
}

enum wake_sources get_wake_source(void)
{
    return GET_REBOOT_REG(SCSS_GPS0,WAKE_SOURCES_MASK,WAKE_SOURCES_POS);
}

void set_wake_source(enum wake_sources wake_source)
{
    SET_REBOOT_REG(SCSS_GPS0,WAKE_SOURCES_MASK,WAKE_SOURCES_POS,wake_source);
}

enum boot_flags get_boot_flags(void)
{
    return GET_REBOOT_REG(SCSS_GPS0,BOOT_FLAG_MASK,BOOT_FLAG_POS);
}

void set_boot_flags(enum boot_flags boot_flag)
{
    SET_REBOOT_REG(SCSS_GPS0,BOOT_FLAG_MASK,BOOT_FLAG_POS,boot_flag);
}

void reboot(enum boot_targets target)
{
    set_boot_target(target);

    /* Perform warm reset*/
    SCSS_REG_VAL(SCSS_SS_CFG) |= ARC_HALT_REQ_A;
    SCSS_REG_VAL(SCSS_RSTC) = RSTC_WARM_RESET;
}

