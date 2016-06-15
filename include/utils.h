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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include <stddef.h>
#include <scss_registers.h>

#define __weak __attribute__((weak))

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

#define BUILD_BUG_ON_ZERO(e) (sizeof(char[1 - 2 * !!(e)]) - 1)

#define __must_be_array(a) \
	BUILD_BUG_ON_ZERO(__builtin_types_compatible_p(typeof(a), typeof(&a[0])))

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0])) + __must_be_array(a)

#define IS_BIG_ENDIAN (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)

#define between(a, b, c) (((a) >= (b)) || ((a) <= (c)))

#define strict_between(a, b, c) (((a) > (b)) || ((a) < (c)))

#define die() while(1)

#define get_32k_time() MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_AONC_CNT)

#define interrupt_lock(saved);
#define interrupt_unlock(saved);

/*!
 * Common driver function return codes
 */
typedef enum {
    DRV_RC_OK = 0,
    DRV_RC_FAIL,
    DRV_RC_TIMEOUT,
    DRV_RC_INVALID_CONFIG,              /*!< configuration parameters incorrect */
    DRV_RC_MODE_NOT_SUPPORTED,          /*!< controller/driver doesn't support this mode (master/slave) */
    DRV_RC_CONTROLLER_IN_USE,           /*!< controller is in use */
    DRV_RC_CONTROLLER_NOT_ACCESSIBLE,   /*!< controller not accessible from this core */
    DRV_RC_INVALID_OPERATION,           /*!< attempt to perform an operation that is invalid */
    DRV_RC_WRITE_PROTECTED,             /*!< Attempt to erase/program a memory region that is write protected */
    DRV_RC_READ_PROTECTED,              /*!< Attempt to read a memory region that is read protected */
    DRV_RC_CHECK_FAIL,                  /*!< Read back data after programming does not match the word written to memory */
    DRV_RC_OUT_OF_MEM,                  /*!< Attempt to program data outside the memory boundaries */
    DRV_RC_ERASE_PC,                    /*!< Attempt to write/erase executable code currently in use */
    DRV_RC_TOTAL_RC_CODE                /*!< Number of DRIVER_API_RC codes (used to extend this enum) */
} DRIVER_API_RC;


#define CLEAR_MMIO_BIT(reg, bit) \
 do { \
    uint32_t saved = 0; \
    interrupt_lock(saved) \
    MMIO_REG_VAL_FROM_BASE(reg, 0) &= ~(1 << bit); \
    interrupt_unlock(saved); \
 } while(0)

#define SET_MMIO_BIT(reg, bit) \
 do { \
    uint32_t saved = 0; \
    interrupt_lock(saved) \
    MMIO_REG_VAL_FROM_BASE(reg, 0) |= (1 << bit); \
    interrupt_unlock(saved); \
 } while(0)


void mdelay(uint32_t delay);

#endif
