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

#ifndef PANIC_DUMP_H__
#define PANIC_DUMP_H__

#include <stdint.h>
#include <stdbool.h>
// FIXME: cleanup files shared between QRK_SE and FC
#ifdef CONFIG_CHIP_QUARK_SE
#include "quark_se/board_intel.h"
#include "quark_se/boot_config.h"
#else
#include "board_intel.h"
#include "boot_config.h"
#endif

#if defined(CONFIG_PANIC)

#define PANIC_DATA_MAGIC 0x21636e50	/* "Pnc!" */

/* CONFIG_PANIC_VOLATILE_END have to be defined in project/board specific file*/
#ifndef CONFIG_PANIC_VOLATILE_END
#define PANIC_VOLATILE_END 0
#else
#define PANIC_VOLATILE_END CONFIG_PANIC_VOLATILE_END
#endif

#define PANIC_SIZE_ADDR  (PANIC_VOLATILE_END - sizeof(uint32_t))
#define PANIC_VOLATILE_BASE (PANIC_VOLATILE_END - *(uint32_t *) PANIC_SIZE_ADDR)

#define PANIC_NVM_BASE (PART_PANIC_START * ERASE_PAGE_SIZE)
#define PANIC_NVM_SIZE (PART_PANIC_SIZE * ERASE_PAGE_SIZE)
#define PANIC_NVM_END (PANIC_NVM_BASE + PANIC_NVM_SIZE)

enum PANIC_TYPE {
	PANIC_SUCCESS,
	PANIC_FAILURE
};

void panic_dump(void);
uint32_t panic_volatile_get(uint32_t * panic, uint32_t * size);
void panic_nvm_init(uint32_t ** where, uint32_t size);
void panic_nvm_erase(uint32_t where, uint32_t size);
void panic_nvm_write(uint32_t * panic_volatile, uint32_t * panic_nvm,
		     uint32_t size);
void panic_nvm_write_word(uint32_t * address, uint32_t value);
bool panic_occurred(void);

#endif

#endif /* PANIC_DUMP_H__ */
