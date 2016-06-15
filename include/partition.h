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

#ifndef __CTB_PARTITION_H__
#define __CTB_PARTITION_H__

#define PAGE_SIZE 2048
#define BASE_FLASH_ADDR 0x40000000
#define VERSION_HEADER_SIZE 0x30

#define ROM_PAGE_START 0
#define ROM_PAGE_NR 4
#define ROM_PAGE_START_ADDR 0xffffe000
#define BOOTSTRAP_FLASH_START_ADDR 0xffffe400

#define BOOT_PAGE_START 0
#define BOOT_PAGE_NR 30
#define BOOT_FLASH_START_ADDR (BASE_FLASH_ADDR + (BOOT_PAGE_START * PAGE_SIZE))
#define BOOT_RAM_START_ADDR 0xA8000400
#define BOOT_FLASH_SIZE (BOOT_PAGE_NR * PAGE_SIZE)
#define BOOT_RAM_SIZE (81920 - 1024)

#define PANIC_PAGE_START       30
#define PANIC_FLASH_START_ADDR (BASE_FLASH_ADDR + (PANIC_PAGE_START * PAGE_SIZE))
#define PANIC_PAGE_NR          2

#define QUARK_PAGE_START 32
#define QUARK_FLASH_START_ADDR (BASE_FLASH_ADDR + (QUARK_PAGE_START * PAGE_SIZE))
#define QUARK_PAGE_NR 72

#define ARC_PAGE_START 104
#define ARC_FLASH_START_ADDR (BASE_FLASH_ADDR + (ARC_PAGE_START * PAGE_SIZE))
#define ARC_PAGE_NR 76

#define BLE_CORE_FLASH_SIZE (262144)

/** USB Driver static data shared between
 * QRK Bootloader and QRK App
 * NOTE: Must match arduino101_firmware/device/quark_se/common/include/quark_se_mapping.h
 */
#define BOOTLOADER_SHARED_DATA  0xa8000400

#endif /*__CTB_PARTITION_H__ */
