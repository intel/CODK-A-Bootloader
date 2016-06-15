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

#ifndef PANIC_H__
#define PANIC_H__

#include <stdint.h>

#define PANIC_DATA_MAGIC 0x21636e50 /* "Pnc!" */

/**
 * Structure for data panic in RAM.
 */
struct panic_data_footer {
    uint8_t arch;               /*!<Architecture (PANIC_ARCH_*) */
    uint8_t struct_version;     /*!<Structure version */
    uint8_t flags;              /*!<Flags (PANIC_DATA_FLAG_*) */
    uint8_t reserved;           /*!<Reserved; set 0 */
    uint32_t time;              /*!<Time stamp */
    uint32_t build_cksum;       /*!<Build checksum (micro-sha1) */
    /*
     * These fields go at the END of the struct so we can find it at the
     * end of memory.
     */
    uint32_t struct_size;       /*!<Size of the full dump structure */
    uint32_t magic;             /*!<PANIC_SAVE_MAGIC if valid */
};

/**
 * Structure for data panic in flash.
 */
struct panic_data_flash_header {
    uint32_t magic;             /*!<PANIC_SAVE_MAGIC if valid */
    uint32_t struct_size;       /*!<Size of this struct */

    uint32_t build_cksum;       /*!<Build checksum (micro-sha1) */
    uint32_t time;              /*!<Time stamp */
    uint8_t arch;               /*!<Architecture (PANIC_ARCH_*) */
    uint8_t struct_version;     /*!<Structure version */
    uint8_t flags;              /*!<Flags (PANIC_DATA_FLAG_*) */
    uint8_t reserved;           /*!<Reserved; set 0 */
};

#endif /* PANIC_H__ */
