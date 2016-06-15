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

#ifndef __PRINTK_H__
#define __PRINTK_H__

#include <stdarg.h>
#define DEBUG
#if defined(DEBUG)
#define PRINTK_BUFSIZ 256

enum {
	LEVEL_NONE = 0,
	LEVEL_BINARY,
	LEVEL_ERROR,
	LEVEL_WARNING,
	LEVEL_INFO,
	LEVEL_DEBUG,
	LEVEL_PROFILE,
	LEVEL_NUM /* gives the number of log levels */
};

extern int log_level;

#define pr_err(...)					\
	do {						\
		printk(LEVEL_ERROR, __VA_ARGS__);		\
	} while (0)

#define pr_warning(...)					\
	do {						\
		printk(LEVEL_WARNING, __VA_ARGS__);	\
	} while (0)

#define pr_warn pr_warning

#define pr_info(...)					\
	do {						\
		printk(LEVEL_INFO, __VA_ARGS__);		\
	} while (0)

#define pr_debug(...)					\
	do {						\
		printk(LEVEL_DEBUG, __VA_ARGS__);		\
	} while (0)

#define pr_profile(...)					\
	do {						\
		printk(LEVEL_PROFILE,  __VA_ARGS__);	\
	} while (0)

int printk(int level, const char *fmt, ...);


#else

#define LOG_LEVEL_INIT

#define pr_err(...)					\
	do {						\
	} while (0)

#define pr_warning(...)					\
	do {						\
	} while (0)

#define pr_warn pr_warning

#define pr_info(...)					\
	do {						\
	} while (0)

#define pr_debug(...)					\
	do {						\
	} while (0)

#define pr_profile(...)					\
	do {						\
	} while (0)

#endif /* DEBUG */

#endif /* __PRINTK_H__*/
