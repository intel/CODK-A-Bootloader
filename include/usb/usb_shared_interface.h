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

#ifndef __USB_DYL_H__
#define __USB_DYL_H__
#include <stdint.h>
#include "usb.h"
#include "usb_driver_interface.h"


/**
 * @defgroup usb_shared_interface Shared USB interface definition
 * Shared USB interface definition.
 *
 *	This file defines the interface that is used to share the USB driver
 *	between the bootloader and the application
 * @ingroup usb_driver
 * @{
 */

/** USB interface version
 *
 * Used to check that the bootloader interface version and the expected app
 * interface version matches.
 */
#define USB_DRIVER_INTERFACE_VERSION 1

/**
 * USB driver os dependency
 *
 * This structure is used by bootloader and application to set the function
 * pointers to the os dependencies for the USB driver.
 */
struct usb_os_dep {
	/**
	 * Allocate a block of memory
	 *
	 * @param size size of the block to allocate
	 * @return the pointer to allocated block.
	 */
	void * (*alloc)(int size);
	/**
	 * Free a block of memory
	 *
	 * @param ptr the pointer to free
	 */
	void (*free)(void *ptr);
	/**
	 * Display a log message
	 *
	 * @param level the log level
	 * @param fmt the format string
	 * @param ... the parameters for the format string
	 */
	int (*printk)(int level, const char *, ...);
};

/**
 * Macro used to access the struct usb_os_dep shared structure
 */
#define usb_driver_os_dep ((struct usb_os_dep*) (BOOTLOADER_SHARED_DATA + 4))

/**
 * USB Driver interface
 *
 * This structure is set by the bootloader and used by the application to allow
 * accessing the usb driver inteface from the bootloader. It has a fixed memory
 * address.
 * non documented fields are just wrappers to the functions defined in
 * @ref usb_driver_interface
 */
struct usb_interface {
	int (*usb_interface_init)(struct usb_interface_init_data * init_data);
	int (*usb_ep_read)(int ep_address, uint8_t *buf, int len, void *priv);
	int (*usb_ep_write)(int ep_address, uint8_t *buf, int len, void *priv);
	int (*usb_ep_disable)(int ep_address);
	int (*usb_get_config)(void);
	void (*usb_isr)(void);
	int (*usb_driver_init)(uint32_t base_addr);
	/**
	 * inform usb driver that the connector was unplugged.
	 */
	void (*usb_disconnect)(void);
	/**
	 * inform usb driver that the connector was plugged.
	 */
	void (*usb_connect)(void);
	/**
	 * returns the version of the usb driver interface for runtime checking
	 * compatibility between bootloader and application.
	 */
	int (*version)(void);
};

/**
 * Macro used to access the struct usb_inteface shared structure
 */
#define usb_driver_intf (*(struct usb_interface **) BOOTLOADER_SHARED_DATA)

/**
 * @}
 */
#endif
