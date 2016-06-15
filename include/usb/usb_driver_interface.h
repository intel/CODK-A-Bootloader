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

#ifndef __USB_DRIVER_INTERFACE_H__
#define __USB_DRIVER_INTERFACE_H__

/**
 * @defgroup usb_driver USB driver
 * USB driver APIs.
 * @ingroup common_drivers
 * @{
 * @defgroup usb_driver_interface USB function driver API
 * USB function driver API.
 * @ingroup usb_driver
 * This file defines the USB function driver interface that is used to
 * implement a function driver on top of the USB hardware driver.
 * @{
 */

enum event_type {
	/** Usb reset detected event */
	USB_EVENT_RESET,
	/** Usb set config received event
	 *
	 * When USB set config is received, the enpoints are enabled,
	 * and the USB_EVENT_SET_CONFIG is sent with the selected config.
	 */
	USB_EVENT_SET_CONFIG,
	/** Usb disconnect event */
	USB_EVENT_DISCONNECT,
};

/**
 * Usb event structure.
 *
 * This structure is used to report an usb driver event to the application.
 */
struct usb_event {
	/** Event type @ref event_type*/
	enum event_type event;
	/** Event data */
	union {
		/** Selected configuration for @ref USB_EVENT_SET_CONFIG */
		int config;
	} event_data;
};

/**
 * Usb event callback.
 *
 * This function is called whenever an event happens. It is used by the driver
 * to transfer an event of type usb_event to the function driver.
 * This function is called in the context of the usb interrupt. It should not
 * do heavy processing.
 *
 * @param evt the event to be processed by the function driver. This pointer is
 * only valid during the execution of the callback and must not be used outside
 * the scope of this function.
 */
typedef void (*usb_event_cb_t)(struct usb_event *evt);

/**
 * Endpoint transfer completion callback
 *
 * This type defines an enpoint transfer completion callback.
 * It is called whenever the usb transfer is completed.
 * This function is called in the context of the usb interrupt. It should not
 * do heavy processing.
 *
 * @param ep_address the enpoint address on which the transfer is triggered.
 * @param priv the private data as passed by usb_ep_read() or usb_ep_write()
 * @param status the status of the transfer
 * @param actual the actual length transfered
 */
typedef void (*usb_ep_complete_t)(int ep_address, void *priv, int status, int actual);

/**
 * Usb ep0 class handler.
 *
 * This function is called whenever the host calls class specific handlers.
 * This function is called in the context of the usb interrupt. It should not
 * do heavy processing.
 *
 * @param request the usb control request. eventually followed by the OUT data.
 * @param len the length of the response of the request.
 * @param data the pointer to the pointer of the buffer that holds response data
 * @return 0 if successfull, negative in case of error / unhandled request.
 */
typedef int (*usb_class_handler_t)(usb_device_request_t *request, uint32_t *len, uint8_t **data);

/**
 * Usb Interface initialization data
 *
 * This structure contains all the initialization data to be passed to the
 * driver through the call to @ref usb_interface_init.
 *
 */

struct usb_interface_init_data
{
	/**The enpoint completion callback function. */
	usb_ep_complete_t ep_complete;
	/** The class handler for ep0 class requests */
	usb_class_handler_t class_handler;
	/** The usb event handler callback see struct usb_event
	 *  this function can be called in the context of the usb
	 *  interrupt, and should not do heavy processing. */
	usb_event_cb_t usb_evt_cb;
	/** The buffer used for transactions on endpoint 0. */
	uint8_t *ep0_buffer;
	/** The size of ep0_buffer. */
	uint32_t ep0_buffer_size;
	/** The usb device descriptor */
	usb_device_descriptor_t *dev_desc;
	/** The usb configuration descriptor */
	usb_config_descriptor_t *conf_desc;
	/** The size of the configuration descriptor in bytes */
	uint32_t conf_desc_size;
	/** The table of strings descriptors */
	usb_string_descriptor_t *strings_desc;
	/** The number of strings descriptors */
	int num_strings;
	/** Array of endpoint descriptors */
	usb_endpoint_descriptor_t *eps;
	/** Number of endpoint descriptors */
	int num_eps;
};

/**
 * Initialize the usb driver with the given driver interface.
 *
 * This function will set the descriptors that will be used for
 * the usb enumeration, and set the class specific ep0 request handler.
 *
 * @param init_data The initialization data.
 */
int usb_interface_init(struct usb_interface_init_data * init_data);

/**
 * queue a read buffer
 *
 * This function will queue a read request on the specified OUT enpoint.
 * when the buffer will be filled, the ep_complete() callback set in
 * usb_interface_init() will be called.
 *
 * @param ep_address the OUT enpoint to read from
 * @param buf the buffer to write data on
 * @param len the length of the requested transfer
 * @param priv a private pointer passed in the ep_complete() callback
 *
 * @return 0 if no error occured.
 */
int usb_ep_read(int ep_address, uint8_t *buf, int len, void *priv);

/**
 * queue a write buffer
 *
 * This function will queue a write request on the specified IN enpoint.
 * when the buffer will be completely transfered to the host, the ep_complete()
 * callback set in usb_interface_init() will be called.
 *
 * @param ep_address the IN enpoint to write to
 * @param buf the buffer to send data from
 * @param len the length of the requested transfer
 * @param priv a private pointer passed in the ep_complete() callback
 *
 * @return 0 if no error occured.
 */
int usb_ep_write(int ep_address, uint8_t *buf, int len, void *priv);

/**
 * disable an endpoint
 *
 * This function should be called by the function driver whenever there is a
 * transfer pending and the cable is unplugged.
 *
 * @param ep_address the endpoint address of the endpoint to disable.
 *
 * @return 0 if no error occured.
 */
int usb_ep_disable(int ep_address);

/**
 * get currently selected configuration.
 *
 * This function returns the currently selected configuration.
 * It allows the application to know if the usb is enumerated and ready to
 * handle endpoint transfer requests.
 *
 * @return the currently selected configuration. -1 if usb is not enumerated
 *         yet.
 */
int usb_get_config();
/**
 * @}
 * @}
 */

#endif /* __USB_DRIVER_INTERFACE_H__ */

