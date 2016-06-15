/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_usb_device_driver.c $
 *
 * Synopsys HS OTG Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */

/** @file
 * The dwc_otg_driver module provides the initialization entry
 * points for the Quark_SE DWC_otg driver. This module statically instantiated
 * at boot. The dwc_otg_driver_init function is the entry point.
 *
 * The Peripheral Controller Driver (PCD) is responsible for
 * translating requests from the Function Driver into the appropriate
 * actions on the DWC_otg controller. It isolates the Function Driver
 * from the specifics of the controller by providing an API to the
 * Function Driver.
 *
 * The Peripheral Controller Driver for Quark_SE will implement the
 * DFU protocol.
 *
 */
#include "gpio/gpio.h"
#include "scss_registers.h"
#include "dwc_usb_device_driver.h"
#include "dwc_otg_core_if.h"
#include "dwc_otg_pcd_if.h"
#include "dwc_otg_dbg.h"
#include "dwc_otg_pcd.h"
#include "dwc_otg_dbg.h"
#include "usb.h"
#include "dwc_os.h"

#include "usb_driver_interface.h"

#include "usb_shared_interface.h"
#include "partition.h"

#define VENABLE_USB_REGULATOR	28

struct data_request {
	uint8_t *buf;
	uint32_t length;
	uint8_t zero;
};

struct out_request {
	usb_device_request_t out_req;
	struct data_request data;
};

const struct usb_interface intf_inst;

struct usb_static_data {
	struct usb_interface * intf;
	struct usb_os_dep os_dep_inst;

	dwc_otg_device_t *dwc_otg_device;
	dwc_otg_device_t dwc_otg_device_inst;
	dwc_otg_pcd_t static_pcd;

	uint8_t *ep0_buffer;
	uint32_t ep0_buffer_size;

	int usb_config;

	uint8_t *descriptor;
	uint8_t config_descriptor_size;

	usb_string_descriptor_t * strings_desc;
	uint32_t usb_strings_count;

	usb_device_descriptor_t *usb_device_descriptor;
	usb_config_descriptor_t *usb_config_descriptor;

	usb_endpoint_descriptor_t * usb_endpoints;
	int usb_num_endpoints;

	usb_event_cb_t usb_event_cb;
	usb_class_handler_t usb_class_handler;
	usb_ep_complete_t usb_ep_complete;

	struct out_request dl_req;
};

/* This structure has to be allocated in a RAM region that needs to be shared
 * between the bootloader code and the application code.
 */
struct usb_static_data static_data_inst __attribute__ ((section(".usb_shared_if")));

/* This define points to the location of the shared ram structure.
 * static_data_inst should be put at this BOOTLOADER_SHARED_DATA RAM location.
 * using a pointer would not work.
 * An alternative would be to use static_data_inst. instead of static_data->.
 */
#define static_data ((struct usb_static_data *) BOOTLOADER_SHARED_DATA)

/* Convenience define to access the otg_device from the static data struct */
#define dwc_otg_device (static_data->dwc_otg_device)

int dwc_usb_reset(void)
{
	return DWC_READ_REG32(dwc_otg_device->core_if->core_global_regs->gintmsk);
}
#ifdef USB_DEBUG
uint32_t g_dbg_lvl = 0; /* OFF */
#endif


/* Basic sanity check of the descriptor
 * Copy the provided descriptor locally*/
static int init_descriptors(usb_device_descriptor_t * dev_desc,
			    usb_config_descriptor_t * conf_desc, uint32_t csize,
			    usb_string_descriptor_t *strings, int num_strings)
{
	static_data->usb_device_descriptor = (usb_device_descriptor_t *) dev_desc;
	if (static_data->usb_device_descriptor->bNumConfigurations != 1) {
		__DWC_ERROR("Only one USB config is supported\n");
		return -DWC_E_INVALID;
	}

	static_data->usb_config_descriptor = (usb_config_descriptor_t *) conf_desc;

	static_data->config_descriptor_size = csize;
	static_data->strings_desc = strings;
	static_data->usb_strings_count = num_strings;
	return 0;
}

int usb_interface_init(struct usb_interface_init_data *init_data) {
	static_data->usb_ep_complete = init_data->ep_complete;
	init_descriptors(init_data->dev_desc,
			 init_data->conf_desc, init_data->conf_desc_size,
			 init_data->strings_desc, init_data->num_strings);
	static_data->usb_class_handler = init_data->class_handler;
	static_data->usb_endpoints = init_data->eps;
	static_data->usb_num_endpoints = init_data->num_eps;
	static_data->usb_event_cb = init_data->usb_evt_cb;
	static_data->ep0_buffer = init_data->ep0_buffer;
	static_data->ep0_buffer_size = init_data->ep0_buffer_size;
	return 0;
}

int usb_ep_read(int ep_address, uint8_t *buf, int len, void *priv)
{
	return dwc_otg_pcd_ep_queue(dwc_otg_device->pcd, ep_address, buf, buf,
			     len, 0, priv, 0);
}

int usb_ep_write(int ep_address, uint8_t *buf, int len, void *priv)
{
	return	dwc_otg_pcd_ep_queue(dwc_otg_device->pcd, ep_address, buf, buf,
			     len, 0, priv, 0);
}

int usb_get_config()
{
	return static_data->usb_config;
}

void usb_connect()
{
	gpio_cfg_data_t config;

	/* Setup and turn on USB PLL */
	MMIO_REG_VAL(USB_PLL_CFG0) = USB_PLL_CFG0_DEFAULT | USB_PLL_PDLD;

	/*Wait for the PLL lock */
	int count = 500;
	while(count-- && (0 == (MMIO_REG_VAL(USB_PLL_CFG0) & USB_PLL_LOCK))){}

	/* Turn on the clock gate */
	MMIO_REG_VAL(AHB_CTRL_REG) |= CCU_USB_CLK_EN;

	/* GPIO already configured by BOOTLOADER on OUTPUT MODE */
	/* Enable internal regulator */
	soc_gpio_write(SOC_GPIO_32, VENABLE_USB_REGULATOR, 1);
}

void usb_disconnect()
{
	struct usb_event evt;
	evt.event = USB_EVENT_DISCONNECT;
	if (static_data->usb_event_cb) {
		static_data->usb_event_cb(&evt);
	}
	if (dwc_otg_device->core_if->pcd_cb) {
		DWC_FREE(dwc_otg_device->core_if->pcd_cb);
		dwc_otg_device->core_if->pcd_cb = NULL;
	}
	if (dwc_otg_device->pcd) {
		dwc_otg_pcd_remove(dwc_otg_device->pcd);
		dwc_otg_device->pcd = NULL;
	}
	if (dwc_otg_device->core_if) {
		dwc_otg_cil_remove(dwc_otg_device->core_if);
		dwc_otg_device->core_if = NULL;
	}

	/* Disable clock */
	MMIO_REG_VAL(AHB_CTRL_REG) &= ~CCU_USB_CLK_EN;

	/* Stopping USB PLL */
	MMIO_REG_VAL(USB_PLL_CFG0) &= ~USB_PLL_PDLD;

	/* Disable regulator */
	soc_gpio_write(SOC_GPIO_32, VENABLE_USB_REGULATOR, 0);
}

static int _setup(dwc_otg_pcd_t * pcd, uint8_t * bytes)
{
	DWC_DEBUGPL(DBG_PCD, "%s(): enter \n", __func__);

	const usb_device_request_t *ctrlReq = (usb_device_request_t *) bytes;

	int value = 0;
	int idx;
	usb_device_request_t ctrlConvertedReq;
	usb_device_request_t *ctrl = &ctrlConvertedReq;


	ctrl->bmRequestType = (uByte) (ctrlReq->bmRequestType);
	ctrl->bRequest = (uByte) (ctrlReq->bRequest);
	ctrl->wValue[0] = ctrlReq->wValue[0];
	ctrl->wValue[1] = ctrlReq->wValue[1];
	ctrl->wIndex[0] = ctrlReq->wIndex[0];
	ctrl->wIndex[1] = ctrlReq->wIndex[1];
	ctrl->wLength[0] = ctrlReq->wLength[0];
	ctrl->wLength[1] = ctrlReq->wLength[1];
	/*
	 * usually this stores reply data in the pre-allocated ep0 buffer,
	 * but config change events will reconfigure hardware.
	 */
	static_data->dl_req.data.zero = 0;
	if (UT_GET_RECIPIENT(ctrl->bmRequestType) == UT_INTERFACE) {

		if ((UT_GET_DIR(ctrl->bmRequestType) == UT_WRITE)
		    && (UGETW(ctrl->wLength) > 0)) {
			static_data->dl_req.data.buf = &static_data->ep0_buffer[0];
			DWC_MEMCPY(&static_data->dl_req.out_req, ctrl, sizeof(*ctrl));
			static_data->dl_req.data.length =
			    DWC_MIN(UGETW(ctrl->wLength), static_data->ep0_buffer_size);

			// either shorter than asked, or multiple of the MPS require ZLP.
			static_data->dl_req.data.zero = ((value < UGETW(ctrl->wLength))
					     &&
					     ((value %
					       pcd->ep0.dwc_ep.maxpacket) ==
					      0)) ? 1 : 0;

			// Elaborate when getting the time.
			dwc_otg_pcd_ep_queue(pcd, /*void* ep_hanlde */ NULL,
					static_data->dl_req.data.buf,
					     (dwc_dma_t) static_data->dl_req.data.buf,
					     static_data->dl_req.data.length,
					     static_data->dl_req.data.zero,
					     /*void* req_handle */ &static_data->dl_req, 0);
		} else {
			static_data->dl_req.data.buf = &static_data->ep0_buffer[0];
			if (static_data->usb_class_handler(ctrl, &value, &static_data->dl_req.data.buf) != 0) {
				value = -DWC_E_NOT_SUPPORTED;
			}
		}

	} else {
		switch (ctrl->bRequest) {

		case UR_GET_DESCRIPTOR:
			__DWC_ERROR("ctrl->Length = %d \n",
				    UGETW(ctrl->wLength));

			switch (UGETW(ctrl->wValue) >> 8) {

			case UDESC_DEVICE:
				value =
				    DWC_MIN(UGETW(ctrl->wLength),
					    USB_DEVICE_DESCRIPTOR_SIZE);
				static_data->dl_req.data.buf = (uint8_t *)
						static_data->usb_device_descriptor;

				break;

			case UDESC_CONFIG:
				value = DWC_MIN(UGETW(ctrl->wLength),
						UGETW(static_data->usb_config_descriptor->wTotalLength));
				static_data->dl_req.data.buf = (uint8_t*)
					static_data->usb_config_descriptor;

				break;

			case UDESC_OTHER_SPEED_CONFIGURATION:
				__DWC_ERROR
				    ("UDESC_OTHER_SPEED_CONFIGURATION NOT IMPLEMENTED! \n");
				break;

			case UDESC_STRING:
				/* wIndex == language code.
				 * this driver only handles one language, you can
				 * add string tables for other languages, using
				 * any UTF-8 characters
				 */
				idx = UGETW(ctrl->wValue) & 0xFF;
				__DWC_ERROR("UDESC_STRING idx = %d \n", idx);
				__DWC_ERROR("UDESC_STRING bLength = %d \n",
						&static_data->strings_desc[idx].bLength);
				if (idx > static_data->usb_strings_count) {
					if (idx == 0xEE) {
						// Microsoft MTP extension
						value = 0;
					} else {
						value = -DWC_E_INVALID;
					}
				} else {
					value = DWC_MIN(UGETW(ctrl->wLength),
							static_data->strings_desc[idx].bLength);
					static_data->dl_req.data.buf = (uint8_t*)
						&static_data->strings_desc[idx];
				}

				break;
			}
			break;

		case UR_SET_CONFIG:
			static_data->usb_config = UGETW(ctrl->wValue);
			value = 0;

			{
				int i;
				for (i=0; i < static_data->usb_num_endpoints; i++) {
					int ret = dwc_otg_pcd_ep_enable(pcd,
					     (const uint8_t *)&static_data->usb_endpoints[i],
					     static_data->usb_endpoints[i].bEndpointAddress);
					__DWC_WARN("Enabled endpoint: %x ret: %d\n",
							static_data->usb_endpoints[i].bEndpointAddress,
							ret);
				}
				struct usb_event evt;
				evt.event = USB_EVENT_SET_CONFIG;
				evt.event_data.config = static_data->usb_config;
				if (static_data->usb_event_cb) {
					static_data->usb_event_cb(&evt);
				}
			}

			break;
		case UR_GET_CONFIG:
			static_data->dl_req.data.buf = &static_data->usb_config;
			value = DWC_MIN(UGETW(ctrl->wLength), 1);
			break;

			/* until we add altsetting support, or other interfaces,
			 * only 0/0 are possible.  pxa2xx only supports 0/0 (poorly)
			 * and already killed pending endpoint I/O.
			 */
		case UR_SET_INTERFACE:
			value = 0;
			break;

		case UR_GET_INTERFACE:
			__DWC_ERROR("UR_GET_INTERFACE NOT IMPLEMENTED! \n");
			goto unknown;
			break;

		default:
unknown:

			value = -DWC_E_NOT_SUPPORTED;
			break;
		}
	}
	/* respond with data transfer before status phase? */
	__DWC_ERROR("%s(): req.length = %d - max length is 64 \n", __func__,
		    value);
	if (value >= 0) {
		static_data->dl_req.data.length = value;

		// either shorter than asked, or multiple of the MPS require ZLP.
		static_data->dl_req.data.zero = ((value < UGETW(ctrl->wLength))
			    && ((value % pcd->ep0.dwc_ep.maxpacket) ==
				0)) ? 1 : 0;

		// Elaborate when getting the time.
		dwc_otg_pcd_ep_queue(pcd, /*void* ep_hanlde */ NULL,
				static_data->dl_req.data.buf,
				     (dwc_dma_t) static_data->dl_req.data.buf,
				     static_data->dl_req.data.length,
				     static_data->dl_req.data.zero,
				     /*void* req_handle */ NULL, 0);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static int _complete(dwc_otg_pcd_t * pcd, void *ep_handle,
		     void *req_handle, int32_t status, uint32_t actual)
{

	struct out_request *oreq = (struct out_request *)req_handle;

	if (ep_handle != NULL) {
		static_data->usb_ep_complete((int)ep_handle, req_handle, status, actual);
		return 0;
	}
	__DWC_ERROR("%s(): enter \n", __func__);
	if (oreq == NULL) {
		return 0;
	}
	if (status != 0) {
		__DWC_ERROR("%s(): status not null: %d \n", __func__, status);
		return 0;
	}

	if (static_data->usb_class_handler(&oreq->out_req, &oreq->data.length,
			      &oreq->data.buf) != 0) {
		__DWC_ERROR("%s(): Error while processing packet\n", __func__);
	}

	return 0;
}

static int _connect(dwc_otg_pcd_t * pcd, int speed)
{
	DWC_DEBUGPL(DBG_PCD, "%s(): enter \n", __func__);

	return 0;
}

static int _disconnect(dwc_otg_pcd_t * pcd)
{
	DWC_DEBUGPL(DBG_PCD, "%s(): enter \n", __func__);

	return 0;
}

static int _resume(dwc_otg_pcd_t * pcd)
{
	DWC_DEBUGPL(DBG_PCD, "%s(): enter \n", __func__);

	return 0;
}

static int _suspend(dwc_otg_pcd_t * pcd)
{
	DWC_DEBUGPL(DBG_PCD, "%s(): enter \n", __func__);

	return 0;
}

/*
 * This function updates the otg values in the gadget structure.
 */
static int _hnp_changed(dwc_otg_pcd_t * pcd)
{
	DWC_DEBUGPL(DBG_PCD, "%s(): enter \n", __func__);

	return 0;
}

static int _reset(dwc_otg_pcd_t * pcd)
{
	DWC_DEBUGPL(DBG_PCD, "%s(): enter \n", __func__);

	struct usb_event evt;
	evt.event = USB_EVENT_RESET;
	if (static_data->usb_event_cb) {
		static_data->usb_event_cb(&evt);
	}

	return 0;
}

#ifdef DWC_UTE_CFI
static int _cfi_setup(dwc_otg_pcd_t * pcd, void *cfi_req)
{
	DWC_DEBUGPL(DBG_PCD, "%s(): enter \n", __func__);

	return 0;
}
#endif

/*
 * This function is the top level interrupt handler for the Common
 * (Device and host modes) interrupts.
 */

/*
 * This function must be registered as a interrupt handler after calling the
 * driver_init() function.
 */
void usb_ISR(void)
{
	/*
	 * FIXME: we shall be able to detect USB reset here with resetdet field
	 * or is it host only
	 */
	gintsts_data_t gintsts;
	__DWC_WARN("USB ISR! - enter\n");
	do {
		dwc_otg_handle_common_intr(dwc_otg_device);
		dwc_otg_pcd_handle_intr(dwc_otg_device->pcd);
		gintsts.d32 = dwc_otg_read_core_intr(dwc_otg_device->core_if);
		__DWC_WARN("USB ISR! - gintsts: %d\n", gintsts.d32);
	} while (gintsts.d32 != 0);
	__DWC_WARN("USB ISR! - exit\n");
}

const struct dwc_otg_pcd_function_ops fops = {
	.complete = _complete,
	.setup = _setup,
	.disconnect = _disconnect,
	.connect = _connect,
	.resume = NULL,		/* _resume, */
	.suspend = NULL,	/* _suspend, */
	.hnp_changed = NULL,	/* _hnp_changed, */
	.reset = _reset,
#ifdef DWC_UTE_CFI
	.cfi_setup = _cfi_setup,
#endif
};

 /* Allocates and init the pcd */
static int pcd_init(dwc_otg_device_t * otg_dev)
{
	int retval = 0;

	otg_dev->pcd = dwc_otg_pcd_init(otg_dev->core_if, &static_data->static_pcd);
	if (!otg_dev->pcd) {
		__DWC_ERROR("dwc_otg_pcd_init failed\n");
		return -DWC_E_NO_MEMORY;
	}

	dwc_otg_pcd_start(otg_dev->pcd, &fops);

	return retval;

}

/*
 * This function is called when an lm_device is bound to a
 * dwc_otg_driver. It creates the driver components required to
 * control the device (CIL, HCD, and PCD) and it initializes the
 * device. The driver components are stored in a dwc_otg_device
 * structure. A reference to the dwc_otg_device is saved in the
 * lm_device. This allows the driver to access the dwc_otg_device
 * structure on subsequent calls to driver methods for this device.
 *
 */

int usb_driver_init(uint32_t base_addr)
{
	int retval = 0;

	__DWC_WARN("dwc_otg_driver_init\n");

	dwc_otg_device = &static_data->dwc_otg_device_inst;

	DWC_MEMSET(dwc_otg_device, 0, sizeof(*dwc_otg_device));

	/*
	 * Initialize driver data to point to the global DWC_otg
	 * Device structure.
	 */
	__DWC_ERROR("dwc_otg_device addr = 0x%p\n", dwc_otg_device);

	dwc_otg_device->core_if = dwc_otg_cil_init(base_addr);
	if (!dwc_otg_device->core_if) {
		__DWC_ERROR("CIL initialization failed!\n");
		retval = -DWC_E_NO_MEMORY;
		goto fail;
	}

#ifdef USB_DEBUG
	/*
	 * Attempt to ensure this device is really a DWC_otg Controller.
	 * Read and verify the SNPSID register contents. The value should be
	 * 0x45F42XXX or 0x45F42XXX, which corresponds to either "OT2" or "OTG3",
	 * as in "OTG version 2.XX" or "OTG version 3.XX".
	 */

	if (((dwc_otg_get_gsnpsid(dwc_otg_device->core_if) & 0xFFFFF000) !=
	     0x4F542000)
	    && ((dwc_otg_get_gsnpsid(dwc_otg_device->core_if) & 0xFFFFF000) !=
		0x4F543000)) {
		__DWC_WARN("Bad value for SNPSID: 0x%x\n",
			   dwc_otg_get_gsnpsid(dwc_otg_device->core_if));
		retval = -DWC_E_INVALID;
		goto fail;
	}
#endif /* USB_DEBUG */

	/*
	 * Disable the global interrupt until all the interrupt
	 * handlers are installed.
	 */
	__DWC_WARN("USB INIT: Disabling global IRQ\n");
	dwc_otg_disable_global_interrupts(dwc_otg_device->core_if);

	/*
	 * Initialize the DWC_otg core.
	 */
	__DWC_WARN("USB INIT: Initializing OTG core\n");
	dwc_otg_core_init(dwc_otg_device->core_if);

	/*
	 * Initialize the PCD
	 */
	__DWC_WARN("USB INIT: Initializing PCD\n");
	retval = pcd_init(dwc_otg_device);
	if (retval != 0) {
		__DWC_ERROR("pcd_init failed\n");
		dwc_otg_device->pcd = NULL;
		goto fail;
	}

	__DWC_WARN("USB INIT: Enabling global IRQ\n");
	dwc_otg_enable_global_interrupts(dwc_otg_device->core_if);

	__DWC_WARN("DWC core initialized! \n");
	return 0;

fail:
	__DWC_ERROR("ERROR! unable to init USB driver\n");
	return retval;
}

static int usb_driver_version()
{
	return USB_DRIVER_INTERFACE_VERSION;
}

void usb_shared_interface_init()
{
	static_data->intf = &intf_inst;
}

int usb_ep_disable(int ep_address)
{
	return dwc_otg_pcd_ep_disable(dwc_otg_device->pcd, (void*)ep_address);
}

const struct usb_interface intf_inst = {
		.usb_interface_init = usb_interface_init,
		.usb_ep_read = usb_ep_read,
		.usb_ep_write = usb_ep_write,
		.usb_ep_disable = usb_ep_disable,
		.usb_get_config = usb_get_config,
		.usb_isr = usb_ISR,
		.usb_driver_init = usb_driver_init,
		.usb_connect = usb_connect,
		.usb_disconnect = usb_disconnect,
		.version = usb_driver_version,
};
