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

#include <dfu.h>
#include <usb/usb_driver_interface.h>
#include <printk.h>
#include <bootlogic.h>
#include <utils.h>
#include <infra/factory_data.h>

#ifndef NULL
#define NULL 0
#endif

/*Class specific request*/
#define DFU_DETACH          0x00
#define DFU_DNLOAD          0x01
#define DFU_UPLOAD          0x02
#define DFU_GETSTATUS       0x03
#define DFU_CLRSTATUS       0x04
#define DFU_GETSTATE        0x05
#define DFU_ABORT           0x06

/* ToDo - move this defines in a common file */
#define FACTORY_DATA_ADDR 0xffffe000
const struct customer_data* otp_data_ptr = (struct customer_data*)(FACTORY_DATA_ADDR + 0x200);

static struct dfu_ops dfu_ops = {
	.alternate = -1,
	.state = dfuIDLE,
	.status = statusOK,
	.data = NULL,
	.len = 0,
	.device_request = NULL,
	.erase = NULL,
	.write = NULL,
	.read = NULL
};

/*
 * ToDo: The Makefile is not able to include string.h for memcpy()
 * and memset() functional calls. We received orders to implement
 * these functions manually and stop the Makefile changes. We need
 * to remove this functions and make the changes in the right way
 */
void * _memset(void *b, uint8_t c, int len)
{
  if (b == NULL) return NULL;

  unsigned char *ptr = (unsigned char *)b;
  int i = 0;
  while(len > 0)
  {
      *ptr = c;
      ptr++;
      len--;
  }

  return(b);
}

int _memcmp(void * ptr_dest, void * ptr_src, int size)
{

    // checking memory pointers
    if ((ptr_dest == NULL) || (ptr_src == NULL)) return -1;

    // byte by byte copy
    uint8_t *pdest = (uint8_t*) ptr_dest;
    uint8_t *psrc = (uint8_t*) ptr_src;
    int index = 0;

    for(index = 0; index < size; ++index)
    {

      uint8_t d = *((uint8_t*)pdest);
      uint8_t s = *((uint8_t*)psrc);

      if (s < d)
      {
        return -1;
      }
      else if (s > d)
      {
        return 1;
      }

      pdest += sizeof(uint8_t);
      psrc += sizeof(uint8_t);

    }

  return 0;

}


bool convert2Unicode(const char * ascii_ptr,  uint32_t ascii_len, usb_string_descriptor_t *usb_desc_ptr)
{
    unsigned int counter = 0;

    if ((ascii_ptr == NULL) || (usb_desc_ptr == NULL))
    {
       return 0;
    }

    // the current data struct does not have this problem but
    // keeping this checking only to avoid problems in the future changes
    if (ascii_len > USB_MAX_STRING_LEN)
    {
       ascii_len = USB_MAX_STRING_LEN - 1;
    }

    // clear the buffer
    _memset((void*) usb_desc_ptr->bString, 0, sizeof(usb_desc_ptr->bString));

    for(counter = 0; counter < ascii_len + 1; counter++)
    {
         // no special characters in the descriptor, so simple conversion
         usb_desc_ptr->bString[counter][0] = *ascii_ptr++;
    }

    // the number of characters converted it is possible to update
    // the bLenght
    usb_desc_ptr->bLength = ascii_len * 2 + 2;

    return 1;
}


void do_dfu_download(usb_device_request_t *setup_packet, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	if (0 != UGETW(setup_packet->wLength)) {
		/* download has started */
		dfu_ops.state = dfuDNBUSY;
		*len = UGETW(setup_packet->wLength);
		dfu_ops.len = *len;
		dfu_set_alternate(&dfu_ops);

		if (dfu_ops.write)
			dfu_ops.write(&dfu_ops);
	} else {
		/* Download complete */
		dfu_ops.state = dfuIDLE;
		dfu_ops.alternate = -1;
	}

	*len = 0;
}

void do_dfu_upload(usb_device_request_t *setup_packet, uint32_t *len)
{
	pr_debug("%s\n", __func__);

	if (0 != UGETW(setup_packet->wLength)) {
		/* upload has started */
		dfu_ops.state = dfuUPLOAD_IDLE;
		*len = UGETW(setup_packet->wLength);
		dfu_ops.len = *len;
		dfu_set_alternate(&dfu_ops);

		if (dfu_ops.read) {
			dfu_ops.read(&dfu_ops);
			/* Send data */
		}
	} else {
		/* Upload complete */
		dfu_ops.state = dfuIDLE;
		dfu_ops.alternate = -1;
	}

	*len = 0;
}

void do_dfu_abort(usb_device_request_t *setup_packet, uint32_t *len)
{
	pr_debug("%s\n", __func__);
	dfu_ops.state = dfuIDLE;
}

void do_dfu_detach(usb_device_request_t *setup_packet, uint32_t *len)
{
	pr_debug("%s\n", __func__);
	dfu_ops.state = dfuIDLE;
	*len = 0;
	/*
	 * call dfu_reset to catch USB reset
	 * if do_dfu_reset returns, we boot
	 */
	do_dfu_reset();
}

volatile int dfu_reset = 0;
void do_dfu_reset(void)
{
	pr_debug("%s\n", __func__);
	dfu_reset = 1;
}

extern uint32_t dfu_busy;
int ClassHandleReq(usb_device_request_t *setup_packet, uint32_t *len,
		   uint8_t **data)
{
	unsigned char *dfu_buffer = *data;
	dfu_ops.data = *data;
	dfu_ops.len = *len;
	dfu_ops.device_request = setup_packet;

	switch (setup_packet->bRequest) {
	case UR_SET_INTERFACE:
		dfu_ops.alternate = UGETW(setup_packet->wValue);
		dfu_buffer[0] = dfu_ops.alternate;
		*len = 0;
		break;
	case DFU_GETSTATUS:
		pr_debug("DFU_GETSTATUS: %x\r\n", dfu_ops.state);
		dfu_buffer[0] = dfu_ops.status;
		dfu_buffer[1] = 1;
		dfu_buffer[2] = 0;
		dfu_buffer[3] = 0;
		dfu_buffer[4] = dfu_ops.state;	/* state: idle */
		dfu_buffer[5] = 0;	/* status string */
		*len = 6;

		if (dfu_ops.state == dfuDNBUSY)
			dfu_ops.state = dfuDNLOAD_IDLE;
		break;

	case DFU_GETSTATE:
		pr_debug("DFU_GETSTATE\r\n");
		(*data)[0] = dfu_ops.state;
		*len = 1;
		break;

	case DFU_ABORT:
		do_dfu_abort(setup_packet, len);
		break;

	case DFU_CLRSTATUS:
		pr_debug("DFU_CLRSTATUS\r\n");
		dfu_ops.state = dfuIDLE;
		dfu_ops.status = statusOK;
		break;

	case DFU_DNLOAD:
		/* a dfu transfer has started */
		dfu_busy = 1;
		do_dfu_download(setup_packet, len);
		break;

	case DFU_UPLOAD:
		/* a dfu transfer has started */
		dfu_busy = 1;
		do_dfu_upload(setup_packet, len);
		*len = dfu_ops.len;
		break;

	case DFU_DETACH:
		do_dfu_detach(setup_packet, len);
			break;
	default:
		pr_debug("UNKNOWN: %d\r\n", setup_packet->bRequest);
		return -1;
	}

	return 0;
}

#define EP0_BUFFER_SIZE 2048
uint8_t ep0_buffer[EP0_BUFFER_SIZE];

struct usb_interface_init_data dfu_init_data = {
	.ep_complete = NULL,
	.class_handler = ClassHandleReq,
	.usb_evt_cb = NULL,
	.ep0_buffer = &ep0_buffer[0],
	.ep0_buffer_size = EP0_BUFFER_SIZE,
	.dev_desc = &dfu_device_desc,
	.conf_desc = (usb_config_descriptor_t*)&dfu_config_desc,
	.conf_desc_size = sizeof(dfu_config_desc),
	.strings_desc = dfu_strings_desc,
	.num_strings = sizeof(dfu_strings_desc) /
		sizeof(usb_string_descriptor_t),
	.eps = NULL,
	.num_eps = 0,
};

void dfu_class_init()
{

        // need to check if there is something in the OTP area
        if ((otp_data_ptr->patternKeyStart == PATTERN_KEY_START) &&
            (otp_data_ptr->patternKeyEnd == PATTERN_KEY_END))
        {
          // PVT board with OTP are programmed
          // let's convert the vendor and the board name to unicode
          // and update the strings array with right data and lenght
          convert2Unicode((const char *)otp_data_ptr->vendor_name, otp_data_ptr->vendor_name_len, &dfu_init_data.strings_desc[1]);
          convert2Unicode((const char *)otp_data_ptr->board_name , otp_data_ptr->board_name_len, &dfu_init_data.strings_desc[2]);
          convert2Unicode((const char *)otp_data_ptr->product_sn , otp_data_ptr->product_sn_len, &dfu_init_data.strings_desc[3]);

	}
        else if ((_memcmp((const void *)otp_data_ptr->product_sn, INVALID_SN_F, sizeof(INVALID_SN_F)) != 0) &&
                 (_memcmp((const void *)otp_data_ptr->product_sn, INVALID_SN_0, sizeof(INVALID_SN_0)) != 0))
	{
          // there is something written as product serial number, let's update the usb descriptor

          // checking if there is a valid product serial number
	  // there is not "magic" number in the customer data structure on DVT boards in order
          // to identify if the OTP customer area was written or not. The best we can do is check with
          // 0xFF or 0x00 sequences (blank flash)
          convert2Unicode((const char *)otp_data_ptr->product_sn , 15, &dfu_init_data.strings_desc[3]);

          // in the DFU the default board name is "QRK_SE-Dev1.0". In order to make consistent with acm-cdc application
          // let's change to 'Arduino101'
          unsigned char atl_name[11]= {'A','t','l','a','s',' ','E','d','g','e', 0};
          convert2Unicode((const char *) atl_name, 11, &dfu_init_data.strings_desc[2]);

	}

	usb_interface_init(&dfu_init_data);
}
