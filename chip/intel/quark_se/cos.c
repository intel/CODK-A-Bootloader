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
#include "machine/soc/quark_se/scss_registers.h"
#include "machine/soc/quark_se/cos_interface.h"
#include <printk.h>

/*
 * Charging OS binary blob and length are defined in a generated file.
 */
extern unsigned int cos_bin_len;
extern unsigned char cos_bin[];

void cos_start()
{
	int i;

	volatile unsigned char * dst = COS_STARTUP;
	volatile unsigned char * src = cos_bin;

	/* Init mailbox interrupt */
	MMIO_REG_VAL(IO_REG_MAILBOX_INT_MASK) = 0xffffffff;

	pr_info("Copy %d bytes from %p to %p \r\n",
			cos_bin_len, src, dst);

	for (i=0;i<cos_bin_len; i++) {
		*dst++ = *src++;
	}

	MBX_STS(4) = 0x3;

	*(uint32_t*) 0xa8000000 = COS_STARTUP;

	COS_ARC_REQ = 0;
	COS_ARC_READY = 0;

	SCSS_REG_VAL(SCSS_SS_CFG) = ARC_RUN_REQ_A;

	while (!COS_ARC_READY) ;
	pr_info("ARC ready\n\r");

}

/*
 * Send a request to the arc core.
 *
 * \param request the request id to send
 * \param data the data parameter to set for the command
 *             (optional based on request)
 *
 * \return the request response
 */
static int cos_send_request(int request, int data)
{
	int count;
	COS_ARC_ACK = 0;
	COS_ARC_DATA = 4;
	COS_ARC_REQ = request;

	count = 1000000;
	while(!COS_ARC_ACK && count--) ;
	if (count <= 0) {
		pr_info("Timeout waiting for ack %d\r\n", request);
	}
	int ret = COS_ARC_DATA;
	return ret;
}

int cos_adc_request(int channel)
{
	return cos_send_request(COS_CMD_READ_ADC, channel);
}


