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

/* #include <string.h> */
/* #include <stdlib.h> */
/* #include <stdio.h> */
#include <stdarg.h>
#include "printk.h"
#include "mem.h"
#include "irq.h"
#include "partition.h"
#include "usb/usb_shared_interface.h"

/* OS-Level Implementations */

/* This is the Linux kernel implementation of the DWC platform library. */
#include "dwc_os.h"

/* MISC */

void *DWC_MEMSET(void *dest, uint8_t byte, uint32_t size)
{
/* 	return memset(dest, byte, size); */
	uint32_t i;
	for(i = 0; i<size; i++) {
		((uint8_t*)dest)[i] = byte;
	}
	return dest;
}

void *DWC_MEMCPY(void *dest, void const *src, uint32_t size)
{
/* 	return memcpy(dest, src, size); */
	uint32_t i;
	for(i = 0; i<size; i++) {
		((uint8_t*)dest)[i] = ((uint8_t*)src)[i];
	}
	return dest;
}

char *DWC_STRCPY(char *to, char const *from)
{
	uint32_t i;
	i = 0;
	do {
		to[i] = from[i];
	} while(from[i++] != '\0');
/* 	return strcpy(to, from); */
	return to;
}

/* dwc_debug.h */

dwc_bool_t DWC_IN_IRQ(void)
{
	/* TODO: REMOVE THIS FUNCTION */
	return 1;
}

dwc_bool_t DWC_IN_BH(void)
{
	/* TODO: REMOVE THIS FUNCTION */
	return 1;
}

/* USED in cil.c l4538 
 * inside USB_DEBUG */
int DWC_SPRINTF(char *buffer, char *format, ...)
{
/*	int retval;
	va_list args;

	va_start(args, format);
	retval = vsprintf(buffer, format, args);
	va_end(args);
	return retval;*/
	return 0;
}

void *__DWC_DMA_ALLOC(void *dma_ctx, uint32_t size, dwc_dma_t * dma_addr)
{

	void *buf = DWC_ALLOC(size);
	if (!buf) {
		return NULL;
	}
	*dma_addr = (dwc_dma_t) buf;
	DWC_MEMSET(buf, 0, size);
	return buf;

}

void *__DWC_DMA_ALLOC_ATOMIC(void *dma_ctx, uint32_t size, dwc_dma_t * dma_addr)
{
	return __DWC_DMA_ALLOC(dma_ctx, size, dma_addr);
}

void __DWC_DMA_FREE(void *dma_ctx, uint32_t size, void *virt_addr,
		    dwc_dma_t dma_addr)
{
	DWC_FREE(virt_addr);
}

void *__DWC_ALLOC(void *mem_ctx, uint32_t size)
{
	return usb_driver_os_dep->alloc(size);
}

void *__DWC_ALLOC_ATOMIC(void *mem_ctx, uint32_t size)
{
	/* Save IRQ*/
	return usb_driver_os_dep->alloc(size);
}

void __DWC_FREE(void *mem_ctx, void *addr)
{
	usb_driver_os_dep->free(addr);
}

/* Registers */

uint32_t DWC_READ_REG32(uint32_t volatile *reg)
{
	return *reg;
}

void DWC_WRITE_REG32(uint32_t volatile *reg, uint32_t value)
{
	*reg = value;
}

void DWC_MODIFY_REG32(uint32_t volatile *reg, uint32_t clear_mask,
		      uint32_t set_mask)
{
	uint32_t tmp;
	tmp = (*reg & ~clear_mask) | set_mask;
	*reg = tmp;
}

/* Locking */
/* TODO: This Spinlock implementation is only suitable for mono-thread, no
 * preemption system */

dwc_spinlock_t *DWC_SPINLOCK_ALLOC(void)
{
	dwc_spinlock_t *sl = (dwc_spinlock_t *) 1;
	return sl;
}

void DWC_SPINLOCK_FREE(dwc_spinlock_t * lock)
{
}

void DWC_SPINLOCK(dwc_spinlock_t * lock)
{
}

void DWC_SPINUNLOCK(dwc_spinlock_t * lock)
{
}

void DWC_SPINLOCK_IRQSAVE(dwc_spinlock_t * lock, dwc_irqflags_t * flags)
{
	dwc_irqflags_t f;
	f = interrupt_lock();
	*flags = f;
}

void DWC_SPINUNLOCK_IRQRESTORE(dwc_spinlock_t * lock, dwc_irqflags_t flags)
{
	interrupt_unlock(flags);
}

/* TODO: MUTEXES ARE NOT USED*/
dwc_mutex_t *DWC_MUTEX_ALLOC(void)
{
	return (dwc_mutex_t *) NULL;
}

void DWC_MUTEX_FREE(dwc_mutex_t * mutex)
{
}

void DWC_MUTEX_LOCK(dwc_mutex_t * mutex)
{
}

int DWC_MUTEX_TRYLOCK(dwc_mutex_t * mutex)
{
	return 0;
}

void DWC_MUTEX_UNLOCK(dwc_mutex_t * mutex)
{
}

/* Timing */
/* TODO: Implement these functions */

void DWC_UDELAY(uint32_t usecs)
{
}

void DWC_MDELAY(uint32_t msecs)
{
}

/* Timers */

struct dwc_timer {
/* 	T_TIMER t; */
	dwc_timer_callback_t cb;
	void *data;
	uint8_t scheduled;
	dwc_spinlock_t *lock;
};

/*static void timer_callback(void *data)
{
	dwc_timer_t *timer = (dwc_timer_t *) data;
	dwc_irqflags_t flags;

	DWC_SPINLOCK_IRQSAVE(timer->lock, &flags);
	timer->scheduled = 0;
	DWC_SPINUNLOCK_IRQRESTORE(timer->lock, flags);
	timer->cb(timer->data);
}
*/
/* dwc_timer_callback_t is void*
 * T_ENTRY_POINT is void*
 * T_TIMER is void*
 * */

dwc_timer_t *DWC_TIMER_ALLOC(char *name, dwc_timer_callback_t cb, void *data)
{

	dwc_timer_t *t = DWC_ALLOC(sizeof(*t));
	/*OS_ERR_TYPE err;*/

	if (t == NULL) {
		DWC_ERROR("Cannot allocate memory for timer");
		return NULL;
	}

	t->lock = DWC_SPINLOCK_ALLOC();
	if (!t->lock) {
		DWC_ERROR("Cannot allocate memory for lock");
		goto no_lock;
	}
	t->scheduled = 0;
	t->cb = cb;
	t->data = data;
	/*t->t = timer_create(timer_callback, (void *)t, 0, 0, 0, &err);
	if (err == E_OS_ERR) {
		DWC_ERROR("Cannot allocate memory for timer->t");
		goto no_timer;
	}*/



	return t;

/* no_timer: */
	DWC_SPINLOCK_FREE(t->lock);
no_lock:
	DWC_FREE(t);
	return NULL;

}

void DWC_TIMER_FREE(dwc_timer_t * timer)
{
	DWC_SPINLOCK_FREE(timer->lock);
/* 	timer_delete(timer->t, NULL); */
	DWC_FREE(timer);
}

void DWC_TIMER_SCHEDULE(dwc_timer_t * timer, uint32_t time)
{
/*	OS_ERR_TYPE err;
	dwc_irqflags_t flags;

	DWC_SPINLOCK_IRQSAVE(timer->lock, &flags);

	if (!timer->scheduled) {
		timer->scheduled = 1;
		DWC_DEBUG("Scheduling timer to expire in +%d msec",
			  time);
		timer_start(timer->t, time, &err);
	} else {
		DWC_DEBUG("Modifying timer to expire in +%d msec",
			  time);
		timer_stop(timer->t, NULL);
		timer_start(timer->t, time, &err);
	}

	DWC_SPINUNLOCK_IRQRESTORE(timer->lock, flags);*/
	/* Timers seems not used for us 
	 * directly call the callback */
	timer->cb(timer->data);
}

void DWC_TIMER_CANCEL(dwc_timer_t * timer)
{
/* 	timer_stop(timer->t, NULL); */
}

/* Wait Queues */
/* TODO: REMOVE WAIT QUEUE */

struct dwc_waitq {
	void *queue;
	int abort;
};

dwc_waitq_t *DWC_WAITQ_ALLOC(void)
{
	return NULL;
}

void DWC_WAITQ_FREE(dwc_waitq_t * wq)
{
}

int32_t DWC_WAITQ_WAIT(dwc_waitq_t * wq, dwc_waitq_condition_t cond, void *data)
{
	return 42;
}

int32_t DWC_WAITQ_WAIT_TIMEOUT(dwc_waitq_t * wq, dwc_waitq_condition_t cond,
			       void *data, int32_t msecs)
{
	return 42;
}

void DWC_WAITQ_TRIGGER(dwc_waitq_t * wq)
{
}

void DWC_WAITQ_ABORT(dwc_waitq_t * wq)
{
}

/* tasklets
 - run in interrupt context (cannot sleep)
 - each tasklet runs on a single CPU
 - different tasklets can be running simultaneously on different CPUs
 */
/* TODO: the current implementation does not use any tasklet-like feature. It
 * calls the callback directly instead of queuing it */

struct dwc_tasklet {
	dwc_tasklet_callback_t cb;
	void *data;
};

dwc_tasklet_t *DWC_TASK_ALLOC(char *name, dwc_tasklet_callback_t cb, void *data)
{
	dwc_tasklet_t *t = DWC_ALLOC(sizeof(*t));

	if (t == NULL) {
		return NULL;
	}
	t->cb = cb;
	t->data = data;
	return t;
}

void DWC_TASK_FREE(dwc_tasklet_t * task)
{
	DWC_FREE(task);
}

void DWC_TASK_SCHEDULE(dwc_tasklet_t * task)
{
	task->cb(task->data);
}

/* workqueues
 - run in process context (can sleep)
 */

/* TODO: Seems to be used only for OTG
 * so do a synchronous implementation, hopping it will never be called
 * TODO: VERIFY it is never called and remove this code */

struct dwc_workq {
	int dummy;
};

int DWC_WORKQ_WAIT_WORK_DONE(dwc_workq_t * workq, int timeout)
{
	/* Work is always done at time */
	return 0;
}

dwc_workq_t *DWC_WORKQ_ALLOC(char *name)
{
	/* return 0 said 'fail', so return 1
	 * The allocated wq is never dereferenced in code so it is safe */
	return (dwc_workq_t *) 1;
}

void DWC_WORKQ_FREE(dwc_workq_t * wq)
{
	/* nothing allocated */
}

void DWC_WORKQ_SCHEDULE(dwc_workq_t * wq, dwc_work_callback_t cb, void *data,
			char *format, ...)
{
	/* Do Stuff synchronously */
	cb(data);
}

void DWC_WORKQ_SCHEDULE_DELAYED(dwc_workq_t * wq, dwc_work_callback_t cb,
				void *data, uint32_t time, char *format, ...)
{
	/* Time is money, do not wait
	 * Do Stuff synchronously */
	cb(data);
}

int DWC_WORKQ_PENDING(dwc_workq_t * wq)
{
	return 0;
}
