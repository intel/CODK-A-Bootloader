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

#ifndef __IRQ_H__
#define __IRQ_H__
/*
 * \brief Disable all (unmasked) interrupts.
 *  Authorized execution levels:  task, fiber, ISR.
 *
 * \return a key representing the IRQ that were enabled before calling
 this service.
 */


static inline __attribute__((always_inline)) unsigned int interrupt_lock(void)
{
	unsigned int key;

	__asm__ volatile (
			"pushfl;\n\t"
			"cli;\n\t"
			"popl %0;\n\t"
			: "=g" (key)
			:
			: "memory"
			 );
	return key;
}

/*
 * \brief Enable the set of interrupts that were disabled by
 interrupt_disable_all.
 *  Authorized execution levels:  task, fiber, ISR.
 *
 * \param key: value returned by interrupt_disable_all.
 *
 */
static inline __attribute__((always_inline))
void interrupt_unlock(unsigned int key)
{
	__asm__ volatile (
			"testl $0x200, %0;\n\t"
			"jz .end;\n\t"
			"sti;\n\t"
			".end:"
			:
			: "g" (key)
			: "cc"
		);
}
#endif /* __IRQ_H__ */
