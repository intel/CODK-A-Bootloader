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

#include <gpio/soc_gpio.h>

#define GPIO_CLKENA_POS         (31)
#define GPIO_LS_SYNC_POS        (0)


typedef void (*ISR) ();

/*! GPIO management structure */
typedef struct gpio_info_struct
{
    /* static settings */
    uint32_t           reg_base;       /*!< base address of device register set */
    uint8_t            no_bits;        /*!< no of gpio bits in this entity */
    uint8_t            vector;         /*!< GPIO ISR vector */
    ISR                gpio_isr;       /*!< GPIO ISR */
    uint32_t           gpio_int_mask;  /*!< SSS Interrupt Routing Mask Registers */
    gpio_callback_fn  *gpio_cb;        /*!< Array of user callback functions for user */
    void             **gpio_cb_arg;    /*!< Array of user priv data for callbacks */
    uint8_t            is_init;        /*!< Init state of GPIO port */
} gpio_info_t, *gpio_info_pt;

static gpio_info_t gpio_ports_devs[] = {

        { .is_init = 0,
          .reg_base = SOC_GPIO_BASE_ADDR,
          .no_bits = SOC_GPIO_32_BITS,
          .gpio_int_mask = INT_GPIO_MASK,
          .vector = SOC_GPIO_INTERRUPT,
          .gpio_cb = NULL,
          .gpio_cb_arg = NULL,
          .gpio_isr = NULL },
        };


uint8_t soc_gpio_enable(uint8_t port_id)
{

    gpio_info_pt dev = &gpio_ports_devs[port_id];

    /* enable peripheral clock */
    SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_LS_SYNC), (uint32_t)GPIO_CLKENA_POS);
    SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_LS_SYNC), (uint32_t)GPIO_LS_SYNC_POS);
    /* Clear any existing interrupts */
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_PORTA_EOI) = ~(0);
    /* enable interrupt for this GPIO block */
    SET_INTERRUPT_HANDLER(dev->vector, dev->gpio_isr);
    /* Enable GPIO  Interrupt into SSS  */
    SOC_UNMASK_INTERRUPTS(dev->gpio_int_mask);

    dev->is_init = 1;

    return DRV_RC_OK;
}

uint8_t soc_gpio_set_config(uint8_t port_id, uint8_t bit, gpio_cfg_data_t *config)
{
    uint8_t ret = DRV_RC_FAIL;
    uint32_t saved;

    gpio_info_pt dev = &gpio_ports_devs[port_id];

    // Check if device is initialized
    if(dev->is_init == 0) {
        if((ret = soc_gpio_enable(port_id)) != DRV_RC_OK) {
            return ret;
        }
    }

    /* Disable Interrupts from this bit */
    CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_INTEN), (uint32_t)bit);

    switch(config->gpio_type)
    {
    case GPIO_INPUT:
        /* configure as input */
        CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_SWPORTA_DDR), (uint32_t)bit);
        break;
    case GPIO_OUTPUT:
        /* configure as output */
        SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_SWPORTA_DDR), (uint32_t)bit);
        break;
    default:
        return ret;
    }

    return DRV_RC_OK;
}

uint8_t soc_gpio_write(uint8_t port_id, uint8_t bit, uint8_t value)
{
    gpio_info_pt dev = &gpio_ports_devs[port_id];

    // Check pin index
    if (bit >= dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }
    /* read/modify/write bit */
    if (value) {
        SET_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_SWPORTA_DR), (uint32_t)bit);
    } else {
        CLEAR_MMIO_BIT((volatile uint32_t *)(dev->reg_base+SOC_GPIO_SWPORTA_DR), (uint32_t)bit);
    }

    return DRV_RC_OK;
}

uint8_t soc_gpio_write_port(uint8_t port_id, uint32_t value)
{
    gpio_info_pt dev = &gpio_ports_devs[port_id];
    MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_SWPORTA_DR) = value;
    return DRV_RC_OK;
}

uint8_t soc_gpio_read(uint8_t port_id, uint8_t bit)
{
    gpio_info_pt dev = &gpio_ports_devs[port_id];
    // Check pin index
    if (bit >= dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }
    if (MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_SWPORTA_DDR) & (1 << bit)) {
        return DRV_RC_INVALID_OPERATION;          /* not configured as input */
    }
    return !!(MMIO_REG_VAL_FROM_BASE(dev->reg_base, SOC_GPIO_EXT_PORTA) & (1 << bit));
}
