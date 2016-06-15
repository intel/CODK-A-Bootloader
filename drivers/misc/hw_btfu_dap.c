/*
 * Copyright (C) 2014 Silicon Labs, http://www.silabs.com
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 */

/*
 * 2015: file modified by Intel Corporation
 */

#include <swd/hw_btfu_dap.h>

/*!
 * GPIO configuration structure
 */
static gpio_cfg_data_t pin_cfg = {
  .gpio_type = GPIO_OUTPUT,
  .int_type = LEVEL,
  .int_polarity = ACTIVE_LOW,
  .int_debounce= DEBOUNCE_OFF,
  .int_ls_sync =LS_SYNC_OFF,
  .gpio_cb = NULL};

#define swd_gpio_set_input(pin)                                                \
  do {                                                                         \
        MMIO_REG_VAL_FROM_BASE((SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DDR), 0) \
        &= ~(1 << pin);                                                        \
  } while(0)

#define swd_gpio_set_output(pin)                                               \
  do {                                                                         \
        MMIO_REG_VAL_FROM_BASE((SOC_GPIO_BASE_ADDR + SOC_GPIO_SWPORTA_DDR), 0) \
        |= (1 << pin);                                                        \
  } while(0)

//------------------------------------------------------------------------------
// Private Function Definitions
//------------------------------------------------------------------------------
static btfu_dap_error_t JTAG_to_SWD_Sequence(void);
static btfu_dap_error_t readDP(int reg, uint32_t *data);
static btfu_dap_error_t writeDP(int reg, uint32_t data);
static btfu_dap_error_t readReg(uint8_t ap, int reg, uint32_t *data);
static btfu_dap_error_t writeReg(uint8_t ap, int reg, uint32_t data,
                                 uint8_t ignoreAck);
/*******************************************************************************
 * Initialize the interface
 ******************************************************************************/

btfu_dap_error_t hw_BtfuDapInit(void)
{
    swd_gpio_set_output(NRF_SWCLK_PIN);
    SWCLK_CLR();
    swd_gpio_set_output(NRF_SWDIO_PIN);
    return SWD_ERROR_OK;
}
/**********************************************************
 * Close pins and clocks used for by application.
 * Leave these pins in tristate when not used so it doesn't
 * disrupt external J-link.
 **********************************************************/
void hw_BtfuDapHibernate(void)
{
    SWCLK_CLR();
    SWDIO_SET();
    swd_gpio_set_input(NRF_SWCLK_PIN);
    swd_gpio_set_input(NRF_SWDIO_PIN);
}

/**********************************************************
 * Performs a pin reset on the target
 * Only works in 'normal mode' not 'debug mode' and
 * SWCLK must be held low while SWDIO transitioned from high to low to high
 **********************************************************/
void hw_BtfuDapHardReset(void)
{
    SWCLK_CLR();    //ensure the SWCLK is low
    SWDIO_SET();    //ensure the SWDIO starts high
    mdelay(1);      //no particular timing.
    SWDIO_CLR();    //set the reset pin low
    mdelay(1);      //minimum hold time is 100usec.
    SWDIO_SET();    //release the reset pin
}

/**********************************************************
 * Performs a pin reset on the target. But Hold the Reset PIN
 * to prevent it to start.
 * The release should be done by calling hw_BtfuDapHardResetRelease
 **********************************************************/
void hw_BtfuDapHardResetHold(void)
{
    SWCLK_CLR();    //ensure the SWCLK is low
    SWDIO_SET();    //ensure the SWDIO starts high
    mdelay(1);                                           //no particular timing.
    SWCLK_CLR();    //set the reset pin low
}

/**********************************************************
 * Release the Reset PIN to allow the device start.
 * Should be called after hw_BtfuDapHardResetHold
 **********************************************************/
void hw_BtfuDapHardResetRelease(void)
{
    mdelay(1);      //minimum hold time is 100usec.
    SWDIO_SET();    //release the reset pin
}

/**********************************************************
 * Initialized the SW-DP. This function first
 * sends the JTAG-to-SWD sequence and then reads
 * the IDCODE register and checks it for validity.
 *
 * @returns     SWD_ERROR_OK or an error code
 **********************************************************/
btfu_dap_error_t hw_BtfuDapInitDp(void)
{
    uint32_t dp_id;
    uint32_t status;
    btfu_dap_error_t error_code;

    /* Send the JTAG-to-SWD switching sequence */
    JTAG_to_SWD_Sequence();

    /* Read IDCODE to get the DAP out of reset state */
    if ((error_code = readDP(DP_IDCODE, &dp_id)) != SWD_ERROR_OK) {
        return (error_code);
    }

    if (dp_id != NRF51_DPID_1) {
//        pr_warning(LOG_MODULE_BL, "WRN_BTFU_CONNECT - Read IDCODE = 0x%x",
//                   dp_id);
        return SWD_ERROR_INVALID_IDCODE;
    }

    /* Read DP_CTRL to check status */
    if ((error_code = readDP(DP_CTRL, &status)) != SWD_ERROR_OK) {
        return (error_code);
    }

    /* Check for errors.  Overrun is possible if the remote device was already in
     * SWD mode */
    if (status
            & (DP_CTRL_STICKYORUN | DP_CTRL_STICKYCMP | DP_CTRL_STICKYERR
                    | DP_CTRL_WDATAERR)) {
        if ((error_code = writeDP(
                DP_ABORT,
                (DP_ABORT_STKCMPCLR | DP_ABORT_STKERRCLR | DP_ABORT_WDERRCLR
                        | DP_ABORT_ORUNERRCLR))) != SWD_ERROR_OK) {
            return (error_code);
        }
    }

    /* Has a reset already been done?  If not, do one now */
    if ((status & (DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK))
            != (DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK)) {
        /* Debug power up request */
        if ((error_code = writeDP(DP_CTRL,
                                  DP_CTRL_CSYSPWRUPREQ | DP_CTRL_CDBGPWRUPREQ))
                != SWD_ERROR_OK) {
            return (error_code);
        }

        /* Wait until we receive powerup ACK */
        uint8_t retry = PWRUP_RETRY_COUNT;
        do {
            error_code = readDP(DP_CTRL, &status);
            if ((status & (DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK))
                    != (DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK)) {
                error_code = SWD_ERROR_DEBUG_POWER;
            }
        } while ((error_code != SWD_ERROR_OK) && (retry-- > 0));

        if (error_code != SWD_ERROR_OK) {
            return (error_code);
        }
    }

    /* Select first AP bank */
    if ((error_code = writeDP(DP_SELECT, 0x00)) != SWD_ERROR_OK) {
        return (error_code);
    }

    return SWD_ERROR_OK;
}

/**********************************************************
 * Reads the ID of AP #0.
 * The value of IDR register (address 0xFC) for AP #0
 *
 * @returns     SWD_ERROR_OK or an error code
 **********************************************************/
btfu_dap_error_t hw_BtfuDapReadApId(void)
{
    uint32_t ap_id;
    btfu_dap_error_t error_code;

    /* Select last AP bank */
    if ((error_code = writeDP(DP_SELECT, 0xf0)) != SWD_ERROR_OK) {
        return error_code;
    }

    /* Dummy read AP ID */
    if ((error_code = hw_BtfuDapReadAP(AP_IDR, &ap_id)) != SWD_ERROR_OK) {
        return error_code;
    }

    /* Read AP ID */
    if ((error_code = readDP(DP_RDBUFF, &ap_id)) != SWD_ERROR_OK) {
        return error_code;
    }

    /* Select first AP bank again */
    if ((error_code = writeDP(DP_SELECT, 0x00)) != SWD_ERROR_OK) {
        return error_code;
    }
    /* Check against the valid IDs for this CPU */
    if (ap_id != NRF51_APB_AP_ID_1) {
//        pr_warning(LOG_MODULE_BL, "WRN_BTFU_CONNECT - Read IDR = 0x%x", ap_id);
        return SWD_ERROR_INVALID_IDR;  // Valid for NRF51
    }

    return SWD_ERROR_OK;
}

/**********************************************************
 * Initialize the AHB-AP. The transfer size must be set
 * to 32-bit before trying to access any internal
 * memory.
 *
 * @returns     SWD_ERROR_OK or an error code
 **********************************************************/
btfu_dap_error_t hw_BtfuDapInitAhbAp(void)
{
    /* Set transfer size to 32 bit */
    return hw_BtfuDapWriteAP(AP_CSW, AP_CSW_DEFAULT);
}

/**********************************************************
 * Sends the JTAG-to-SWD sequence. This must be performed
 * at the very beginning of every debug session and
 * again in case of a protocol error.
 **********************************************************/
/* From Nordic Reference Manual 10.1.2
 Debug interface mode is initiated by clocking one clock cycle on SWDCLK with SWDIO=1. Due to delays
 caused by starting up the DAP's power domain, a minimum of 150 clock cycles must be clocked at a speed
 of minimum 125 kHz on SWDCLK with SWDIO=1 to guaranty that the DAP is able to capture a minimum of
 50 clock cycles.
 */
static btfu_dap_error_t JTAG_to_SWD_Sequence(void)
{
    int i;
    int b;

    swd_gpio_set_output(NRF_SWCLK_PIN);
    swd_gpio_set_output(NRF_SWDIO_PIN);

    /*
     *  First reset line with > 50 cycles with SWDIO high
     *  Minimum 125 kHz on SWDCLK *
     */
    SWDIO_OUT(1);
    for (i = 0; i < 150; i++) {
        SWCLK_CYCLE();
    }

    /* Transmit 16-bit JTAG-to-SWD sequence */
    for (i = 0; i < 16; i++) {
        b = (JTAG2SWD >> i) & 0x1;
        WRITE_BIT(b);
    }

    /* Do another reset to make sure SW-DP is in reset state */
    SWDIO_OUT(1);
    for (i = 0; i < 150; i++) {
        SWCLK_CYCLE();
    }

    /* Insert a 16 cycle idle period */
    SWDIO_OUT(0);
    for (i = 0; i < 16; i++) {
        SWCLK_CYCLE();
    }
    return SWD_ERROR_OK;
}

/**********************************************************
 * Reads one word from internal memory
 *
 * @param addr
 *    The address to read from
 *
 * @returns
 *    The value at @param addr
 **********************************************************/
uint32_t hw_BtfuDapReadMem(uint32_t addr)
{
    uint32_t ret = 0;

    if (hw_BtfuDapWriteAP(AP_TAR, addr) != SWD_ERROR_OK) {
//        pr_info(LOG_MODULE_BL, "WRN_BTFU_READ_REG");
    }

    if (hw_BtfuDapReadAP(AP_DRW, &ret) != SWD_ERROR_OK) {
//        pr_info(LOG_MODULE_BL, "WRN_BTFU_READ_REG");
    }

    if (readDP(DP_RDBUFF, &ret) != SWD_ERROR_OK) {
//        pr_info(LOG_MODULE_BL, "WRN_BTFU_READ_REG");
    }

    return ret;
}

/**********************************************************
 * Reads one of the four AP registers in the currently
 * selected AP bank.
 *
 * @param reg[in]
 *    The register number [0-3] to read
 *
 * @param data[out]
 *    Value of register is written to this parameter
 * @return   SWD_ERROR_OK or an error code.
 **********************************************************/
btfu_dap_error_t hw_BtfuDapReadAP(int reg, uint32_t *data)
{
    btfu_dap_error_t swdStatus;
    uint8_t retry = SWD_RETRY_COUNT;

    do {
        swdStatus = readReg(1, reg, data);
        retry--;
    } while (swdStatus == SWD_ERROR_WAIT && retry > 0);

    return (swdStatus);
}

/**********************************************************
 * Reads one of the four DP registers.
 *
 * @param reg[in]
 *    The register number [0-3] to read
 *
 * @param data[out]
 *    Value of register is written to this parameter
 * @return   SWD_ERROR_OK or an error code.
 **********************************************************/
static btfu_dap_error_t readDP(int reg, uint32_t *data)
{
    btfu_dap_error_t swdStatus;
    uint8_t retry = SWD_RETRY_COUNT;

    do {
        swdStatus = readReg(0, reg, data);
        retry--;
    } while (swdStatus == SWD_ERROR_WAIT && retry > 0);

    return (swdStatus);
}

/**********************************************************
 * Writes one word to internal memory
 *
 * @param addr
 *    The address to write to
 *
 * @param data
 *    The value to write
 *
 * @returns
 *    The value at @param addr
 **********************************************************/
void hw_BtfuDapWriteMem(uint32_t addr, uint32_t data)
{
    if (hw_BtfuDapWriteAP(AP_TAR, addr) != SWD_ERROR_OK) {
//        pr_info(LOG_MODULE_BL, "WRN_BTFU_READ_REG");
    }

    if (hw_BtfuDapWriteAP(AP_DRW, data) != SWD_ERROR_OK) {
//        pr_info(LOG_MODULE_BL, "WRN_BTFU_READ_REG");
    }
}

/**********************************************************
 * Writes to one of the four AP registers in the currently
 * selected AP bank.
 *
 * @param reg[in]
 *    The register number [0-3] to write to
 *
 * @param data[in]
 *    Value to write to the register
 *
 **********************************************************/
btfu_dap_error_t hw_BtfuDapWriteAP(int reg, uint32_t data)
{
    btfu_dap_error_t swdStatus;
    uint8_t retry = SWD_RETRY_COUNT;

    do {
        swdStatus = writeReg(1, reg, data, 0);
        retry--;
    } while (swdStatus == SWD_ERROR_WAIT && retry > 0);

    return (swdStatus);
}

/**********************************************************
 * Writes to one of the four DP registers.
 *
 * @param reg[in]
 *    The register number [0-3] to write to
 *
 * @param data[in]
 *    Value to write to the register
 * @return   SWD_ERROR_OK or an error code.
 **********************************************************/
static btfu_dap_error_t writeDP(int reg, uint32_t data)
{
    btfu_dap_error_t swdStatus;
    uint8_t retry = SWD_RETRY_COUNT;

    do {
        swdStatus = writeReg(0, reg, data, 0);
        retry--;
    } while (swdStatus == SWD_ERROR_WAIT && retry > 0);

    return (swdStatus);
}

/**********************************************************
 * Reads from an AP or DP register.
 *
 * @param ap
 *   If this parameter is true, read from AP register.
 *   If false read from DP register.
 *
 * @param reg
 *   The register number [0-3] to read from
 *
 * @param data[out]
 *   The register value is written to this parameter
 **********************************************************/
static btfu_dap_error_t readReg(uint8_t ap, int reg, uint32_t *data)
{
    int i;
    uint32_t cb = 0;
    uint32_t parity;
    uint32_t b;
    uint32_t ack = 0;
    btfu_dap_error_t ret = SWD_ERROR_OK;

    /* Initalize output variable */
    *data = 0;

    /* Convert to int */
    int _read = (int) 1;

    int A2 = reg & 0x1;
    int A3 = (reg >> 1) & 0x1;

    /* Calulate parity */
    parity = (ap + _read + A2 + A3) & 0x1;

    swd_gpio_set_output(NRF_SWDIO_PIN);

    /* Send request */
    WRITE_BIT(1);
    WRITE_BIT(ap);
    WRITE_BIT(_read);
    WRITE_BIT(A2);
    WRITE_BIT(A3);
    WRITE_BIT(parity);
    WRITE_BIT(0);
    WRITE_BIT(1);

    /* Turnaround */
    swd_gpio_set_input(NRF_SWDIO_PIN);

    SWCLK_CYCLE();

    /* Read ACK */
    for (i = 0; i < 3; i++) {
        READ_A_BIT(b);
        ack |= b << i;
    }

    /* Verify that ACK is OK */
    if (ack == ACK_OK) {

        for (i = 0; i < 32; i++) {
            /* Read bit */
            READ_A_BIT(b);
            *data |= b << i;

            /* Keep track of expected parity */
            if (b)
                cb = !cb;
        }

        /* Read parity bit */
        READ_A_BIT(parity);

        /* Verify parity */
        if (cb == parity) {
            ret = SWD_ERROR_OK;
        } else {
            ret = SWD_ERROR_PARITY;
        }

    } else if (ack == ACK_WAIT) {
        ret = SWD_ERROR_WAIT;
    } else if (ack == ACK_FAULT) {
        ret = SWD_ERROR_FAULT;
    } else {
        /* Line not driven. Protocol error */
        ret = SWD_ERROR_PROTOCOL;
    }

    /* Turnaround */
    SWCLK_CYCLE();

    /* 8-cycle idle period. Make sure transaction
     * is clocked through DAP. */
    swd_gpio_set_output(NRF_SWDIO_PIN);

    for (i = 0; i < 8; i++) {
        WRITE_BIT(0);
    }

    return ret;
}

/**********************************************************
 * Writes to a DP or AP register.
 *
 * @param ap
 *   If this parameter is true, write to AP register.
 *   If false write to DP register.
 *
 * @param reg
 *   The register number [0-3] to write to
 *
 * @param data
 *   The value to write to the register
 **********************************************************/
static btfu_dap_error_t writeReg(uint8_t ap, int reg, uint32_t data,
                                 uint8_t ignoreAck)
{
    uint32_t ack = 0;
    int i;
    uint32_t parity = 0;
    uint32_t b;
    btfu_dap_error_t ret = SWD_ERROR_OK;

    /* Convert to int */
    int _read = (int) 0;

    /* Calulate address bits */
    int A2 = reg & 0x1;
    int A3 = (reg >> 1) & 0x1;

    /* Calculate parity */
    parity = (ap + _read + A2 + A3) & 0x1;

    swd_gpio_set_output(NRF_SWDIO_PIN);

  /* Write request */
    WRITE_BIT(1);
    WRITE_BIT(ap);
    WRITE_BIT(_read);
    WRITE_BIT(A2);
    WRITE_BIT(A3);
    WRITE_BIT(parity);
    WRITE_BIT(0);
    WRITE_BIT(1);

    swd_gpio_set_input(NRF_SWDIO_PIN);

    /* Turnaround */
    SWCLK_CYCLE();

    /* Read acknowledge */
    for (i = 0; i < 3; i++) {
        READ_A_BIT(b);
        ack |= b << i;
    }

    if (ack == ACK_OK || ignoreAck == 1) {
        /* Turnaround */
        SWCLK_CYCLE();
        swd_gpio_set_output(NRF_SWDIO_PIN);

        /* Write data */
        parity = 0;
        for (i = 0; i < 32; i++) {
            b = (data >> i) & 0x1;
            WRITE_BIT(b);
            if (b)
                parity = !parity;
        }

        /* Write parity bit */
        WRITE_BIT(parity);

    } else if (ack == ACK_WAIT) {
        ret = SWD_ERROR_WAIT;
    } else if (ack == ACK_FAULT) {
        ret = SWD_ERROR_FAULT;
    } else {
        /* Line not driven. Protocol error */
        ret = SWD_ERROR_PROTOCOL;
    }

    /* 8-cycle idle period. Make sure transaction
     * is clocked through DAP. */
    swd_gpio_set_output(NRF_SWDIO_PIN);

    for (i = 0; i < 8; i++) {
        WRITE_BIT(0);
    }

    return ret;

}
