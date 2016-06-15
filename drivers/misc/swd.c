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

#include "swd/swd.h"

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------
/* Value to write to SYSTEM_CTRL_BLOCK in order to do a soft reset of the target 0x05FA0006 */

#define SYSTEM_CTRL_BLOCK_RESET_CMD ((0x05FA << SYSTEM_CTRL_BLOCK_VECTKEY_BIT) | SYSTEM_CTRL_BLOCK_SYS_RESET_MASK | SYSTEM_CTRL_BLOCK_VECT_CLR_ACTIVE_MASK)

static struct coredebug_struct* core_debug = (struct coredebug_struct*) CoreDebug_BASE ;   /*!< Core Debug configuration struct    */
static struct scb_struct* scb = (struct scb_struct*) SCB_BASE;

/* Commands to run/step and let CPU run.
 * Write these to DHCSR */

#define RUN_CMD  0xA05F0001
#define STOP_CMD 0xA05F0003
#define STEP_CMD 0xA05F0005

//-----------------------------------------------------------------------------
// Function Declarations
//------------------------------------------------------------------------------
static uint8_t reset_and_halt_target(void);
static uint8_t write_nvmc_config(uint32_t value);

//------------------------------------------------------------------------------
// Public Functions
//------------------------------------------------------------------------------

uint8_t swd_connect_to_target(void)
{
  uint8_t error_code;
  uint8_t connect_retry = CONNECT_RETRY_COUNT;

  /* Try connecting several times */
  do
  {
     mdelay(1);
     /* Initialize the Debug Port */
     if ((error_code = hw_BtfuDapInitDp()) != SWD_ERROR_OK)
     {
        continue;
     }

     /* Verify that the AP returns the correct ID */
     uint8_t retry = AHB_IDR_RETRY_COUNT;
     do
     {
        error_code = hw_BtfuDapReadApId();
     } while ((error_code != SWD_ERROR_OK) && (retry-- > 0));

     if (error_code != SWD_ERROR_OK)
     {
        continue;
     }

     /* Set up parameters for AHB-AP. This must be done before accessing
      * internal memory. */
     if ((error_code = hw_BtfuDapInitAhbAp()) != SWD_ERROR_OK)
     {
        continue;
     }

     /* Halt the processor */
     if ((error_code = reset_and_halt_target()) != SWD_ERROR_OK)
     {
        continue;
     }
  } while ((error_code != SWD_ERROR_OK) && (connect_retry-- > 0));

  return error_code;
}


uint8_t swd_unlock_target(void)
{
   uint32_t value;

   /* Check for a preprogrammed Nordic - if so we can't flash it ourselves */
   value = hw_BtfuDapReadMem((uint32_t)&(NRF_FICR->PPFC));
   if ( (value & FICR_PPFC_PPFC_Msk) != FICR_PPFC_PPFC_NotPresent )
   {  /* also NRF_FICR->CLENR0 and NRF_FICR->CONFIGID->FWID != 0xFFFFFFFF
         but lets not check everything */
      /* We shouldn't continue in this case as this is should not be possible in chip */
      return SWD_ERROR_MCU_LOCKED;
   }

   /* Unlock the MPU Write/erase protection in debug mode. */
   hw_BtfuDapWriteMem((uint32_t)&(NRF_MPU->DISABLEINDEBUG), MPU_DISABLEINDEBUG_DISABLEINDEBUG_Disabled);

  return SWD_ERROR_OK;
}


uint8_t swd_soft_reset_target(void)
{
  uint32_t dhcsr;
  uint8_t timeout = DEBUG_EVENT_RETRY_COUNT;

  /* Clear the VC_CORERESET bit */
  hw_BtfuDapWriteMem((uint32_t)&(core_debug->DEMCR), 0);

  /* Do a dummy read of sticky bit to make sure it is cleared */
  hw_BtfuDapReadMem((uint32_t)&(core_debug->DHCSR));
  dhcsr = hw_BtfuDapReadMem((uint32_t)&(core_debug->DHCSR));

  /* Reset CPU */
  hw_BtfuDapWriteMem((uint32_t)&(scb->AIRCR), SYSTEM_CTRL_BLOCK_RESET_CMD);

  /* Wait for reset to complete */

  /* First wait until sticky bit is set. This means we are
   * or have been in reset */
  mdelay(1);
  do {
    mdelay(1);
    dhcsr = hw_BtfuDapReadMem((uint32_t)&(core_debug->DHCSR));
    timeout--;
  } while ( !(dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk) && timeout > 0 );

  /* Throw error if sticky bit is never set */
  if ( !(dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk) ) {
    return (SWD_ERROR_TIMEOUT_WAITING_RESET);
  }

  /* Wait for sticky bit to be cleared. When bit is cleared are we out of reset */
  timeout = DEBUG_EVENT_RETRY_COUNT;
  do {
    mdelay(1);
    dhcsr = hw_BtfuDapReadMem((uint32_t)&(core_debug->DHCSR));
    timeout--;
  } while ( (dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk) && timeout > 0 );

  /* Throw error if bit is never cleared */
  if ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk ) {
    return (SWD_ERROR_TIMEOUT_WAITING_RESET);
  }

  /* Wait a few milliseconds for the Nordic to initialize before returning.
   * This fixes some errors related to installing new Nordic Firmware/SoftDevice. */
  mdelay(50);

  return (SWD_ERROR_OK);
}

uint8_t swd_debug_mode_reset_to_normal(void)
{
   /* Set the Enable in the Power->Reset register */
   hw_BtfuDapWriteMem((uint32_t)&(NRF_POWER->RESET), POWER_RESET_RESET_Enabled);

   hw_BtfuDapHardReset();
   hw_BtfuDapHibernate();
   return (SWD_ERROR_OK);
}

static uint8_t reset_and_halt_target(void)
{
  uint32_t dhcsr;
  btfu_dap_error_t error_code;
  uint8_t timeout = DEBUG_EVENT_RETRY_COUNT;

  /* Halt target first. This is necessary before setting
   * the VECTRESET bit */
  if ((error_code = swd_halt_target()) != SWD_ERROR_OK)
  {
     return error_code;
  }

  /* Set halt-on-reset bit */
  hw_BtfuDapWriteMem((uint32_t)&(core_debug->DEMCR), CoreDebug_DEMCR_VC_CORERESET_Msk);

  /* Clear exception state and reset target */
  hw_BtfuDapWriteMem((uint32_t)&(scb->AIRCR), SYSTEM_CTRL_BLOCK_RESET_CMD);

  /* Wait for target to reset */
  do {
    mdelay(1);
    timeout--;
    dhcsr = hw_BtfuDapReadMem((uint32_t)&(core_debug->DHCSR));
  } while ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk );

  /* Check if we timed out */
  if ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk )
  {
    return (SWD_ERROR_TIMEOUT_WAITING_RESET);
  }

  /* Verify that target is halted */
  if ( !(dhcsr & CoreDebug_DHCSR_S_HALT_Msk) )
  {
    return (SWD_ERROR_TARGET_NOT_HALTED);
  }
  return (SWD_ERROR_OK);
}


uint8_t swd_halt_target(void)
{
  uint32_t dhcsr;
  uint8_t timeout = DEBUG_EVENT_RETRY_COUNT;

  hw_BtfuDapWriteMem((uint32_t)&(core_debug->DHCSR), STOP_CMD);

  do {
    dhcsr = hw_BtfuDapReadMem((uint32_t)&(core_debug->DHCSR));
    timeout--;
  } while ( !(dhcsr & CoreDebug_DHCSR_S_HALT_Msk) && timeout > 0 );

  if ( !(dhcsr & CoreDebug_DHCSR_S_HALT_Msk) ) {
    return (SWD_ERROR_TIMEOUT_HALT);
  }
  return (SWD_ERROR_OK);

}


void swd_run_target(void)
{
  hw_BtfuDapWriteMem((uint32_t)&(core_debug->DHCSR), RUN_CMD);
}


void swd_step_target(void)
{
  hw_BtfuDapWriteMem((uint32_t)&(core_debug->DHCSR), STEP_CMD);
}

uint8_t swd_init(void)
{
    btfu_dap_error_t error_code = SWD_ERROR_OK;

    SET_PIN_MODE(NRF_SWCLK_PIN, QRK_PMUX_SEL_MODEA);
    SET_PIN_MODE(NRF_SWDIO_PIN, QRK_PMUX_SEL_MODEA);

    hw_BtfuDapInit();
    hw_BtfuDapHardReset();

    //Connect to the target in debug mode
    if ((error_code = swd_connect_to_target()) != SWD_ERROR_OK){
       return error_code;
    }

    //Unlock the MPU
    if ((error_code = swd_unlock_target()) != SWD_ERROR_OK){
       return error_code;
    }

    return error_code;
}

uint8_t write_nvmc_config(uint32_t value)
{
    uint32_t mscStatus;
    uint8_t timeOut;

    // Enable erase
    hw_BtfuDapWriteMem((uint32_t) & (NRF_NVMC->CONFIG), value);
    // Wait until it takes effect
    timeOut = NVMC_MODE_TIMEOUT;
    do {
        mscStatus = hw_BtfuDapReadMem((uint32_t) & (NRF_NVMC->READY));
        timeOut--;
    } while (mscStatus == NVMC_READY_READY_Busy && timeOut > 0);

    if (mscStatus == NVMC_READY_READY_Busy)
        return SWD_ERROR_CONFIG_WRITE_FAILED;

    return SWD_ERROR_OK;
}

/* ! \fn     uint8_t swd_dump_image(uint32_t addr, uint32_t *fw_image, uint32_t len)
*
*  \brief   Direct memory reads in autoincrement mode.
*
* \param  addr          : Start address in Nordic flash for the read
*                         Reads are always 32bit reads so the address
*                         should always be 4 byte aligned.
* \param fw_image       : Pointer to the output buffer
* \param len            : Length in bytes of the data to read
*
* \return               SWD_ERROR_OK if success, or an error code otherwise.
*/

uint8_t swd_dump_image(uint32_t addr, uint32_t *fw_image, uint32_t len)
{
    uint32_t value;
    uint32_t buff_offset = 0;
    btfu_dap_error_t ret;

    /* FIXME do assert on buffer address not null */
    if ((addr & 0x3) != 0 || len == 0) {
        return SWD_ERROR_FLASH_WRITE_FAILED;
    }
    //extend length to multiple of 4
    if ((len & 0x3) != 0) {
        len = len + (4 - (len & 0x3));
    }

    // Enable read-only mode
    if ((ret = write_nvmc_config(NVMC_CONFIG_WEN_Ren)) != SWD_ERROR_OK)
        return ret;

    /* Set autoincrement on TAR */
    hw_BtfuDapWriteAP(AP_CSW, AP_CSW_DEFAULT | AP_CSW_AUTO_INCREMENT);

    /* Initialize the TAR unless it is on a wrap boundary if so it will be done in the loop */
    if ((addr & NRF51_TAR_WRAP) != 0) {
        hw_BtfuDapWriteAP(AP_TAR, addr);
        /* Do one dummy read. Subsequent reads will return the correct result. */
        hw_BtfuDapReadAP(AP_DRW, &value);
    }

    do {
        /* TAR must be initialized at every TAR wrap boundary
         * because the autoincrement wraps around at these */
        if ((addr & NRF51_TAR_WRAP) == 0) {
            hw_BtfuDapWriteAP(AP_TAR, addr);
            /* Do one dummy read. Subsequent reads will return the correct result. */
            hw_BtfuDapReadAP(AP_DRW, &value);
        }

        /* Read the value from addr */
        hw_BtfuDapReadAP(AP_DRW, &value);

        /* Stack the read value */
        fw_image[buff_offset] = value;

        len -= 4;
        addr += 4;
        buff_offset++;
    } while (len > 0);

    /* Disable autoincrement on TAR */
    hw_BtfuDapWriteAP(AP_CSW, AP_CSW_DEFAULT);

    return SWD_ERROR_OK;
}

 /*! \fn       uint8_t swd_load_image(uint32_t addr, const uint32_t *fw_image, uint32_t len)
 *
 *  \brief     Writes a firmware segment to Nordic internal flash using
 *             direct writes to the NVMC registers.
 *
 * \param      Start address in Nordic flash for the write
 *                       Writes are always 32bit writes so the address
 *                       should always be 4 byte aligned.
 * \param     addr          : Pointer the the data to write
 * \param     fw_image      : Length in bytes of the data to write           -If length is not 4 byte aligned it will be extended
 *                                                                            to 4 bytes and the extended bytes will contain
 *                                                                            random garbage from the buffer.
 * \return    SWD_ERROR_OK if success, or an error code otherwise.
 */
uint8_t swd_load_image(uint32_t addr, const uint32_t *fw_image, uint32_t len)
{
    uint32_t mscStatus;
    uint8_t timeOut;
    btfu_dap_error_t ret;

    if ((addr & 0x3) != 0 || fw_image == NULL || len == 0) {
        return SWD_ERROR_FLASH_WRITE_FAILED;
    }
    //extend length to multiple of 4
    if ((len & 0x3) != 0) {
        len = len + (4 - (len & 0x3));
    }

    // Enable writes
    if ((ret = write_nvmc_config(NVMC_CONFIG_WEN_Wen)) != SWD_ERROR_OK)
        return ret;

    // Write loop
    do {
        // Write the data
        hw_BtfuDapWriteMem(addr, *fw_image);
        // Wait until write complete - max 43usec according to data sheet
        timeOut = NVMC_WRITE_TIMEOUT;
        do {
            mscStatus = hw_BtfuDapReadMem((uint32_t) & (NRF_NVMC->READY));
            timeOut--;
        } while (mscStatus == NVMC_READY_READY_Busy && timeOut > 0);

        if (mscStatus == NVMC_READY_READY_Busy) {
            return SWD_ERROR_FLASH_WRITE_FAILED;
        }

        len -= 4;
        addr += 4;
        fw_image++;  //4 byte increment.
    } while (len > 0);

    // Write of the segment suceeded.
    return SWD_ERROR_OK;
}

uint8_t swd_erase_all(void)
{
    uint32_t mscStatus;
    uint8_t timeOut;
    btfu_dap_error_t ret;

    // Enable erase
    if ((ret = write_nvmc_config(NVMC_CONFIG_WEN_Een)) != SWD_ERROR_OK)
        return ret;

    // Start erase
    hw_BtfuDapWriteMem((uint32_t) & (NRF_NVMC->ERASEALL),
                       NVMC_ERASEALL_ERASEALL_Erase);
    // Wait until erase is complete - typical 21ms according to datasheet
    timeOut = NVMC_ERASE_TIMEOUT;
    do {
        mdelay(1);
        mscStatus = hw_BtfuDapReadMem((uint32_t) & (NRF_NVMC->READY));
        timeOut--;
    } while (mscStatus == NVMC_READY_READY_Busy && timeOut > 0);

    if (mscStatus == NVMC_READY_READY_Busy) {
        return SWD_ERROR_DEVICE_ERASE_FAILED;
    }

#if 1
    // Enable read-only mode
    if ((ret = write_nvmc_config(NVMC_CONFIG_WEN_Ren)) != SWD_ERROR_OK)
        return ret;
#endif

    // Erase completed now verify erase of each section
    if ((hw_BtfuDapReadMem(0x0) != 0xFFFFFFFF)
            || (hw_BtfuDapReadMem(NRF_REGION0_LEN) != 0xFFFFFFFF)
            || (hw_BtfuDapReadMem((uint32_t) & NRF_UICR->CLENR0) != 0xFFFFFFFF)) {
        return SWD_ERROR_DEVICE_ERASE_FAILED;
    }

    // Erase successful
    return SWD_ERROR_OK;
}

uint8_t swd_page_erase(uint8_t page_num)
{
    uint32_t p_page = NRF_PAGESIZE * page_num;
    uint32_t mscStatus;
    uint8_t timeOut;
    btfu_dap_error_t ret;

    // Turn on flash erase enable and wait until the NVMC is ready.

    // Enable erase
    if ((ret = write_nvmc_config(NVMC_CONFIG_WEN_Een)) != SWD_ERROR_OK)
         return ret;

    // Start erase
       hw_BtfuDapWriteMem((uint32_t)&(NRF_NVMC->ERASEPAGE),p_page) ;

   // Wait until erase is complete - typical 21ms according to datasheet
     timeOut = NVMC_ERASE_TIMEOUT;
     do
     {
        mdelay(1);
        mscStatus = hw_BtfuDapReadMem((uint32_t)&(NRF_NVMC->READY));
        timeOut--;
     } while (mscStatus == NVMC_READY_READY_Busy && timeOut > 0);

//    // Turn off flash erase enable and wait until the NVMC is ready.
//    if ((ret = hw_BtfuWriteNvmcConfig(~NVMC_CONFIG_WEN_Een)) != SWD_ERROR_OK)
//            return ret;

    // Erase successful
    return SWD_ERROR_OK;
}


uint8_t swd_verify_image(uint32_t addr, const uint32_t *fw_image, uint32_t len)
{
    uint32_t value;
    btfu_dap_error_t ret;

    if ((addr & 0x3) != 0 || fw_image == NULL || len == 0) {
        return SWD_ERROR_FLASH_WRITE_FAILED;
    }
    //extend length to multiple of 4
    if ((len & 0x3) != 0) {
        len = len + (4 - (len & 0x3));
    }

    // Enable read-only mode
    if ((ret = write_nvmc_config(NVMC_CONFIG_WEN_Ren)) != SWD_ERROR_OK)
        return ret;

    /* Set autoincrement on TAR */
    hw_BtfuDapWriteAP(AP_CSW, AP_CSW_DEFAULT | AP_CSW_AUTO_INCREMENT);

    /* Initialize the TAR unless it is on a wrap boundary if so it will be done in the loop */
    if ((addr & NRF51_TAR_WRAP) != 0) {
        hw_BtfuDapWriteAP(AP_TAR, addr);
        /* Do one dummy read. Subsequent reads will return the correct result. */
        hw_BtfuDapReadAP(AP_DRW, &value);
    }

    do {
        /* TAR must be initialized at every TAR wrap boundary
         * because the autoincrement wraps around at these */
        if ((addr & NRF51_TAR_WRAP) == 0) {
            hw_BtfuDapWriteAP(AP_TAR, addr);
            /* Do one dummy read. Subsequent reads will return the correct result. */
            hw_BtfuDapReadAP(AP_DRW, &value);
        }

        /* Read the value from addr */
        hw_BtfuDapReadAP(AP_DRW, &value);

        if (value != *fw_image) {
//            pr_warning(LOG_MODULE_BL, "RN_BTFU_VERIFY - Fail at %x: %x not %x",
//                       addr, value, *fw_image);
            /* Disable autoincrement on TAR */
            hw_BtfuDapWriteAP(AP_CSW, AP_CSW_DEFAULT);
            return SWD_ERROR_FLASH_WRITE_FAILED;
        }
        len -= 4;
        addr += 4;
        fw_image++;  //4 byte increment.
    } while (len > 0);

    /* Disable autoincrement on TAR */
    hw_BtfuDapWriteAP(AP_CSW, AP_CSW_DEFAULT);

    return SWD_ERROR_OK;
}

uint8_t swd_flash_erase_region1_pages(uint32_t size)
{
   uint32_t mscStatus;
   uint8_t timeOut;

   /* Recover the flash information */
   uint32_t pagesize = hw_BtfuDapReadMem((uint32_t)&(NRF_FICR->CODEPAGESIZE));
   uint32_t memsize =  hw_BtfuDapReadMem((uint32_t)&(NRF_FICR->CODESIZE));
   uint32_t addr = NRF_REGION0_LEN;

   /* This is a known value lets just check it and then use the define afterward */
   if (pagesize != NRF_PAGESIZE)
   {
      return SWD_ERROR_DEVICE_ERASE_FAILED;
   }

   /* Round down the size to the page boundary */
   if ((size & (~(NRF_PAGESIZE - 1))) == 0)
   {  //if it was a page boundary then we actually meant 1 less page.
      size -= NRF_PAGESIZE;
   }
   else
   {
      size = (size & (~(NRF_PAGESIZE - 1)));
   }

   size = addr + size;  //now use size to mean the start address of last page to erase.

   memsize = memsize * NRF_PAGESIZE;
   if ( size >= memsize ) //oops tried to erase past the end of flash
   {
      return SWD_ERROR_DEVICE_ERASE_FAILED;
   }

   /* Enough of that lets do it */
   btfu_dap_error_t ret;
   // Enable erase
   if ((ret = write_nvmc_config(NVMC_CONFIG_WEN_Een)) != SWD_ERROR_OK)
        return ret;

   do
   {
      // Start erase
      hw_BtfuDapWriteMem((uint32_t)&(NRF_NVMC->ERASEPAGE), addr);
      // Wait until erase is complete - typical 21ms according to datasheet
      timeOut = NVMC_ERASE_TIMEOUT;
      do
      {
         mdelay(1);
         mscStatus = hw_BtfuDapReadMem((uint32_t)&(NRF_NVMC->READY));
         timeOut--;
      } while (mscStatus == NVMC_READY_READY_Busy && timeOut > 0);

      if (mscStatus == NVMC_READY_READY_Busy)
      {
         return SWD_ERROR_DEVICE_ERASE_FAILED;
      }
      // Erase completed now verify erase of each section
      if ( (hw_BtfuDapReadMem(addr) != 0xFFFFFFFF) )
      {
         return SWD_ERROR_DEVICE_ERASE_FAILED;
      }
      // Next page
      addr += NRF_PAGESIZE;

   } while (addr <= size);
   //note this is <= size because we have already rounded down to the page start above.

#if 1
   // Enable read-only mode
   if ((ret = write_nvmc_config(NVMC_CONFIG_WEN_Ren)) != SWD_ERROR_OK)
        return ret;
#endif

   // Erase successful
   return SWD_ERROR_OK;
}


