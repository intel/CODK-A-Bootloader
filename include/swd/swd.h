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

#ifndef SWD_H_
#define SWD_H_

#include "utils.h"
#include "hw_btfu_dap.h"
#include "nrf51.h"

#define NVMC_MODE_TIMEOUT  50          // Number of times to wait for NVMC mode enable operation to complete
#define NVMC_WRITE_TIMEOUT 100         // Number of times to wait for NVMC write or mode enable operation to complete
#define NVMC_ERASE_TIMEOUT 200         // Number of times to wait for NVMC erase operation to complete
#define NRF_REGION0_LEN    0x00016000  // Region0 length
#define NRF_PAGESIZE       0x400       // The flash page size of the Nordic

#define CoreDebug_BASE      (0xE000EDF0UL)
#define SCS_BASE            (0xE000E000UL)
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address*/
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address   */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)

/* System control block fields */

#define SYSTEM_CTRL_BLOCK_VECTKEY_BIT             16
#define SYSTEM_CTRL_BLOCK_VECTKEY_MASK            (0xFFFFUL << SYSTEM_CTRL_BLOCK_VECTKEY_BIT)


#define SYSTEM_CTRL_BLOCK_VECT_CLR_ACTIVE_BIT     1
#define SYSTEM_CTRL_BLOCK_VECT_CLR_ACTIVE_MASK    (1UL << SYSTEM_CTRL_BLOCK_VECT_CLR_ACTIVE_BIT)

#define SYSTEM_CTRL_BLOCK_SYS_RESET_BIT           2
#define SYSTEM_CTRL_BLOCK_SYS_RESET_MASK          (1UL << SYSTEM_CTRL_BLOCK_SYS_RESET_BIT)

/* Debug Halting Control and Status Register */

#define CoreDebug_DHCSR_S_RESET_ST_Pos     25                                             /*!< CoreDebug DHCSR: S_RESET_ST Position */
#define CoreDebug_DHCSR_S_RESET_ST_Msk     (1UL << CoreDebug_DHCSR_S_RESET_ST_Pos)        /*!< CoreDebug DHCSR: S_RESET_ST Mask */

#define CoreDebug_DHCSR_S_HALT_Pos         17                                             /*!< CoreDebug DHCSR: S_HALT Position */
#define CoreDebug_DHCSR_S_HALT_Msk         (1UL << CoreDebug_DHCSR_S_HALT_Pos)            /*!< CoreDebug DHCSR: S_HALT Mask */

#define CoreDebug_DEMCR_VC_CORERESET_Pos    0                                             /*!< CoreDebug DEMCR: VC_CORERESET Position */
#define CoreDebug_DEMCR_VC_CORERESET_Msk   (1UL << CoreDebug_DEMCR_VC_CORERESET_Pos)      /*!< CoreDebug DEMCR: VC_CORERESET Mask */


struct coredebug_struct
{
  uint32_t DHCSR;                   /*!< Offset: 0x000 (R/W)  Debug Halting Control and Status Register    */
  uint32_t DCRSR;                   /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register        */
  uint32_t DCRDR;                   /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register            */
  uint32_t DEMCR;                   /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register */
};

struct scb_struct
{
  uint32_t CPUID;                   /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
  uint32_t ICSR;                    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
  uint32_t RESERVED0;
  uint32_t AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
  uint32_t SCR;                     /*!< Offset: 0x010 (R/W)  System Control Register                               */
  uint32_t CCR;                     /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
  uint32_t RESERVED1;
  uint32_t SHP[2];                  /*!< Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED   */
  uint32_t SHCSR;                   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
};

/*! \fn     uint8_t swd_connect_to_target(void)
*
* \brief    Performs the initialization sequence on the SW-DP.
*           After this completes the debug interface can be used.
*
* @return   SWD_ERROR_OK if success, or an error code otherwise
*/
extern uint8_t swd_connect_to_target(void);

/*! \fn     uint8_t swd_unlock_target(void)
*
* \brief    Performs unlocking and some checks of the Nordic
*           so that flashing/writing can proceed.
*
* @return   SWD_ERROR_OK if success, or an error code otherwise
*/
extern uint8_t swd_unlock_target(void);

/*! \fn     uint8_t swd_soft_reset_target(void);
*
* \brief    Resets the target CPU by using the AIRCR register.
*           Does not reset the debug interface
*
* @return   SWD_ERROR_OK if success, or an error code otherwise
*/
extern uint8_t swd_soft_reset_target(void);

/*! \fn      uint8_t swd_debug_mode_reset_to_normal(void)
*
* \brief     Configures a target in debug mode to accept a special pin reset
*
* @return    SWD_ERROR_OK if success, or an error code otherwise
*/
extern uint8_t swd_debug_mode_reset_to_normal(void);

/*! \fn      uint8_t swd_halt_target(void);
*
* \brief     Halts the target CPU
*
* @return    SWD_ERROR_OK if success, or an error code otherwise
*/
extern uint8_t swd_halt_target(void);


/*! \fn      void swd_run_target(void)
*
* \brief     Lets the target CPU run freely (stops halting)
*
* @return    SWD_ERROR_OK if success, or an error code otherwise
*/
extern void swd_run_target(void);

/*! \fn      void swd_step_target(void)
*
* \brief     Single steps the target
*
* @return    SWD_ERROR_OK if success, or an error code otherwise
*/
extern void swd_step_target(void);

/*! \fn      uint8_t swd_init(void)
*
* \brief    Initialize the SWD port
*           connect to the target
*           and unlock the target
*
* @return   SWD_ERROR_OK if success, or an error code otherwise
*/
extern uint8_t swd_init(void);

/*!\fn       uint8_t swd_dump_image(uint32_t addr, uint32_t *fw_image, uint32_t len)
*
*  \brief    Direct memory reads in autoincrement mode.
*
*  \param    addr      Start address in Nordic flash for the read
*  \param    fw_image  Pointer to the output buffer
*  \param    len       Length in bytes of the data to read
*
*  \return  SWD_ERROR_OK if success, or an error code otherwise.
*/
extern uint8_t swd_dump_image(uint32_t addr, uint32_t *fw_image, uint32_t len);

/*! \fn      uint8_t swd_load_image(uint32_t addr, const uint32_t *fw_image, uint32_t len)
*
*  \brief    Writes a firmware segment to Nordic internal flash using
*            direct writes to the NVMC registers.
*
* \param     addr      Start address in Nordic flash for the write
* \param     fw_image  Pointer the the data to write
* \param     len       Length in bytes of the data to write
*
* \return    SWD_ERROR_OK if success, or an error code otherwise.
*/
extern uint8_t swd_load_image(uint32_t addr, const uint32_t *fw_image, uint32_t len);

/*! \fn      uint8_t swd_erase_all(void)
*
* \brief    Erases the entire flash (faster) including the UICR area.
*           This is the fastest way when needing to reflash the SoftDevice
*           as we need to erase Region 0 and UICR and most likely the
*           Application in Region 1 also.
*
* \return   SWD_ERROR_OK if success, or an error code otherwise.
*/
extern uint8_t swd_erase_all(void);

/*! \fn     uint8_t swd_flash_erase_region1_pages(uint32_t size)
*
* \brief    Erases flash pages of Region1 for the given size
*           This is used when we only have to reflash the BT App and
*           not the softdevice, so starting at NRF_REGION0_LEN.
*
* \param    size      the size of the space to erase
*
* \return   SWD_ERROR_OK if success, or an error code otherwise.
*/
extern uint8_t swd_erase_region1_pages(uint32_t size);

/**! \fn     void swd_page_erase(uint8_t page_num);
 * \brief    Erases a page in flash.
 *
 * \param    page_num    Number of page to be erased.
 */
extern uint8_t swd_page_erase(uint8_t page_num);

/*! \fn      uint8_t swd_verify_image(uint32_t addr, const uint32_t *fw_image, uint32_t len)
*
*  \brief    Function to verify a firmware segment against Nordic internal flash using
*            direct memory reads in autoincrement mode.
*
* \param     addr      Start address in Nordic flash for the verify
* \param     fw_image  Pointer the the sourcedata that was written
* \param     len       Length in bytes of the data to write
*
* \return    SWD_ERROR_OK if success, or an error code otherwise.
*/
extern uint8_t swd_verify_image(uint32_t addr, const uint32_t *fw_image, uint32_t len);

#endif /* SWD_H_ */
