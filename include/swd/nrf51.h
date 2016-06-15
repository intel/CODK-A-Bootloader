#ifndef _NRF51_H_
#define _NRF51_H_
/****************************************************************************************************//**
 * @file     Source is from nrf51.h and nrf51bitfields.h
 *
 * @brief    CMSIS Cortex-M0 Peripheral Access Layer Header File for
 *           nRF51 from Nordic Semiconductor.
 *
 * @version  V522
 * @date     4. March 2014
 *
 * @note     Generated with SVDConv V2.77p
 *           from CMSIS SVD File 'nRF51.xml' Version 522,
 *
 * @par      Copyright (c) 2013, Nordic Semiconductor ASA
 *           All rights reserved.
 *
 *           Redistribution and use in source and binary forms, with or without
 *           modification, are permitted provided that the following conditions are met:
 *
 *           * Redistributions of source code must retain the above copyright notice, this
 *           list of conditions and the following disclaimer.
 *
 *           * Redistributions in binary form must reproduce the above copyright notice,
 *           this list of conditions and the following disclaimer in the documentation
 *           and/or other materials provided with the distribution.
 *
 *           * Neither the name of Nordic Semiconductor ASA nor the names of its
 *           contributors may be used to endorse or promote products derived from
 *           this software without specific prior written permission.
 *
 *           THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *           AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *           IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *           DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *           FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *           DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *           SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *           CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *           OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *           OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *******************************************************************************************************
 * @par      Partial excerpt of the original file used by Basis Science 2014
 *******************************************************************************************************/

/* ================================================================================ */
/* ================                      POWER                     ================ */
/* ================================================================================ */

/* Register: POWER_RESET */
/* Description: Pin reset functionality configuration register. This register is a retained register. */

/* Bit 0 : Enable pin reset in debug interface mode. */
#define POWER_RESET_RESET_Pos (0UL) /*!< Position of RESET field. */
//#define POWER_RESET_RESET_Msk (0x1UL << POWER_RESET_RESET_Pos) /*!< Bit mask of RESET field. */
//#define POWER_RESET_RESET_Disabled (0UL) /*!< Pin reset in debug interface mode disabled. */
#define POWER_RESET_RESET_Enabled (1UL) /*!< Pin reset in debug interface mode enabled. */

/**
  * @brief Power Control. (POWER)
  */

typedef struct {                                    /*!< POWER Structure                                                       */
   uint32_t  RESERVED0[30];
   uint32_t  TASKS_CONSTLAT;                    /*!< Enable constant latency mode.                                         */
   uint32_t  TASKS_LOWPWR;                      /*!< Enable low power mode (variable latency).                             */
   uint32_t  RESERVED1[34];
   uint32_t  EVENTS_POFWARN;                    /*!< Power failure warning.                                                */
   uint32_t  RESERVED2[126];
   uint32_t  INTENSET;                          /*!< Interrupt enable set register.                                        */
   uint32_t  INTENCLR;                          /*!< Interrupt enable clear register.                                      */
   uint32_t  RESERVED3[61];
   uint32_t  RESETREAS;                         /*!< Reset reason.                                                         */
   uint32_t  RESERVED4[63];
   uint32_t  SYSTEMOFF;                         /*!< System off register.                                                  */
   uint32_t  RESERVED5[3];
   uint32_t  POFCON;                            /*!< Power failure configuration.                                          */
   uint32_t  RESERVED6[2];
   uint32_t  GPREGRET;                          /*!< General purpose retention register. This register is a retained
                                                         register.                                                             */
   uint32_t  RESERVED7;
   uint32_t  RAMON;                             /*!< Ram on/off.                                                           */
   uint32_t  RESERVED8[7];
   uint32_t  RESET;                             /*!< Pin reset functionality configuration register. This register
                                                         is a retained register.                                               */
   uint32_t  RESERVED9[12];
   uint32_t  DCDCEN;                            /*!< DCDC converter enable configuration register.                         */
} NRF_POWER_Type;


/* ================================================================================ */
/* ================                       MPU                      ================ */
/* ================================================================================ */

/* Register: MPU_DISABLEINDEBUG */
/* Description: Disable protection mechanism in debug mode. */

/* Bit 0 : Disable protection mechanism in debug mode. */
#define MPU_DISABLEINDEBUG_DISABLEINDEBUG_Pos (0UL) /*!< Position of DISABLEINDEBUG field. */
//#define MPU_DISABLEINDEBUG_DISABLEINDEBUG_Msk (0x1UL << MPU_DISABLEINDEBUG_DISABLEINDEBUG_Pos) /*!< Bit mask of DISABLEINDEBUG field. */
//#define MPU_DISABLEINDEBUG_DISABLEINDEBUG_Enabled (0UL) /*!< Protection enabled. */
#define MPU_DISABLEINDEBUG_DISABLEINDEBUG_Disabled (1UL) /*!< Protection disabled. */

/**
  * @brief Memory Protection Unit. (MPU)
  */

typedef struct {                                    /*!< MPU Structure                                                         */
   uint32_t  RESERVED0[330];
   uint32_t  PERR0;                             /*!< Configuration of peripherals in mpu regions.                          */
   uint32_t  RLENR0;                            /*!< Length of RAM region 0.                                               */
   uint32_t  RESERVED1[52];
   uint32_t  PROTENSET0;                        /*!< Erase and write protection bit enable set register.                   */
   uint32_t  PROTENSET1;                        /*!< Erase and write protection bit enable set register.                   */
   uint32_t  DISABLEINDEBUG;                    /*!< Disable erase and write protection mechanism in debug mode.           */
   uint32_t  PROTBLOCKSIZE;                     /*!< Erase and write protection block size.                                */
   uint32_t  RESERVED2[255];
   uint32_t  ENRBDREG;                          /*!< Enable or disable RBD.                                                */
} NRF_MPU_Type;


/* ================================================================================ */
/* ================                      NVMC                      ================ */
/* ================================================================================ */

/* Peripheral: NVMC */
/* Description: Non Volatile Memory Controller. */

/* Register: NVMC_READY */
/* Description: Ready flag. */

/* Bit 0 : NVMC ready. */
//#define NVMC_READY_READY_Pos (0UL) /*!< Position of READY field. */
//#define NVMC_READY_READY_Msk (0x1UL << NVMC_READY_READY_Pos) /*!< Bit mask of READY field. */
#define NVMC_READY_READY_Busy (0UL) /*!< NVMC is busy (on-going write or erase operation). */
//#define NVMC_READY_READY_Ready (1UL) /*!< NVMC is ready. */

/* Register: NVMC_CONFIG */
/* Description: Configuration register. */

/* Bits 1..0 : Program write enable. */
#define NVMC_CONFIG_WEN_Pos (0UL) /*!< Position of WEN field. */
//#define NVMC_CONFIG_WEN_Msk (0x3UL << NVMC_CONFIG_WEN_Pos) /*!< Bit mask of WEN field. */
#define NVMC_CONFIG_WEN_Ren (0x00UL) /*!< Read only access. */
#define NVMC_CONFIG_WEN_Wen (0x01UL) /*!< Write enabled. */
#define NVMC_CONFIG_WEN_Een (0x02UL) /*!< Erase enabled. */

/* Register: NVMC_ERASEALL */
/* Description: Register for erasing all non-volatile user memory. */

/* Bit 0 : Starts the erasing of all user NVM (code region 0/1 and UICR registers). */
//#define NVMC_ERASEALL_ERASEALL_Pos (0UL) /*!< Position of ERASEALL field. */
//#define NVMC_ERASEALL_ERASEALL_Msk (0x1UL << NVMC_ERASEALL_ERASEALL_Pos) /*!< Bit mask of ERASEALL field. */
//#define NVMC_ERASEALL_ERASEALL_NoOperation (0UL) /*!< No operation. */
#define NVMC_ERASEALL_ERASEALL_Erase (1UL) /*!< Start chip erase. */

/* Register: NVMC_ERASEUICR */
/* Description: Register for start erasing User Information Congfiguration Registers. */

/* Bit 0 : It can only be used when all contents of code region 1 are erased. */
//#define NVMC_ERASEUICR_ERASEUICR_Pos (0UL) /*!< Position of ERASEUICR field. */
//#define NVMC_ERASEUICR_ERASEUICR_Msk (0x1UL << NVMC_ERASEUICR_ERASEUICR_Pos) /*!< Bit mask of ERASEUICR field. */
//#define NVMC_ERASEUICR_ERASEUICR_NoOperation (0UL) /*!< No operation. */
//#define NVMC_ERASEUICR_ERASEUICR_Erase (1UL) /*!< Start UICR erase. */


/**
  * @brief Non Volatile Memory Controller. (NVMC)
  */

typedef struct {                                    /*!< NVMC Structure                                                        */
   uint32_t  RESERVED0[256];
   uint32_t  READY;                             /*!< Ready flag.                                                           */
   uint32_t  RESERVED1[64];
   uint32_t  CONFIG;                            /*!< Configuration register.                                               */
   uint32_t  ERASEPAGE;                         /*!< Register for erasing a non-protected non-volatile memory page.        */
   uint32_t  ERASEALL;                          /*!< Register for erasing all non-volatile user memory.                    */
   uint32_t  ERASEPROTECTEDPAGE;                /*!< Register for erasing a protected non-volatile memory page.            */
   uint32_t  ERASEUICR;                         /*!< Register for start erasing User Information Congfiguration Registers. */
} NRF_NVMC_Type;


/* ================================================================================ */
/* ================                      FICR                      ================ */
/* ================================================================================ */

/* Peripheral: FICR */
/* Description: Factory Information Configuration. */

/* Register: FICR_PPFC */
/* Description: Pre-programmed factory code present. */

/* Bits 7..0 : Pre-programmed factory code present. */
#define FICR_PPFC_PPFC_Pos (0UL) /*!< Position of PPFC field. */
#define FICR_PPFC_PPFC_Msk (0xFFUL << FICR_PPFC_PPFC_Pos) /*!< Bit mask of PPFC field. */
#define FICR_PPFC_PPFC_NotPresent (0xFFUL) /*!< Not present. */
#define FICR_PPFC_PPFC_Present (0x00UL) /*!< Present. */

/**
  * @brief Factory Information Configuration. (FICR)
  */

typedef struct {                                    /*!< FICR Structure                                                        */
   uint32_t  RESERVED0[4];
   uint32_t  CODEPAGESIZE;                      /*!< Code memory page size in bytes.                                       */
   uint32_t  CODESIZE;                          /*!< Code memory size in pages.                                            */
   uint32_t  RBD;                               /*!< RBD.                                                                  */
   uint32_t  RESERVED1[3];
   uint32_t  CLENR0;                            /*!< Length of code region 0 in bytes.                                     */
   uint32_t  PPFC;                              /*!< Pre-programmed factory code present.                                  */
   uint32_t  RESERVED2;
   uint32_t  NUMRAMBLOCK;                       /*!< Number of individualy controllable RAM blocks.                        */

  union {
     uint32_t  SIZERAMBLOCK[4];                 /*!< Deprecated array of size of RAM block in bytes. This name is
                                                         kept for backward compatinility purposes. Use SIZERAMBLOCKS
                                                          instead.                                                             */
     uint32_t  SIZERAMBLOCKS;                   /*!< Size of RAM blocks in bytes.                                          */
  } ;
   uint32_t  RESERVED3[5];
   uint32_t  CONFIGID;                          /*!< Configuration identifier.                                             */
   uint32_t  DEVICEID[2];                       /*!< Device identifier.                                                    */
   uint32_t  RESERVED4[6];
   uint32_t  ER[4];                             /*!< Encryption root.                                                      */
   uint32_t  IR[4];                             /*!< Identity root.                                                        */
   uint32_t  DEVICEADDRTYPE;                    /*!< Device address type.                                                  */
   uint32_t  DEVICEADDR[2];                     /*!< Device address.                                                       */
   uint32_t  OVERRIDEEN;                        /*!< Radio calibration override enable.                                    */
   uint32_t  NRF_1MBIT[5];                      /*!< Override values for the OVERRIDEn registers in RADIO for NRF_1Mbit
                                                         mode.                                                                 */
   uint32_t  RESERVED5[10];
   uint32_t  BLE_1MBIT[5];                      /*!< Override values for the OVERRIDEn registers in RADIO for BLE_1Mbit
                                                         mode.                                                                 */
} NRF_FICR_Type;

/* ================================================================================ */
/* ================                      UICR                      ================ */
/* ================================================================================ */


/**
  * @brief User Information Configuration. (UICR)
  */

typedef struct {                                    /*!< UICR Structure                                                        */
   uint32_t  CLENR0;                            /*!< Length of code region 0.                                              */
   uint32_t  RBPCONF;                           /*!< Readback protection configuration.                                    */
   uint32_t  XTALFREQ;                          /*!< Reset value for CLOCK XTALFREQ register.                              */
   uint32_t  RESERVED0;
   uint32_t  FWID;                              /*!< Firmware ID.                                                          */
   uint32_t  BOOTLOADERADDR;                    /*!< Bootloader start address.                                             */
} NRF_UICR_Type;


/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */
#define NRF_POWER_BASE                  0x40000000UL
#define NRF_MPU_BASE                    0x40000000UL
#define NRF_NVMC_BASE                   0x4001E000UL
#define NRF_FICR_BASE                   0x10000000UL
#define NRF_UICR_BASE                   0x10001000UL

/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */
#define NRF_POWER                       ((NRF_POWER_Type          *) NRF_POWER_BASE)
#define NRF_MPU                         ((NRF_MPU_Type            *) NRF_MPU_BASE)
#define NRF_NVMC                        ((NRF_NVMC_Type           *) NRF_NVMC_BASE)
#define NRF_FICR                        ((NRF_FICR_Type           *) NRF_FICR_BASE)
#define NRF_UICR                        ((NRF_UICR_Type           *) NRF_UICR_BASE)


#endif
