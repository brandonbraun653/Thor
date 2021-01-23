/********************************************************************************
 *  File Name:
 *    hw_power_register_stm32l432kc.hpp
 *
 *  Description:
 *    POWER register definitions for the STM32L432KC series chips.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_POWER_REGISTER_STM32L432KC_HPP
#define THOR_HW_POWER_REGISTER_STM32L432KC_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>

/*-------------------------------------------------
Peripheral Availability
-------------------------------------------------*/
#define STM32_PWR_PERIPH_AVAILABLE

namespace Thor::LLD::PWR
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr Reg32_t PWR_BASE_ADDR = Thor::System::MemoryMap::PWR_PERIPH_START_ADDRESS;

  /*-------------------------------------------------
  Peripheral Resource Lookup Indices
  -------------------------------------------------*/
  static constexpr uint32_t PWR_RESOURCE_INDEX = 0u;

  /*-------------------------------------------------
  Lookup addresses
  -------------------------------------------------*/
  static constexpr size_t NUM_PWR_PERIPHS = 1;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/

  /********************  Bit definition for CR1 register  ********************/
  static constexpr Reg32_t CR1_LPR_Pos           = ( 14U );
  static constexpr Reg32_t CR1_LPR_Msk           = ( 0x1UL << CR1_LPR_Pos );
  static constexpr Reg32_t CR1_LPR               = CR1_LPR_Msk;
  static constexpr Reg32_t CR1_VOS_Pos           = ( 9U );
  static constexpr Reg32_t CR1_VOS_Msk           = ( 0x3UL << CR1_VOS_Pos );
  static constexpr Reg32_t CR1_VOS               = CR1_VOS_Msk;
  static constexpr Reg32_t CR1_VOS_0             = ( 0x1UL << CR1_VOS_Pos );
  static constexpr Reg32_t CR1_VOS_1             = ( 0x2UL << CR1_VOS_Pos );
  static constexpr Reg32_t CR1_DBP_Pos           = ( 8U );
  static constexpr Reg32_t CR1_DBP_Msk           = ( 0x1UL << CR1_DBP_Pos );
  static constexpr Reg32_t CR1_DBP               = CR1_DBP_Msk;
  static constexpr Reg32_t CR1_LPMS_Pos          = ( 0U );
  static constexpr Reg32_t CR1_LPMS_Msk          = ( 0x7UL << CR1_LPMS_Pos );
  static constexpr Reg32_t CR1_LPMS              = CR1_LPMS_Msk;
  static constexpr Reg32_t CR1_LPMS_STOP0        = ( 0x00000000UL );
  static constexpr Reg32_t CR1_LPMS_STOP1_Pos    = ( 0U );
  static constexpr Reg32_t CR1_LPMS_STOP1_Msk    = ( 0x1UL << CR1_LPMS_STOP1_Pos );
  static constexpr Reg32_t CR1_LPMS_STOP1        = CR1_LPMS_STOP1_Msk;
  static constexpr Reg32_t CR1_LPMS_STOP2_Pos    = ( 1U );
  static constexpr Reg32_t CR1_LPMS_STOP2_Msk    = ( 0x1UL << CR1_LPMS_STOP2_Pos );
  static constexpr Reg32_t CR1_LPMS_STOP2        = CR1_LPMS_STOP2_Msk;
  static constexpr Reg32_t CR1_LPMS_STANDBY_Pos  = ( 0U );
  static constexpr Reg32_t CR1_LPMS_STANDBY_Msk  = ( 0x3UL << CR1_LPMS_STANDBY_Pos );
  static constexpr Reg32_t CR1_LPMS_STANDBY      = CR1_LPMS_STANDBY_Msk;
  static constexpr Reg32_t CR1_LPMS_SHUTDOWN_Pos = ( 2U );
  static constexpr Reg32_t CR1_LPMS_SHUTDOWN_Msk = ( 0x1UL << CR1_LPMS_SHUTDOWN_Pos );
  static constexpr Reg32_t CR1_LPMS_SHUTDOWN     = CR1_LPMS_SHUTDOWN_Msk;

  /********************  Bit definition for CR2 register  ********************/
  static constexpr Reg32_t CR2_USV_Pos = ( 10U );
  static constexpr Reg32_t CR2_USV_Msk = ( 0x1UL << CR2_USV_Pos );
  static constexpr Reg32_t CR2_USV     = CR2_USV_Msk;

  static constexpr Reg32_t CR2_PVME_Pos  = ( 4U );
  static constexpr Reg32_t CR2_PVME_Msk  = ( 0xDUL << CR2_PVME_Pos );
  static constexpr Reg32_t CR2_PVME      = CR2_PVME_Msk;
  static constexpr Reg32_t CR2_PVME4_Pos = ( 7U );
  static constexpr Reg32_t CR2_PVME4_Msk = ( 0x1UL << CR2_PVME4_Pos );
  static constexpr Reg32_t CR2_PVME4     = CR2_PVME4_Msk;
  static constexpr Reg32_t CR2_PVME3_Pos = ( 6U );
  static constexpr Reg32_t CR2_PVME3_Msk = ( 0x1UL << CR2_PVME3_Pos );
  static constexpr Reg32_t CR2_PVME3     = CR2_PVME3_Msk;
  static constexpr Reg32_t CR2_PVME1_Pos = ( 4U );
  static constexpr Reg32_t CR2_PVME1_Msk = ( 0x1UL << CR2_PVME1_Pos );
  static constexpr Reg32_t CR2_PVME1     = CR2_PVME1_Msk;

  static constexpr Reg32_t CR2_PLS_Pos      = ( 1U );
  static constexpr Reg32_t CR2_PLS_Msk      = ( 0x7UL << CR2_PLS_Pos );
  static constexpr Reg32_t CR2_PLS          = CR2_PLS_Msk;
  static constexpr Reg32_t CR2_PLS_LEV0     = ( 0x00000000UL );
  static constexpr Reg32_t CR2_PLS_LEV1_Pos = ( 1U );
  static constexpr Reg32_t CR2_PLS_LEV1_Msk = ( 0x1UL << CR2_PLS_LEV1_Pos );
  static constexpr Reg32_t CR2_PLS_LEV1     = CR2_PLS_LEV1_Msk;
  static constexpr Reg32_t CR2_PLS_LEV2_Pos = ( 2U );
  static constexpr Reg32_t CR2_PLS_LEV2_Msk = ( 0x1UL << CR2_PLS_LEV2_Pos );
  static constexpr Reg32_t CR2_PLS_LEV2     = CR2_PLS_LEV2_Msk;
  static constexpr Reg32_t CR2_PLS_LEV3_Pos = ( 1U );
  static constexpr Reg32_t CR2_PLS_LEV3_Msk = ( 0x3UL << CR2_PLS_LEV3_Pos );
  static constexpr Reg32_t CR2_PLS_LEV3     = CR2_PLS_LEV3_Msk;
  static constexpr Reg32_t CR2_PLS_LEV4_Pos = ( 3U );
  static constexpr Reg32_t CR2_PLS_LEV4_Msk = ( 0x1UL << CR2_PLS_LEV4_Pos );
  static constexpr Reg32_t CR2_PLS_LEV4     = CR2_PLS_LEV4_Msk;
  static constexpr Reg32_t CR2_PLS_LEV5_Pos = ( 1U );
  static constexpr Reg32_t CR2_PLS_LEV5_Msk = ( 0x5UL << CR2_PLS_LEV5_Pos );
  static constexpr Reg32_t CR2_PLS_LEV5     = CR2_PLS_LEV5_Msk;
  static constexpr Reg32_t CR2_PLS_LEV6_Pos = ( 2U );
  static constexpr Reg32_t CR2_PLS_LEV6_Msk = ( 0x3UL << CR2_PLS_LEV6_Pos );
  static constexpr Reg32_t CR2_PLS_LEV6     = CR2_PLS_LEV6_Msk;
  static constexpr Reg32_t CR2_PLS_LEV7_Pos = ( 1U );
  static constexpr Reg32_t CR2_PLS_LEV7_Msk = ( 0x7UL << CR2_PLS_LEV7_Pos );
  static constexpr Reg32_t CR2_PLS_LEV7     = CR2_PLS_LEV7_Msk;
  static constexpr Reg32_t CR2_PVDE_Pos     = ( 0U );
  static constexpr Reg32_t CR2_PVDE_Msk     = ( 0x1UL << CR2_PVDE_Pos );
  static constexpr Reg32_t CR2_PVDE         = CR2_PVDE_Msk;

  /********************  Bit definition for CR3 register  ********************/
  static constexpr Reg32_t CR3_EIWUL_Pos = ( 15U );
  static constexpr Reg32_t CR3_EIWUL_Msk = ( 0x1UL << CR3_EIWUL_Pos );
  static constexpr Reg32_t CR3_EIWUL     = CR3_EIWUL_Msk;
  static constexpr Reg32_t CR3_APC_Pos   = ( 10U );
  static constexpr Reg32_t CR3_APC_Msk   = ( 0x1UL << CR3_APC_Pos );
  static constexpr Reg32_t CR3_APC       = CR3_APC_Msk;
  static constexpr Reg32_t CR3_RRS_Pos   = ( 8U );
  static constexpr Reg32_t CR3_RRS_Msk   = ( 0x1UL << CR3_RRS_Pos );
  static constexpr Reg32_t CR3_RRS       = CR3_RRS_Msk;
  static constexpr Reg32_t CR3_EWUP5_Pos = ( 4U );
  static constexpr Reg32_t CR3_EWUP5_Msk = ( 0x1UL << CR3_EWUP5_Pos );
  static constexpr Reg32_t CR3_EWUP5     = CR3_EWUP5_Msk;
  static constexpr Reg32_t CR3_EWUP4_Pos = ( 3U );
  static constexpr Reg32_t CR3_EWUP4_Msk = ( 0x1UL << CR3_EWUP4_Pos );
  static constexpr Reg32_t CR3_EWUP4     = CR3_EWUP4_Msk;
  static constexpr Reg32_t CR3_EWUP3_Pos = ( 2U );
  static constexpr Reg32_t CR3_EWUP3_Msk = ( 0x1UL << CR3_EWUP3_Pos );
  static constexpr Reg32_t CR3_EWUP3     = CR3_EWUP3_Msk;
  static constexpr Reg32_t CR3_EWUP2_Pos = ( 1U );
  static constexpr Reg32_t CR3_EWUP2_Msk = ( 0x1UL << CR3_EWUP2_Pos );
  static constexpr Reg32_t CR3_EWUP2     = CR3_EWUP2_Msk;
  static constexpr Reg32_t CR3_EWUP1_Pos = ( 0U );
  static constexpr Reg32_t CR3_EWUP1_Msk = ( 0x1UL << CR3_EWUP1_Pos );
  static constexpr Reg32_t CR3_EWUP1     = CR3_EWUP1_Msk;
  static constexpr Reg32_t CR3_EWUP_Pos  = ( 0U );
  static constexpr Reg32_t CR3_EWUP_Msk  = ( 0x1FUL << CR3_EWUP_Pos );
  static constexpr Reg32_t CR3_EWUP      = CR3_EWUP_Msk;

  /********************  Bit definition for CR4 register  ********************/
  static constexpr Reg32_t CR4_VBRS_Pos = ( 9U );
  static constexpr Reg32_t CR4_VBRS_Msk = ( 0x1UL << CR4_VBRS_Pos );
  static constexpr Reg32_t CR4_VBRS     = CR4_VBRS_Msk;
  static constexpr Reg32_t CR4_VBE_Pos  = ( 8U );
  static constexpr Reg32_t CR4_VBE_Msk  = ( 0x1UL << CR4_VBE_Pos );
  static constexpr Reg32_t CR4_VBE      = CR4_VBE_Msk;
  static constexpr Reg32_t CR4_WP5_Pos  = ( 4U );
  static constexpr Reg32_t CR4_WP5_Msk  = ( 0x1UL << CR4_WP5_Pos );
  static constexpr Reg32_t CR4_WP5      = CR4_WP5_Msk;
  static constexpr Reg32_t CR4_WP4_Pos  = ( 3U );
  static constexpr Reg32_t CR4_WP4_Msk  = ( 0x1UL << CR4_WP4_Pos );
  static constexpr Reg32_t CR4_WP4      = CR4_WP4_Msk;
  static constexpr Reg32_t CR4_WP3_Pos  = ( 2U );
  static constexpr Reg32_t CR4_WP3_Msk  = ( 0x1UL << CR4_WP3_Pos );
  static constexpr Reg32_t CR4_WP3      = CR4_WP3_Msk;
  static constexpr Reg32_t CR4_WP2_Pos  = ( 1U );
  static constexpr Reg32_t CR4_WP2_Msk  = ( 0x1UL << CR4_WP2_Pos );
  static constexpr Reg32_t CR4_WP2      = CR4_WP2_Msk;
  static constexpr Reg32_t CR4_WP1_Pos  = ( 0U );
  static constexpr Reg32_t CR4_WP1_Msk  = ( 0x1UL << CR4_WP1_Pos );
  static constexpr Reg32_t CR4_WP1      = CR4_WP1_Msk;

  /********************  Bit definition for SR1 register  ********************/
  static constexpr Reg32_t SR1_WUFI_Pos = ( 15U );
  static constexpr Reg32_t SR1_WUFI_Msk = ( 0x1UL << SR1_WUFI_Pos );
  static constexpr Reg32_t SR1_WUFI     = SR1_WUFI_Msk;
  static constexpr Reg32_t SR1_SBF_Pos  = ( 8U );
  static constexpr Reg32_t SR1_SBF_Msk  = ( 0x1UL << SR1_SBF_Pos );
  static constexpr Reg32_t SR1_SBF      = SR1_SBF_Msk;
  static constexpr Reg32_t SR1_WUF_Pos  = ( 0U );
  static constexpr Reg32_t SR1_WUF_Msk  = ( 0x1FUL << SR1_WUF_Pos );
  static constexpr Reg32_t SR1_WUF      = SR1_WUF_Msk;
  static constexpr Reg32_t SR1_WUF5_Pos = ( 4U );
  static constexpr Reg32_t SR1_WUF5_Msk = ( 0x1UL << SR1_WUF5_Pos );
  static constexpr Reg32_t SR1_WUF5     = SR1_WUF5_Msk;
  static constexpr Reg32_t SR1_WUF4_Pos = ( 3U );
  static constexpr Reg32_t SR1_WUF4_Msk = ( 0x1UL << SR1_WUF4_Pos );
  static constexpr Reg32_t SR1_WUF4     = SR1_WUF4_Msk;
  static constexpr Reg32_t SR1_WUF3_Pos = ( 2U );
  static constexpr Reg32_t SR1_WUF3_Msk = ( 0x1UL << SR1_WUF3_Pos );
  static constexpr Reg32_t SR1_WUF3     = SR1_WUF3_Msk;
  static constexpr Reg32_t SR1_WUF2_Pos = ( 1U );
  static constexpr Reg32_t SR1_WUF2_Msk = ( 0x1UL << SR1_WUF2_Pos );
  static constexpr Reg32_t SR1_WUF2     = SR1_WUF2_Msk;
  static constexpr Reg32_t SR1_WUF1_Pos = ( 0U );
  static constexpr Reg32_t SR1_WUF1_Msk = ( 0x1UL << SR1_WUF1_Pos );
  static constexpr Reg32_t SR1_WUF1     = SR1_WUF1_Msk;

  /********************  Bit definition for SR2 register  ********************/
  static constexpr Reg32_t SR2_PVMO4_Pos  = ( 15U );
  static constexpr Reg32_t SR2_PVMO4_Msk  = ( 0x1UL << SR2_PVMO4_Pos );
  static constexpr Reg32_t SR2_PVMO4      = SR2_PVMO4_Msk;
  static constexpr Reg32_t SR2_PVMO3_Pos  = ( 14U );
  static constexpr Reg32_t SR2_PVMO3_Msk  = ( 0x1UL << SR2_PVMO3_Pos );
  static constexpr Reg32_t SR2_PVMO3      = SR2_PVMO3_Msk;
  static constexpr Reg32_t SR2_PVDO_Pos   = ( 11U );
  static constexpr Reg32_t SR2_PVDO_Msk   = ( 0x1UL << SR2_PVDO_Pos );
  static constexpr Reg32_t SR2_PVDO       = SR2_PVDO_Msk;
  static constexpr Reg32_t SR2_VOSF_Pos   = ( 10U );
  static constexpr Reg32_t SR2_VOSF_Msk   = ( 0x1UL << SR2_VOSF_Pos );
  static constexpr Reg32_t SR2_VOSF       = SR2_VOSF_Msk;
  static constexpr Reg32_t SR2_REGLPF_Pos = ( 9U );
  static constexpr Reg32_t SR2_REGLPF_Msk = ( 0x1UL << SR2_REGLPF_Pos );
  static constexpr Reg32_t SR2_REGLPF     = SR2_REGLPF_Msk;
  static constexpr Reg32_t SR2_REGLPS_Pos = ( 8U );
  static constexpr Reg32_t SR2_REGLPS_Msk = ( 0x1UL << SR2_REGLPS_Pos );
  static constexpr Reg32_t SR2_REGLPS     = SR2_REGLPS_Msk;

  /********************  Bit definition for SCR register  ********************/
  static constexpr Reg32_t SCR_CSBF_Pos  = ( 8U );
  static constexpr Reg32_t SCR_CSBF_Msk  = ( 0x1UL << SCR_CSBF_Pos );
  static constexpr Reg32_t SCR_CSBF      = SCR_CSBF_Msk;
  static constexpr Reg32_t SCR_CWUF_Pos  = ( 0U );
  static constexpr Reg32_t SCR_CWUF_Msk  = ( 0x1FUL << SCR_CWUF_Pos );
  static constexpr Reg32_t SCR_CWUF      = SCR_CWUF_Msk;
  static constexpr Reg32_t SCR_CWUF5_Pos = ( 4U );
  static constexpr Reg32_t SCR_CWUF5_Msk = ( 0x1UL << SCR_CWUF5_Pos );
  static constexpr Reg32_t SCR_CWUF5     = SCR_CWUF5_Msk;
  static constexpr Reg32_t SCR_CWUF4_Pos = ( 3U );
  static constexpr Reg32_t SCR_CWUF4_Msk = ( 0x1UL << SCR_CWUF4_Pos );
  static constexpr Reg32_t SCR_CWUF4     = SCR_CWUF4_Msk;
  static constexpr Reg32_t SCR_CWUF3_Pos = ( 2U );
  static constexpr Reg32_t SCR_CWUF3_Msk = ( 0x1UL << SCR_CWUF3_Pos );
  static constexpr Reg32_t SCR_CWUF3     = SCR_CWUF3_Msk;
  static constexpr Reg32_t SCR_CWUF2_Pos = ( 1U );
  static constexpr Reg32_t SCR_CWUF2_Msk = ( 0x1UL << SCR_CWUF2_Pos );
  static constexpr Reg32_t SCR_CWUF2     = SCR_CWUF2_Msk;
  static constexpr Reg32_t SCR_CWUF1_Pos = ( 0U );
  static constexpr Reg32_t SCR_CWUF1_Msk = ( 0x1UL << SCR_CWUF1_Pos );
  static constexpr Reg32_t SCR_CWUF1     = SCR_CWUF1_Msk;

  /********************  Bit definition for PUCRA register  ********************/
  static constexpr Reg32_t PUCRA_PA15_Pos = ( 15U );
  static constexpr Reg32_t PUCRA_PA15_Msk = ( 0x1UL << PUCRA_PA15_Pos );
  static constexpr Reg32_t PUCRA_PA15     = PUCRA_PA15_Msk;
  static constexpr Reg32_t PUCRA_PA13_Pos = ( 13U );
  static constexpr Reg32_t PUCRA_PA13_Msk = ( 0x1UL << PUCRA_PA13_Pos );
  static constexpr Reg32_t PUCRA_PA13     = PUCRA_PA13_Msk;
  static constexpr Reg32_t PUCRA_PA12_Pos = ( 12U );
  static constexpr Reg32_t PUCRA_PA12_Msk = ( 0x1UL << PUCRA_PA12_Pos );
  static constexpr Reg32_t PUCRA_PA12     = PUCRA_PA12_Msk;
  static constexpr Reg32_t PUCRA_PA11_Pos = ( 11U );
  static constexpr Reg32_t PUCRA_PA11_Msk = ( 0x1UL << PUCRA_PA11_Pos );
  static constexpr Reg32_t PUCRA_PA11     = PUCRA_PA11_Msk;
  static constexpr Reg32_t PUCRA_PA10_Pos = ( 10U );
  static constexpr Reg32_t PUCRA_PA10_Msk = ( 0x1UL << PUCRA_PA10_Pos );
  static constexpr Reg32_t PUCRA_PA10     = PUCRA_PA10_Msk;
  static constexpr Reg32_t PUCRA_PA9_Pos  = ( 9U );
  static constexpr Reg32_t PUCRA_PA9_Msk  = ( 0x1UL << PUCRA_PA9_Pos );
  static constexpr Reg32_t PUCRA_PA9      = PUCRA_PA9_Msk;
  static constexpr Reg32_t PUCRA_PA8_Pos  = ( 8U );
  static constexpr Reg32_t PUCRA_PA8_Msk  = ( 0x1UL << PUCRA_PA8_Pos );
  static constexpr Reg32_t PUCRA_PA8      = PUCRA_PA8_Msk;
  static constexpr Reg32_t PUCRA_PA7_Pos  = ( 7U );
  static constexpr Reg32_t PUCRA_PA7_Msk  = ( 0x1UL << PUCRA_PA7_Pos );
  static constexpr Reg32_t PUCRA_PA7      = PUCRA_PA7_Msk;
  static constexpr Reg32_t PUCRA_PA6_Pos  = ( 6U );
  static constexpr Reg32_t PUCRA_PA6_Msk  = ( 0x1UL << PUCRA_PA6_Pos );
  static constexpr Reg32_t PUCRA_PA6      = PUCRA_PA6_Msk;
  static constexpr Reg32_t PUCRA_PA5_Pos  = ( 5U );
  static constexpr Reg32_t PUCRA_PA5_Msk  = ( 0x1UL << PUCRA_PA5_Pos );
  static constexpr Reg32_t PUCRA_PA5      = PUCRA_PA5_Msk;
  static constexpr Reg32_t PUCRA_PA4_Pos  = ( 4U );
  static constexpr Reg32_t PUCRA_PA4_Msk  = ( 0x1UL << PUCRA_PA4_Pos );
  static constexpr Reg32_t PUCRA_PA4      = PUCRA_PA4_Msk;
  static constexpr Reg32_t PUCRA_PA3_Pos  = ( 3U );
  static constexpr Reg32_t PUCRA_PA3_Msk  = ( 0x1UL << PUCRA_PA3_Pos );
  static constexpr Reg32_t PUCRA_PA3      = PUCRA_PA3_Msk;
  static constexpr Reg32_t PUCRA_PA2_Pos  = ( 2U );
  static constexpr Reg32_t PUCRA_PA2_Msk  = ( 0x1UL << PUCRA_PA2_Pos );
  static constexpr Reg32_t PUCRA_PA2      = PUCRA_PA2_Msk;
  static constexpr Reg32_t PUCRA_PA1_Pos  = ( 1U );
  static constexpr Reg32_t PUCRA_PA1_Msk  = ( 0x1UL << PUCRA_PA1_Pos );
  static constexpr Reg32_t PUCRA_PA1      = PUCRA_PA1_Msk;
  static constexpr Reg32_t PUCRA_PA0_Pos  = ( 0U );
  static constexpr Reg32_t PUCRA_PA0_Msk  = ( 0x1UL << PUCRA_PA0_Pos );
  static constexpr Reg32_t PUCRA_PA0      = PUCRA_PA0_Msk;

  /********************  Bit definition for PDCRA register  ********************/
  static constexpr Reg32_t PDCRA_PA14_Pos = ( 14U );
  static constexpr Reg32_t PDCRA_PA14_Msk = ( 0x1UL << PDCRA_PA14_Pos );
  static constexpr Reg32_t PDCRA_PA14     = PDCRA_PA14_Msk;
  static constexpr Reg32_t PDCRA_PA12_Pos = ( 12U );
  static constexpr Reg32_t PDCRA_PA12_Msk = ( 0x1UL << PDCRA_PA12_Pos );
  static constexpr Reg32_t PDCRA_PA12     = PDCRA_PA12_Msk;
  static constexpr Reg32_t PDCRA_PA11_Pos = ( 11U );
  static constexpr Reg32_t PDCRA_PA11_Msk = ( 0x1UL << PDCRA_PA11_Pos );
  static constexpr Reg32_t PDCRA_PA11     = PDCRA_PA11_Msk;
  static constexpr Reg32_t PDCRA_PA10_Pos = ( 10U );
  static constexpr Reg32_t PDCRA_PA10_Msk = ( 0x1UL << PDCRA_PA10_Pos );
  static constexpr Reg32_t PDCRA_PA10     = PDCRA_PA10_Msk;
  static constexpr Reg32_t PDCRA_PA9_Pos  = ( 9U );
  static constexpr Reg32_t PDCRA_PA9_Msk  = ( 0x1UL << PDCRA_PA9_Pos );
  static constexpr Reg32_t PDCRA_PA9      = PDCRA_PA9_Msk;
  static constexpr Reg32_t PDCRA_PA8_Pos  = ( 8U );
  static constexpr Reg32_t PDCRA_PA8_Msk  = ( 0x1UL << PDCRA_PA8_Pos );
  static constexpr Reg32_t PDCRA_PA8      = PDCRA_PA8_Msk;
  static constexpr Reg32_t PDCRA_PA7_Pos  = ( 7U );
  static constexpr Reg32_t PDCRA_PA7_Msk  = ( 0x1UL << PDCRA_PA7_Pos );
  static constexpr Reg32_t PDCRA_PA7      = PDCRA_PA7_Msk;
  static constexpr Reg32_t PDCRA_PA6_Pos  = ( 6U );
  static constexpr Reg32_t PDCRA_PA6_Msk  = ( 0x1UL << PDCRA_PA6_Pos );
  static constexpr Reg32_t PDCRA_PA6      = PDCRA_PA6_Msk;
  static constexpr Reg32_t PDCRA_PA5_Pos  = ( 5U );
  static constexpr Reg32_t PDCRA_PA5_Msk  = ( 0x1UL << PDCRA_PA5_Pos );
  static constexpr Reg32_t PDCRA_PA5      = PDCRA_PA5_Msk;
  static constexpr Reg32_t PDCRA_PA4_Pos  = ( 4U );
  static constexpr Reg32_t PDCRA_PA4_Msk  = ( 0x1UL << PDCRA_PA4_Pos );
  static constexpr Reg32_t PDCRA_PA4      = PDCRA_PA4_Msk;
  static constexpr Reg32_t PDCRA_PA3_Pos  = ( 3U );
  static constexpr Reg32_t PDCRA_PA3_Msk  = ( 0x1UL << PDCRA_PA3_Pos );
  static constexpr Reg32_t PDCRA_PA3      = PDCRA_PA3_Msk;
  static constexpr Reg32_t PDCRA_PA2_Pos  = ( 2U );
  static constexpr Reg32_t PDCRA_PA2_Msk  = ( 0x1UL << PDCRA_PA2_Pos );
  static constexpr Reg32_t PDCRA_PA2      = PDCRA_PA2_Msk;
  static constexpr Reg32_t PDCRA_PA1_Pos  = ( 1U );
  static constexpr Reg32_t PDCRA_PA1_Msk  = ( 0x1UL << PDCRA_PA1_Pos );
  static constexpr Reg32_t PDCRA_PA1      = PDCRA_PA1_Msk;
  static constexpr Reg32_t PDCRA_PA0_Pos  = ( 0U );
  static constexpr Reg32_t PDCRA_PA0_Msk  = ( 0x1UL << PDCRA_PA0_Pos );
  static constexpr Reg32_t PDCRA_PA0      = PDCRA_PA0_Msk;

  /********************  Bit definition for PUCRB register  ********************/
  static constexpr Reg32_t PUCRB_PB7_Pos = ( 7U );
  static constexpr Reg32_t PUCRB_PB7_Msk = ( 0x1UL << PUCRB_PB7_Pos );
  static constexpr Reg32_t PUCRB_PB7     = PUCRB_PB7_Msk;
  static constexpr Reg32_t PUCRB_PB6_Pos = ( 6U );
  static constexpr Reg32_t PUCRB_PB6_Msk = ( 0x1UL << PUCRB_PB6_Pos );
  static constexpr Reg32_t PUCRB_PB6     = PUCRB_PB6_Msk;
  static constexpr Reg32_t PUCRB_PB5_Pos = ( 5U );
  static constexpr Reg32_t PUCRB_PB5_Msk = ( 0x1UL << PUCRB_PB5_Pos );
  static constexpr Reg32_t PUCRB_PB5     = PUCRB_PB5_Msk;
  static constexpr Reg32_t PUCRB_PB4_Pos = ( 4U );
  static constexpr Reg32_t PUCRB_PB4_Msk = ( 0x1UL << PUCRB_PB4_Pos );
  static constexpr Reg32_t PUCRB_PB4     = PUCRB_PB4_Msk;
  static constexpr Reg32_t PUCRB_PB3_Pos = ( 3U );
  static constexpr Reg32_t PUCRB_PB3_Msk = ( 0x1UL << PUCRB_PB3_Pos );
  static constexpr Reg32_t PUCRB_PB3     = PUCRB_PB3_Msk;
  static constexpr Reg32_t PUCRB_PB1_Pos = ( 1U );
  static constexpr Reg32_t PUCRB_PB1_Msk = ( 0x1UL << PUCRB_PB1_Pos );
  static constexpr Reg32_t PUCRB_PB1     = PUCRB_PB1_Msk;
  static constexpr Reg32_t PUCRB_PB0_Pos = ( 0U );
  static constexpr Reg32_t PUCRB_PB0_Msk = ( 0x1UL << PUCRB_PB0_Pos );
  static constexpr Reg32_t PUCRB_PB0     = PUCRB_PB0_Msk;

  /********************  Bit definition for PDCRB register  ********************/
  static constexpr Reg32_t PDCRB_PB7_Pos = ( 7U );
  static constexpr Reg32_t PDCRB_PB7_Msk = ( 0x1UL << PDCRB_PB7_Pos );
  static constexpr Reg32_t PDCRB_PB7     = PDCRB_PB7_Msk;
  static constexpr Reg32_t PDCRB_PB6_Pos = ( 6U );
  static constexpr Reg32_t PDCRB_PB6_Msk = ( 0x1UL << PDCRB_PB6_Pos );
  static constexpr Reg32_t PDCRB_PB6     = PDCRB_PB6_Msk;
  static constexpr Reg32_t PDCRB_PB5_Pos = ( 5U );
  static constexpr Reg32_t PDCRB_PB5_Msk = ( 0x1UL << PDCRB_PB5_Pos );
  static constexpr Reg32_t PDCRB_PB5     = PDCRB_PB5_Msk;
  static constexpr Reg32_t PDCRB_PB3_Pos = ( 3U );
  static constexpr Reg32_t PDCRB_PB3_Msk = ( 0x1UL << PDCRB_PB3_Pos );
  static constexpr Reg32_t PDCRB_PB3     = PDCRB_PB3_Msk;
  static constexpr Reg32_t PDCRB_PB1_Pos = ( 1U );
  static constexpr Reg32_t PDCRB_PB1_Msk = ( 0x1UL << PDCRB_PB1_Pos );
  static constexpr Reg32_t PDCRB_PB1     = PDCRB_PB1_Msk;
  static constexpr Reg32_t PDCRB_PB0_Pos = ( 0U );
  static constexpr Reg32_t PDCRB_PB0_Msk = ( 0x1UL << PDCRB_PB0_Pos );
  static constexpr Reg32_t PDCRB_PB0     = PDCRB_PB0_Msk;

  /********************  Bit definition for PUCRC register  ********************/
  static constexpr Reg32_t PUCRC_PC15_Pos = ( 15U );
  static constexpr Reg32_t PUCRC_PC15_Msk = ( 0x1UL << PUCRC_PC15_Pos );
  static constexpr Reg32_t PUCRC_PC15     = PUCRC_PC15_Msk;
  static constexpr Reg32_t PUCRC_PC14_Pos = ( 14U );
  static constexpr Reg32_t PUCRC_PC14_Msk = ( 0x1UL << PUCRC_PC14_Pos );
  static constexpr Reg32_t PUCRC_PC14     = PUCRC_PC14_Msk;

  /********************  Bit definition for PDCRC register  ********************/
  static constexpr Reg32_t PDCRC_PC15_Pos = ( 15U );
  static constexpr Reg32_t PDCRC_PC15_Msk = ( 0x1UL << PDCRC_PC15_Pos );
  static constexpr Reg32_t PDCRC_PC15     = PDCRC_PC15_Msk;
  static constexpr Reg32_t PDCRC_PC14_Pos = ( 14U );
  static constexpr Reg32_t PDCRC_PC14_Msk = ( 0x1UL << PDCRC_PC14_Pos );
  static constexpr Reg32_t PDCRC_PC14     = PDCRC_PC14_Msk;

  /********************  Bit definition for PUCRH register  ********************/
  static constexpr Reg32_t PUCRH_PH3_Pos = ( 3U );
  static constexpr Reg32_t PUCRH_PH3_Msk = ( 0x1UL << PUCRH_PH3_Pos );
  static constexpr Reg32_t PUCRH_PH3     = PUCRH_PH3_Msk;

  /********************  Bit definition for PDCRH register  ********************/
  static constexpr Reg32_t PDCRH_PH3_Pos = ( 3U );
  static constexpr Reg32_t PDCRH_PH3_Msk = ( 0x1UL << PDCRH_PH3_Pos );
  static constexpr Reg32_t PDCRH_PH3     = PDCRH_PH3_Msk;

}    // namespace Thor::LLD::POWER

#endif /* !THOR_HW_POWER_REGISTER_STM32L432KC_HPP */