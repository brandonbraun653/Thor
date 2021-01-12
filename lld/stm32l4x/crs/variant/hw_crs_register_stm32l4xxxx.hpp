/********************************************************************************
 *  File Name:
 *    hw_crs_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    CRS register definitions for the STM32L4xxxx series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_CRS_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_CRS_REGISTER_STM32L4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>


namespace Thor::LLD::CRS
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t CRS1_BASE_ADDR = Thor::System::MemoryMap::CRS1_PERIPH_START_ADDRESS;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /*******************  Bit definition for CR register  *********************/
  static constexpr Reg32_t CR_Msk = 0x00003FEF;
  static constexpr Reg32_t CR_Rst = 0x00002000;

  static constexpr Reg32_t CR_SYNCOKIE_Pos   = ( 0U );
  static constexpr Reg32_t CR_SYNCOKIE_Msk   = ( 0x1UL << CR_SYNCOKIE_Pos );
  static constexpr Reg32_t CR_SYNCOKIE       = CR_SYNCOKIE_Msk;
  static constexpr Reg32_t CR_SYNCWARNIE_Pos = ( 1U );
  static constexpr Reg32_t CR_SYNCWARNIE_Msk = ( 0x1UL << CR_SYNCWARNIE_Pos );
  static constexpr Reg32_t CR_SYNCWARNIE     = CR_SYNCWARNIE_Msk;
  static constexpr Reg32_t CR_ERRIE_Pos      = ( 2U );
  static constexpr Reg32_t CR_ERRIE_Msk      = ( 0x1UL << CR_ERRIE_Pos );
  static constexpr Reg32_t CR_ERRIE          = CR_ERRIE_Msk;
  static constexpr Reg32_t CR_ESYNCIE_Pos    = ( 3U );
  static constexpr Reg32_t CR_ESYNCIE_Msk    = ( 0x1UL << CR_ESYNCIE_Pos );
  static constexpr Reg32_t CR_ESYNCIE        = CR_ESYNCIE_Msk;
  static constexpr Reg32_t CR_CEN_Pos        = ( 5U );
  static constexpr Reg32_t CR_CEN_Msk        = ( 0x1UL << CR_CEN_Pos );
  static constexpr Reg32_t CR_CEN            = CR_CEN_Msk;
  static constexpr Reg32_t CR_AUTOTRIMEN_Pos = ( 6U );
  static constexpr Reg32_t CR_AUTOTRIMEN_Msk = ( 0x1UL << CR_AUTOTRIMEN_Pos );
  static constexpr Reg32_t CR_AUTOTRIMEN     = CR_AUTOTRIMEN_Msk;
  static constexpr Reg32_t CR_SWSYNC_Pos     = ( 7U );
  static constexpr Reg32_t CR_SWSYNC_Msk     = ( 0x1UL << CR_SWSYNC_Pos );
  static constexpr Reg32_t CR_SWSYNC         = CR_SWSYNC_Msk;
  static constexpr Reg32_t CR_TRIM_Pos       = ( 8U );
  static constexpr Reg32_t CR_TRIM_Msk       = ( 0x3FUL << CR_TRIM_Pos );
  static constexpr Reg32_t CR_TRIM           = CR_TRIM_Msk;
  static constexpr Reg32_t CR_TRIM_0         = ( 0x01UL << CR_TRIM_Pos );
  static constexpr Reg32_t CR_TRIM_1         = ( 0x02UL << CR_TRIM_Pos );
  static constexpr Reg32_t CR_TRIM_2         = ( 0x04UL << CR_TRIM_Pos );
  static constexpr Reg32_t CR_TRIM_3         = ( 0x08UL << CR_TRIM_Pos );
  static constexpr Reg32_t CR_TRIM_4         = ( 0x10UL << CR_TRIM_Pos );
  static constexpr Reg32_t CR_TRIM_5         = ( 0x20UL << CR_TRIM_Pos );

  /*******************  Bit definition for CFGR register  *********************/
  static constexpr Reg32_t CFGR_Msk = 0xB7FFFFFF;
  static constexpr Reg32_t CFGR_Rst = 0x2022BB7F;

  static constexpr Reg32_t CFGR_RELOAD_Pos = ( 0U );
  static constexpr Reg32_t CFGR_RELOAD_Msk = ( 0xFFFFUL << CFGR_RELOAD_Pos );
  static constexpr Reg32_t CFGR_RELOAD     = CFGR_RELOAD_Msk;
  static constexpr Reg32_t CFGR_FELIM_Pos  = ( 16U );
  static constexpr Reg32_t CFGR_FELIM_Msk  = ( 0xFFUL << CFGR_FELIM_Pos );
  static constexpr Reg32_t CFGR_FELIM      = CFGR_FELIM_Msk;

  static constexpr Reg32_t CFGR_SYNCDIV_Pos = ( 24U );
  static constexpr Reg32_t CFGR_SYNCDIV_Msk = ( 0x7UL << CFGR_SYNCDIV_Pos );
  static constexpr Reg32_t CFGR_SYNCDIV     = CFGR_SYNCDIV_Msk;
  static constexpr Reg32_t CFGR_SYNCDIV_0   = ( 0x1UL << CFGR_SYNCDIV_Pos );
  static constexpr Reg32_t CFGR_SYNCDIV_1   = ( 0x2UL << CFGR_SYNCDIV_Pos );
  static constexpr Reg32_t CFGR_SYNCDIV_2   = ( 0x4UL << CFGR_SYNCDIV_Pos );

  static constexpr Reg32_t CFGR_SYNCSRC_Pos = ( 28U );
  static constexpr Reg32_t CFGR_SYNCSRC_Msk = ( 0x3UL << CFGR_SYNCSRC_Pos );
  static constexpr Reg32_t CFGR_SYNCSRC     = CFGR_SYNCSRC_Msk;
  static constexpr Reg32_t CFGR_SYNCSRC_0   = ( 0x1UL << CFGR_SYNCSRC_Pos );
  static constexpr Reg32_t CFGR_SYNCSRC_1   = ( 0x2UL << CFGR_SYNCSRC_Pos );

  static constexpr Reg32_t CFGR_SYNCPOL_Pos = ( 31U );
  static constexpr Reg32_t CFGR_SYNCPOL_Msk = ( 0x1UL << CFGR_SYNCPOL_Pos );
  static constexpr Reg32_t CFGR_SYNCPOL     = CFGR_SYNCPOL_Msk;

  /*******************  Bit definition for ISR register  *********************/
  static constexpr Reg32_t ISR_SYNCOKF_Pos   = ( 0U );
  static constexpr Reg32_t ISR_SYNCOKF_Msk   = ( 0x1UL << ISR_SYNCOKF_Pos );
  static constexpr Reg32_t ISR_SYNCOKF       = ISR_SYNCOKF_Msk;
  static constexpr Reg32_t ISR_SYNCWARNF_Pos = ( 1U );
  static constexpr Reg32_t ISR_SYNCWARNF_Msk = ( 0x1UL << ISR_SYNCWARNF_Pos );
  static constexpr Reg32_t ISR_SYNCWARNF     = ISR_SYNCWARNF_Msk;
  static constexpr Reg32_t ISR_ERRF_Pos      = ( 2U );
  static constexpr Reg32_t ISR_ERRF_Msk      = ( 0x1UL << ISR_ERRF_Pos );
  static constexpr Reg32_t ISR_ERRF          = ISR_ERRF_Msk;
  static constexpr Reg32_t ISR_ESYNCF_Pos    = ( 3U );
  static constexpr Reg32_t ISR_ESYNCF_Msk    = ( 0x1UL << ISR_ESYNCF_Pos );
  static constexpr Reg32_t ISR_ESYNCF        = ISR_ESYNCF_Msk;
  static constexpr Reg32_t ISR_SYNCERR_Pos   = ( 8U );
  static constexpr Reg32_t ISR_SYNCERR_Msk   = ( 0x1UL << ISR_SYNCERR_Pos );
  static constexpr Reg32_t ISR_SYNCERR       = ISR_SYNCERR_Msk;
  static constexpr Reg32_t ISR_SYNCMISS_Pos  = ( 9U );
  static constexpr Reg32_t ISR_SYNCMISS_Msk  = ( 0x1UL << ISR_SYNCMISS_Pos );
  static constexpr Reg32_t ISR_SYNCMISS      = ISR_SYNCMISS_Msk;
  static constexpr Reg32_t ISR_TRIMOVF_Pos   = ( 10U );
  static constexpr Reg32_t ISR_TRIMOVF_Msk   = ( 0x1UL << ISR_TRIMOVF_Pos );
  static constexpr Reg32_t ISR_TRIMOVF       = ISR_TRIMOVF_Msk;
  static constexpr Reg32_t ISR_FEDIR_Pos     = ( 15U );
  static constexpr Reg32_t ISR_FEDIR_Msk     = ( 0x1UL << ISR_FEDIR_Pos );
  static constexpr Reg32_t ISR_FEDIR         = ISR_FEDIR_Msk;
  static constexpr Reg32_t ISR_FECAP_Pos     = ( 16U );
  static constexpr Reg32_t ISR_FECAP_Msk     = ( 0xFFFFUL << ISR_FECAP_Pos );
  static constexpr Reg32_t ISR_FECAP         = ISR_FECAP_Msk;

  /*******************  Bit definition for ICR register  *********************/
  static constexpr Reg32_t ICR_SYNCOKC_Pos   = ( 0U );
  static constexpr Reg32_t ICR_SYNCOKC_Msk   = ( 0x1UL << ICR_SYNCOKC_Pos );
  static constexpr Reg32_t ICR_SYNCOKC       = ICR_SYNCOKC_Msk;
  static constexpr Reg32_t ICR_SYNCWARNC_Pos = ( 1U );
  static constexpr Reg32_t ICR_SYNCWARNC_Msk = ( 0x1UL << ICR_SYNCWARNC_Pos );
  static constexpr Reg32_t ICR_SYNCWARNC     = ICR_SYNCWARNC_Msk;
  static constexpr Reg32_t ICR_ERRC_Pos      = ( 2U );
  static constexpr Reg32_t ICR_ERRC_Msk      = ( 0x1UL << ICR_ERRC_Pos );
  static constexpr Reg32_t ICR_ERRC          = ICR_ERRC_Msk;
  static constexpr Reg32_t ICR_ESYNCC_Pos    = ( 3U );
  static constexpr Reg32_t ICR_ESYNCC_Msk    = ( 0x1UL << ICR_ESYNCC_Pos );
  static constexpr Reg32_t ICR_ESYNCC        = ICR_ESYNCC_Msk;
}    // namespace Thor::LLD::CRS

#endif /* !THOR_HW_CRS_REGISTER_STM32L4XXXX_HPP */
