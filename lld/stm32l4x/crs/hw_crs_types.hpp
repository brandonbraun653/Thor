/********************************************************************************
 *  File Name:
 *    hw_crs_types.hpp
 *
 *  Description:
 *    STM32L4 types for the CRS peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_CRS_TYPES_HPP
#define THOR_HW_CRS_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_prj.hpp>

namespace Thor::LLD::CRS
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum class SyncSource : uint8_t
  {
    GPIO,
    LSE,
    USB_SOF,

    NUM_OPTIONS,
    UNKNOWN
  };


  enum class SyncDiv : uint8_t
  {
    SYNC_DIV1,
    SYNC_DIV2,
    SYNC_DIV4,
    SYNC_DIV8,
    SYNC_DIV16,
    SYNC_DIV32,
    SYNC_DIV64,
    SYNC_DIV128,

    NUM_OPTIONS,
    UNKNOWN
  };

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t CR;   /**< CRS ccontrol register,              Address offset: 0x00 */
    volatile uint32_t CFGR; /**< CRS configuration register,         Address offset: 0x04 */
    volatile uint32_t ISR;  /**< CRS interrupt and status register,  Address offset: 0x08 */
    volatile uint32_t ICR;  /**< CRS interrupt flag clear register,  Address offset: 0x0C */
  };

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Control Register (CR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR, CR_Msk, CR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_TRIM_Msk, TRIM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_SWSYNC_Msk, SWSYNC, BIT_ACCESS_RTW1 );
  REG_ACCESSOR( RegisterMap, CR, CR_AUTOTRIMEN_Msk, AUTOTRIMEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_CEN_Msk, CEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_ESYNCIE_Msk, ESYNCIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_ERRIE_Msk, ERRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_SYNCWARNIE_Msk, SYNCWARNIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR, CR_SYNCOKIE_Msk, SYNCOKIE, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Configuration Register CFGR
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_Msk, CFGR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_SYNCPOL_Msk, SYNCPOL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_SYNCSRC_Msk, SYNCSRC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_SYNCDIV_Msk, SYNCDIV, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_FELIM_Msk, FELIM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CFGR, CFGR_RELOAD_Msk, RELOAD, BIT_ACCESS_RW );

  /*-------------------------------------------------
  Interrupt Status Register (ISR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ISR, ISR_FECAP_Msk, FECAP, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_FEDIR_Msk, FEDIR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TRIMOVF_Msk, TRIMOVF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_SYNCMISS_Msk, SYNCMISS, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_SYNCERR_Msk, SYNCERR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_ESYNCF_Msk, ESYNCF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_ERRF_Msk, ERRF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_SYNCWARNF_Msk, SYNCWARNF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_SYNCOKF_Msk, SYNCOKF, BIT_ACCESS_R );

  /*-------------------------------------------------
  Interrupt Flag Clear Register (ICR)
  -------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ICR, ICR_ESYNCC_Msk, ESYNCC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_ERRC_Msk, ERRC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_SYNCWARNC_Msk, SYNCWARNC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ICR, ICR_SYNCOKC_Msk, SYNCOKC, BIT_ACCESS_RW );

}    // namespace Thor::LLD::CRS

#endif /* !THOR_HW_CRS_TYPES_HPP */
