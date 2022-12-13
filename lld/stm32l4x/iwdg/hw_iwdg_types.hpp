/******************************************************************************
 *  File Name:
 *    hw_iwdg_types.hpp
 *
 *  Description:
 *    LLD types for the IWDG Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_IWDG_TYPES_HPP
#define THOR_HW_IWDG_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/iwdg/hw_iwdg_prj.hpp>

namespace Thor::LLD::Watchdog
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct IRegisterMap
  {
    volatile uint32_t KR;   /**< IWDG Key register,       Address offset: 0x00 */
    volatile uint32_t PR;   /**< IWDG Prescaler register, Address offset: 0x04 */
    volatile uint32_t RLR;  /**< IWDG Reload register,    Address offset: 0x08 */
    volatile uint32_t SR;   /**< IWDG Status register,    Address offset: 0x0C */
    volatile uint32_t WINR; /**< IWDG Window register,    Address offset: 0x10 */
  };

}

namespace Thor::LLD::IWDG
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( Watchdog::IRegisterMap, KR, KR_KEY_Msk, KEY, BIT_ACCESS_RW );
  REG_ACCESSOR( Watchdog::IRegisterMap, PR, PR_PR_Msk, PR, BIT_ACCESS_RW );
  REG_ACCESSOR( Watchdog::IRegisterMap, RLR, RLR_RL_Msk, RL, BIT_ACCESS_RW );
  REG_ACCESSOR( Watchdog::IRegisterMap, SR, SR_WVU_Msk, WVU, BIT_ACCESS_R );
  REG_ACCESSOR( Watchdog::IRegisterMap, SR, SR_RVU_Msk, RVU, BIT_ACCESS_R );
  REG_ACCESSOR( Watchdog::IRegisterMap, SR, SR_PVU_Msk, PVU, BIT_ACCESS_R );
  REG_ACCESSOR( Watchdog::IRegisterMap, WINR, WINR_WIN_Msk, WIN, BIT_ACCESS_RW );

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_HW_IWDG_TYPES_HPP*/
