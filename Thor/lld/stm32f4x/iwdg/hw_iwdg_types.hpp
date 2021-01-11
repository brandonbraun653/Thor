/********************************************************************************
 *  File Name:
 *    hw_iwdg_types.hpp
 *
 *  Description:
 *    Types for the independent watchdog driver
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_IWDG_TYPES_HPP
#define THOR_HW_IWDG_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <array>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_prj.hpp>

namespace Thor::LLD::Watchdog
{
  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct IRegisterMap
  {
    volatile uint32_t KR;  /**< IWDG Key Register,       Address offset: 0x00 */
    volatile uint32_t PR;  /**< IWDG Prescale Register,  Address offset: 0x04 */
    volatile uint32_t RLR; /**< IWDG Reload Register,    Address offset: 0x08 */
    volatile uint32_t SR;  /**< IWDG Status Register,    Address offset: 0x0C */
  };

}    // namespace Thor::LLD::Watchdog


namespace Thor::LLD::IWDG
{
  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  REG_ACCESSOR( Watchdog::IRegisterMap, KR, KR_KEY_Msk, KEY, BIT_ACCESS_RW );
  REG_ACCESSOR( Watchdog::IRegisterMap, PR, PR_PR_Msk, PR, BIT_ACCESS_RW );
  REG_ACCESSOR( Watchdog::IRegisterMap, RLR, RLR_RL_Msk, RL, BIT_ACCESS_RW );
  REG_ACCESSOR( Watchdog::IRegisterMap, SR, SR_RVU_Msk, RVU, BIT_ACCESS_R );
  REG_ACCESSOR( Watchdog::IRegisterMap, SR, SR_PVU_Msk, PVU, BIT_ACCESS_R );

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_HW_IWDG_TYPES_HPP */
