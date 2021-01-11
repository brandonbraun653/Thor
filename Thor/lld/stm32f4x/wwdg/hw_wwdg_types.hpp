/********************************************************************************
 *  File Name:
 *    hw_wwdg_types.hpp
 *
 *  Description:
 *
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_WATCHDOG_TYPES_HPP
#define THOR_HW_WATCHDOG_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <array>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_prj.hpp>

namespace Thor::LLD::Watchdog
{
  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct WRegisterMap
  {
    volatile uint32_t CR;  /**< WWDG Control register,       Address offset: 0x00 */
    volatile uint32_t CFR; /**< WWDG Configuration register, Address offset: 0x04 */
    volatile uint32_t SR;  /**< WWDG Status register,        Address offset: 0x08 */
  };
}    // namespace Thor::LLD::WWDG


namespace Thor::LLD::WWDG
{
  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  REG_ACCESSOR( Watchdog::WRegisterMap, CR, CR_WDGA_Msk, WDGA, BIT_ACCESS_RS );
  REG_ACCESSOR( Watchdog::WRegisterMap, CR, CR_T_Msk, T, BIT_ACCESS_RW );

  REG_ACCESSOR( Watchdog::WRegisterMap, CFR, CFR_EWI_Msk, EWI, BIT_ACCESS_RS );
  REG_ACCESSOR( Watchdog::WRegisterMap, CFR, CFR_WDGTB_Msk, WDGTB, BIT_ACCESS_RW );
  REG_ACCESSOR( Watchdog::WRegisterMap, CFR, CFR_W_Msk, W, BIT_ACCESS_RW );

  REG_ACCESSOR( Watchdog::WRegisterMap, SR, SR_EWIF_Msk, EWIF, BIT_ACCESS_RCW0 );
}

#endif /* !THOR_HW_WWDG_TYPES_HPP*/
