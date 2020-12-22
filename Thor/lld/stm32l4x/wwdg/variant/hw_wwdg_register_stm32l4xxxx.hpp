/********************************************************************************
 *  File Name:
 *    hw_wwdg_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    WWDG register definitions for the STM32L4xxxx series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_WWDG_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_WWDG_REGISTER_STM32L4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>


namespace Thor::LLD::WWDG
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t WWDG1_BASE_ADDR = Thor::System::MemoryMap::WWDG1_PERIPH_START_ADDRESS;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /*******************  Bit definition for CR register  ********************/
  static constexpr uint32_t CR_T_Pos = ( 0U );
  static constexpr uint32_t CR_T_Msk = ( 0x7FUL << CR_T_Pos );
  static constexpr uint32_t CR_T     = CR_T_Msk;
  static constexpr uint32_t CR_T_0   = ( 0x01UL << CR_T_Pos );
  static constexpr uint32_t CR_T_1   = ( 0x02UL << CR_T_Pos );
  static constexpr uint32_t CR_T_2   = ( 0x04UL << CR_T_Pos );
  static constexpr uint32_t CR_T_3   = ( 0x08UL << CR_T_Pos );
  static constexpr uint32_t CR_T_4   = ( 0x10UL << CR_T_Pos );
  static constexpr uint32_t CR_T_5   = ( 0x20UL << CR_T_Pos );
  static constexpr uint32_t CR_T_6   = ( 0x40UL << CR_T_Pos );

  static constexpr uint32_t CR_WDGA_Pos = ( 7U );
  static constexpr uint32_t CR_WDGA_Msk = ( 0x1UL << CR_WDGA_Pos );
  static constexpr uint32_t CR_WDGA     = CR_WDGA_Msk;

  /*******************  Bit definition for CFR register  *******************/
  static constexpr uint32_t CFR_W_Pos = ( 0U );
  static constexpr uint32_t CFR_W_Msk = ( 0x7FUL << CFR_W_Pos );
  static constexpr uint32_t CFR_W     = CFR_W_Msk;
  static constexpr uint32_t CFR_W_0   = ( 0x01UL << CFR_W_Pos );
  static constexpr uint32_t CFR_W_1   = ( 0x02UL << CFR_W_Pos );
  static constexpr uint32_t CFR_W_2   = ( 0x04UL << CFR_W_Pos );
  static constexpr uint32_t CFR_W_3   = ( 0x08UL << CFR_W_Pos );
  static constexpr uint32_t CFR_W_4   = ( 0x10UL << CFR_W_Pos );
  static constexpr uint32_t CFR_W_5   = ( 0x20UL << CFR_W_Pos );
  static constexpr uint32_t CFR_W_6   = ( 0x40UL << CFR_W_Pos );

  static constexpr uint32_t CFR_WDGTB_Pos = ( 7U );
  static constexpr uint32_t CFR_WDGTB_Msk = ( 0x3UL << CFR_WDGTB_Pos );
  static constexpr uint32_t CFR_WDGTB     = CFR_WDGTB_Msk;
  static constexpr uint32_t CFR_WDGTB_0   = ( 0x1UL << CFR_WDGTB_Pos );
  static constexpr uint32_t CFR_WDGTB_1   = ( 0x2UL << CFR_WDGTB_Pos );

  static constexpr uint32_t CFR_EWI_Pos = ( 9U );
  static constexpr uint32_t CFR_EWI_Msk = ( 0x1UL << CFR_EWI_Pos );
  static constexpr uint32_t CFR_EWI     = CFR_EWI_Msk;

  /*******************  Bit definition for SR register  ********************/
  static constexpr uint32_t SR_EWIF_Pos = ( 0U );
  static constexpr uint32_t SR_EWIF_Msk = ( 0x1UL << SR_EWIF_Pos );
  static constexpr uint32_t SR_EWIF     = SR_EWIF_Msk;

}    // namespace Thor::LLD::WWDG

#endif /* !THOR_HW_WWDG_REGISTER_STM32L4XXXX_HPP */
