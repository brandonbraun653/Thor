/******************************************************************************
 *  File Name:
 *    hw_sys_register_stm32f446xx.hpp
 *
 *  Description:
 *    SYS definitions for the STM32F446XX series chips
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_SYS_REGISTER_STM32F446XX_HPP
#define THOR_HW_SYS_REGISTER_STM32F446XX_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/stm32f4x/system/sys_memory_map_prj.hpp>

namespace Thor::LLD::SYS
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_SYSCFG_PERIPHS       = 1;
  static constexpr RIndex_t SYSCFG1_RESOURCE_INDEX = 0;
  static constexpr size_t NUM_PERIPHERAL_TYPES     = 21;

  /*---------------------------------------------------------------------------
  Peripheral Instance Memory Map Base
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t SYSCFG_BASE_ADDR = Thor::System::MemoryMap::SYSCFG_PERIPH_START_ADDRESS;

}    // namespace Thor::LLD::SYS

#endif /* !THOR_HW_SYS_REGISTER_STM32F446XX_HPP */
