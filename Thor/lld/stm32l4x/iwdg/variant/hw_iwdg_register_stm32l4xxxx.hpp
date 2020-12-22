/********************************************************************************
 *  File Name:
 *    hw_iwdg_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    IWDG register definitions for the STM32L4xxxx series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_IWDG_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_IWDG_REGISTER_STM32L4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>


namespace Thor::LLD::IWDG
{
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t IWDG1_BASE_ADDR = Thor::System::MemoryMap::IWDG1_PERIPH_START_ADDRESS;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_HW_IWDG_REGISTER_STM32L4XXXX_HPP */
