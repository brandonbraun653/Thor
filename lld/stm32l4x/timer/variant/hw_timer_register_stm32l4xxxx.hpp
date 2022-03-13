/********************************************************************************
 *  File Name:
 *    hw_timer_register_stm32l432kc.hpp
 *
 *  Description:
 *    TIMER register definitions for the STM32L432KC series chips.
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_TIMER_REGISTER_STM32L432KC_HPP
#define THOR_HW_TIMER_REGISTER_STM32L432KC_HPP

/* C++ Includes */
#include <array>
#include <cstdint>
#include <cstddef>

/* Chimera Includes */
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>

/*-------------------------------------------------
Peripheral Availability
-------------------------------------------------*/
// Available on every STM32L4 device
#define STM32_TIMER1_PERIPH_AVAILABLE
#define STM32_TIMER2_PERIPH_AVAILABLE
#define STM32_TIMER6_PERIPH_AVAILABLE
#define STM32_TIMER15_PERIPH_AVAILABLE
#define STM32_TIMER16_PERIPH_AVAILABLE

#if defined( STM32L432xx )
#define STM32_TIMER7_PERIPH_AVAILABLE
#endif


namespace Thor::LLD::TIMER
{

  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t TIMER1_BASE_ADDR   = Thor::System::MemoryMap::TIMER1_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER2_BASE_ADDR   = Thor::System::MemoryMap::TIMER2_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER6_BASE_ADDR   = Thor::System::MemoryMap::TIMER6_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER15_BASE_ADDR  = Thor::System::MemoryMap::TIMER15_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER16_BASE_ADDR  = Thor::System::MemoryMap::TIMER16_PERIPH_START_ADDRESS;
  static constexpr uint32_t LPTIMER1_BASE_ADDR = Thor::System::MemoryMap::LPTIMER1_PERIPH_START_ADDRESS;
  static constexpr uint32_t LPTIMER2_BASE_ADDR = Thor::System::MemoryMap::LPTIMER2_PERIPH_START_ADDRESS;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
  static constexpr uint32_t TIMER3_BASE_ADDR = Thor::System::MemoryMap::TIMER3_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t TIMER3_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif

#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
  static constexpr uint32_t TIMER7_BASE_ADDR = Thor::System::MemoryMap::TIMER7_PERIPH_START_ADDRESS;
#else
  static constexpr uint32_t TIMER7_BASE_ADDR = std::numeric_limits<uint32_t>::max();
#endif
}    // namespace Thor::LLD::TIMER

#endif /* !THOR_HW_TIMER_REGISTER_STM32L432KC_HPP */