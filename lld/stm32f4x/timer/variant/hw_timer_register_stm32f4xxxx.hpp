/********************************************************************************
 *  File Name:
 *    hw_timer_register_stm32f4xxxx.hpp
 *
 *  Description:
 *    TIMER register definitions for the STM32F4 series chips.
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_TIMER_REGISTER_STM32F4_HPP
#define THOR_HW_TIMER_REGISTER_STM32F4_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <array>
#include <cstdint>
#include <cstddef>
#include <Chimera/timer>
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

/*-----------------------------------------------------------------------------
Literals
-----------------------------------------------------------------------------*/
#define STM32_TIMER1_PERIPH_AVAILABLE
#define STM32_TIMER2_PERIPH_AVAILABLE
#define STM32_TIMER3_PERIPH_AVAILABLE
#define STM32_TIMER4_PERIPH_AVAILABLE
#define STM32_TIMER5_PERIPH_AVAILABLE
#define STM32_TIMER6_PERIPH_AVAILABLE
#define STM32_TIMER7_PERIPH_AVAILABLE
#define STM32_TIMER8_PERIPH_AVAILABLE
#define STM32_TIMER9_PERIPH_AVAILABLE
#define STM32_TIMER10_PERIPH_AVAILABLE
#define STM32_TIMER11_PERIPH_AVAILABLE
#define STM32_TIMER12_PERIPH_AVAILABLE
#define STM32_TIMER13_PERIPH_AVAILABLE
#define STM32_TIMER14_PERIPH_AVAILABLE

namespace Thor::LLD::TIMER
{
  static constexpr uint32_t TIMER1_BASE_ADDR  = Thor::System::MemoryMap::TIM1_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER2_BASE_ADDR  = Thor::System::MemoryMap::TIM2_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER3_BASE_ADDR  = Thor::System::MemoryMap::TIM3_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER4_BASE_ADDR  = Thor::System::MemoryMap::TIM4_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER5_BASE_ADDR  = Thor::System::MemoryMap::TIM5_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER6_BASE_ADDR  = Thor::System::MemoryMap::TIM6_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER7_BASE_ADDR  = Thor::System::MemoryMap::TIM7_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER8_BASE_ADDR  = Thor::System::MemoryMap::TIM8_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER9_BASE_ADDR  = Thor::System::MemoryMap::TIM9_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER10_BASE_ADDR = Thor::System::MemoryMap::TIM10_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER11_BASE_ADDR = Thor::System::MemoryMap::TIM11_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER12_BASE_ADDR = Thor::System::MemoryMap::TIM12_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER13_BASE_ADDR = Thor::System::MemoryMap::TIM13_PERIPH_START_ADDRESS;
  static constexpr uint32_t TIMER14_BASE_ADDR = Thor::System::MemoryMap::TIM14_PERIPH_START_ADDRESS;
}    // namespace Thor::LLD::TIMER

#endif /* !THOR_HW_TIMER_REGISTER_STM32F4_HPP */
