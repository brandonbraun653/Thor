/********************************************************************************
 *  File Name:
 *    hw_usart_register_stm32f446xx.hpp
 *
 *  Description:
 *    Explicit hardware register definitions for the STM32F446xx USART/UART
 *    peripherals. The datasheet for this device makes no distinction between the
 *    UART and USART registers, so the definitions for both are combined here.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_REGISTER_HPP
#define THOR_HW_USART_REGISTER_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_USART1_PERIPH_AVAILABLE
#define STM32_USART2_PERIPH_AVAILABLE
#define STM32_USART3_PERIPH_AVAILABLE
#define STM32_USART6_PERIPH_AVAILABLE

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr Reg32_t USART1_BASE_ADDR = Thor::System::MemoryMap::USART1_PERIPH_START_ADDRESS;
  static constexpr Reg32_t USART2_BASE_ADDR = Thor::System::MemoryMap::USART2_PERIPH_START_ADDRESS;
  static constexpr Reg32_t USART3_BASE_ADDR = Thor::System::MemoryMap::USART3_PERIPH_START_ADDRESS;
  static constexpr Reg32_t USART6_BASE_ADDR = Thor::System::MemoryMap::USART6_PERIPH_START_ADDRESS;

  static constexpr Reg32_t NUM_USART_PERIPHS = 4u;

  static constexpr uint32_t USART1_RESOURCE_INDEX = 0u;
  static constexpr uint32_t USART2_RESOURCE_INDEX = 1u;
  static constexpr uint32_t USART3_RESOURCE_INDEX = 2u;
  static constexpr uint32_t USART6_RESOURCE_INDEX = 3u;

}    // namespace Thor::LLD::USART

#endif /* !THOR_HW_USART_REGISTER_HPP */