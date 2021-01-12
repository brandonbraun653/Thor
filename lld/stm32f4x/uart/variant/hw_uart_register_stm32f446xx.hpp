/********************************************************************************
 *  File Name:
 *    hw_uart_register_stm32f446xx.hpp
 *
 *  Description:
 *    Explicit hardware register definitions for the STM32F446xx USART/UART
 *    peripherals. The datasheet for this device makes no distinction between the
 *    UART and USART registers, so the definitions for both are combined here.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_UART_REGISTER_HPP
#define THOR_HW_UART_REGISTER_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

#define STM32_UART4_PERIPH_AVAILABLE
#define STM32_UART5_PERIPH_AVAILABLE

namespace Thor::LLD::UART
{


  static constexpr Reg32_t UART4_BASE_ADDR  = Thor::System::MemoryMap::UART4_PERIPH_START_ADDRESS;
  static constexpr Reg32_t UART5_BASE_ADDR  = Thor::System::MemoryMap::UART5_PERIPH_START_ADDRESS;

  static constexpr Reg32_t NUM_UART_PERIPHS = 2u;

  static constexpr uint32_t UART4_RESOURCE_INDEX = 0u;
  static constexpr uint32_t UART5_RESOURCE_INDEX = 1u;

  static constexpr std::array<Reg32_t, NUM_UART_PERIPHS> periphAddressList = { UART4_BASE_ADDR, UART5_BASE_ADDR };
}    // namespace Thor::LLD::USART

#endif /* !THOR_HW_UART_REGISTER_HPP */