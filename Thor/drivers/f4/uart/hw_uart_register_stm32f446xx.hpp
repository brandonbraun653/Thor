/********************************************************************************
 *   File Name:
 *    hw_uart_register_stm32f446xx.hpp
 *
 *   Description:
 *    Explicit hardware register definitions for the STM32F446xx USART/UART
 *    peripherals. The datasheet for this device makes no distinction between the
 *    UART and USART registers, so the definitions for both are combined here.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_UART_REGISTER_HPP
#define THOR_HW_UART_REGISTER_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/system/sys_memory_map_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )

#define STM32_UART4_PERIPH_AVAILABLE
#define STM32_UART5_PERIPH_AVAILABLE

namespace Thor::Driver::UART
{
  static constexpr Reg32_t UART4_BASE_ADDR  = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x4C00U;
  static constexpr Reg32_t UART5_BASE_ADDR  = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x5000U;

  static constexpr Reg32_t NUM_UART_PERIPHS = 2u;

  static constexpr std::array<Reg32_t, NUM_UART_PERIPHS> periphAddressList = { UART4_BASE_ADDR, UART5_BASE_ADDR };
}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
#endif /* !THOR_HW_UART_REGISTER_HPP */