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

/* Driver Includes */
#include <Thor/drivers/f4/system/sys_memory_map_stm32f446xx.hpp>

namespace Thor::Driver::UART
{
  static constexpr uint32_t UART4_BASE_ADDR  = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x4C00U;
  static constexpr uint32_t UART5_BASE_ADDR  = Thor::System::MemoryMap::APB1PERIPH_BASE_ADDR + 0x5000U;

  static constexpr uint32_t NUM_UART_PERIPHS = 2u;
}    // namespace Thor::Driver::USART

#endif /* !THOR_HW_UART_REGISTER_HPP */