/********************************************************************************
 *   File Name:
 *    hw_rcc_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_RCC_MAPPING_HPP
#define THOR_HW_RCC_MAPPING_HPP

/* C++ Includes */
#include <array>
#include <unordered_map>

/* Chimera Includes */
#include <Chimera/types/peripheral_types.hpp>
#include <Chimera/types/gpio_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_prj.hpp>
#include <Thor/drivers/f4/uart/hw_uart_prj.hpp>
#include <Thor/drivers/f4/usart/hw_usart_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/system/sys_memory_map_prj.hpp>

namespace Thor::Driver::RCC
{
  /**
   *  GPIO Peripheral Config Lookup Tables
   */
  extern const std::array<ClockEnableConfig, Thor::Driver::GPIO::NUM_GPIO_PERIPHS> ClockConfig_GPIO;
  extern const std::array<ClockEnableLowPowerConfig, Thor::Driver::GPIO::NUM_GPIO_PERIPHS> ClockConfigLP_GPIO;
  extern const std::array<PeripheralResetConfig, Thor::Driver::GPIO::NUM_GPIO_PERIPHS> ResetConfig_GPIO;

  /**
   *  UART Peripheral Config Lookup Tables
   */
  extern const std::array<ClockEnableConfig, Thor::Driver::UART::NUM_UART_PERIPHS> ClockConfig_UART;
  extern const std::array<ClockEnableLowPowerConfig, Thor::Driver::UART::NUM_UART_PERIPHS> ClockConfigLP_UART;
  extern const std::array<PeripheralResetConfig, Thor::Driver::UART::NUM_UART_PERIPHS> ResetConfig_UART;

  /**
   *  USART Peripheral Config Lookup Tables
   */
  extern const std::array<ClockEnableConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ClockConfig_USART;
  extern const std::array<ClockEnableLowPowerConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ClockConfigLP_USART;
  extern const std::array<PeripheralResetConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ResetConfig_USART;

}    // namespace Thor::Driver::RCC

#endif /* !THOR_HW_RCC_MAPPING_HPP */
