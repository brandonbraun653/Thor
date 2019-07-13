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
  namespace LookupTables
  {
    /**
     *  GPIO Peripheral Config Lookup Tables
     */
    static constexpr size_t gpioTableSize = Thor::Driver::GPIO::NUM_GPIO_PERIPHS;

    extern const ClockEnableConfig ClockConfig_GPIO[ gpioTableSize ];
    extern const ClockEnableLowPowerConfig ClockConfigLP_GPIO[ gpioTableSize ];
    extern const PeripheralResetConfig ResetConfig_GPIO[ gpioTableSize ];

    /**
     *  UART Peripheral Config Lookup Tables
     */
    static constexpr size_t uartTableSize = Thor::Driver::UART::NUM_UART_PERIPHS;

    extern const ClockEnableConfig ClockConfig_UART[uartTableSize];
    extern const ClockEnableLowPowerConfig ClockConfigLP_UART[uartTableSize];
    extern const PeripheralResetConfig ResetConfig_UART[uartTableSize];

    /**
     *  USART Peripheral Config Lookup Tables
     */
    static constexpr size_t usartTableSize = Thor::Driver::USART::NUM_USART_PERIPHS;
    
    extern const ClockEnableConfig ClockConfig_USART[usartTableSize];
    extern const ClockEnableLowPowerConfig ClockConfigLP_USART[usartTableSize];
    extern const PeripheralResetConfig ResetConfig_USART[usartTableSize ];
  }

}    // namespace Thor::Driver::RCC

#endif /* !THOR_HW_RCC_MAPPING_HPP */
