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
  /* clang-format off */

/*------------------------------------------------
GPIO Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )
  /**
   *  GPIO clock enable register access lookup table
   */
  static const std::array<ClockEnableConfig, Thor::Driver::GPIO::NUM_GPIO_PERIPHS> ClockConfig_GPIO = {{
    /* GPIOA */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOAEN },
    /* GPIOB */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOBEN },
    /* GPIOC */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOCEN },
    /* GPIOD */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIODEN },
    /* GPIOE */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOEEN },
    /* GPIOF */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOFEN },
    /* GPIOG */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOGEN },
    /* GPIOH */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOHEN }
  }};
  
  /**
   *  GPIO low power clock enable register access lookup table
   */
  static const std::array<ClockEnableLowPowerConfig, Thor::Driver::GPIO::NUM_GPIO_PERIPHS> ClockConfigLP_GPIO = {{
    /* GPIOA */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1LPENR) ), AHB1LPENR_GPIOALPEN },
    /* GPIOB */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1LPENR) ), AHB1LPENR_GPIOBLPEN },
    /* GPIOC */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1LPENR) ), AHB1LPENR_GPIOCLPEN },
    /* GPIOD */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1LPENR) ), AHB1LPENR_GPIODLPEN },
    /* GPIOE */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1LPENR) ), AHB1LPENR_GPIOELPEN },
    /* GPIOF */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1LPENR) ), AHB1LPENR_GPIOFLPEN },
    /* GPIOG */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1LPENR) ), AHB1LPENR_GPIOGLPEN },
    /* GPIOH */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1LPENR) ), AHB1LPENR_GPIOHLPEN }
  }};

  /** 
   *  GPIO reset register access lookup table
   */
  static const std::array<PeripheralResetConfig, Thor::Driver::GPIO::NUM_GPIO_PERIPHS> ResetConfig_GPIO = {{
    /* GPIOA */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOARST },
    /* GPIOB */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOBRST },
    /* GPIOC */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOCRST },
    /* GPIOD */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIODRST },
    /* GPIOE */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOERST },
    /* GPIOF */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOFRST },
    /* GPIOG */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOGRST },
    /* GPIOH */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOHRST }
  }};

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */

/*------------------------------------------------
UART Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )
  /**
   *  UART clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  static const std::array<ClockEnableConfig, Thor::Driver::UART::NUM_UART_PERIPHS> ClockConfig_UART = {{
    /* UART4 */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1ENR) ), APB1ENR_UART4EN },
    /* UART5 */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1ENR) ), APB1ENR_UART5EN }
  }};

  /**
   *  UART low power clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  static const std::array<ClockEnableLowPowerConfig, Thor::Driver::UART::NUM_UART_PERIPHS> ClockConfigLP_UART = {{
    /* UART4 */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1LPENR) ), APB1LPENR_UART4LPEN },
    /* UART5 */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1LPENR) ), APB1LPENR_UART5LPEN }
  }};

  /** 
   *  UART reset register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_uart_mapping.hpp
   */
  static const std::array<PeripheralResetConfig, Thor::Driver::UART::NUM_UART_PERIPHS> ResetConfig_UART = {{
    /* UART4 */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1RSTR) ), APB1RSTR_UART4RST },
    /* UART5 */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1RSTR) ), APB1RSTR_UART5RST }
  }};
  
#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */

/*------------------------------------------------
USART Peripheral RCC Configuration Resources
------------------------------------------------*/
#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )
  /**
   *  USART clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_usart_mapping.hpp
   */
  static const std::array<ClockEnableConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ClockConfig_USART = {{
    /* USART1 */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB2ENR) ), APB2ENR_USART1EN },
    /* USART2 */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1ENR) ), APB1ENR_USART2EN },
    /* USART3 */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1ENR) ), APB1ENR_USART3EN },
    /* USART6 */
    { reinterpret_cast<decltype(ClockEnableConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB2ENR) ), APB2ENR_USART6EN }
  }};

  /**
   *  USART low power clock enable register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_usart_mapping.hpp
   */
  static const std::array<ClockEnableLowPowerConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ClockConfigLP_USART = {{
    /* USART1 */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB2LPENR) ), APB2LPENR_USART1LPEN },
    /* USART2 */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1LPENR) ), APB1LPENR_USART2LPEN },
    /* USART3 */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1LPENR) ), APB1LPENR_USART3LPEN },
    /* USART6 */
    { reinterpret_cast<decltype(ClockEnableLowPowerConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB2LPENR) ), APB2LPENR_USART6LPEN }
  }};

  /** 
   *  USART reset register access lookup table
   *
   *  @note Indexing must match the lookup table in hw_usart_mapping.hpp
   */
  static const std::array<PeripheralResetConfig, Thor::Driver::USART::NUM_USART_PERIPHS> ResetConfig_USART = {{
    /* USART1 */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB2RSTR) ), APB2RSTR_USART1RST },
    /* USART2 */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1RSTR) ), APB1RSTR_USART2RST },
    /* USART3 */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB1RSTR) ), APB1RSTR_USART3RST },
    /* USART6 */
    { reinterpret_cast<decltype(PeripheralResetConfig::reg)>( RCC_BASE_ADDR + offsetof(RegisterMap, APB2RSTR) ), APB2RSTR_USART6RST }
  
  }};

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */

  /* clang-format on */
}    // namespace Thor::Driver::RCC

#endif /* !THOR_HW_RCC_MAPPING_HPP */
