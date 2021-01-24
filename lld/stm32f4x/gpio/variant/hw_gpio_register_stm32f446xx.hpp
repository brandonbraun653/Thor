/********************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32f446xx.hpp
 *
 *  Description:
 *    GPIO register definitions for the STM32F446xx series chips.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_REGISTER_HPP
#define THOR_HW_GPIO_REGISTER_HPP

/* C++ Includes */
#include <cstdint>
#include <cstddef>

/* Chimera Includes */
#include <Chimera/gpio>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_GPIOA_PERIPH_AVAILABLE
#define STM32_GPIOB_PERIPH_AVAILABLE
#define STM32_GPIOC_PERIPH_AVAILABLE
#define STM32_GPIOD_PERIPH_AVAILABLE
#define STM32_GPIOE_PERIPH_AVAILABLE
#define STM32_GPIOF_PERIPH_AVAILABLE
#define STM32_GPIOG_PERIPH_AVAILABLE
#define STM32_GPIOH_PERIPH_AVAILABLE

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_GPIO_PERIPHS = 8;   /**< Supported GPIO peripherals */
  static constexpr size_t NUM_GPIO_PINS    = 50;  /**< Max available pins to be configured as GPIO */

  static constexpr uint32_t GPIOA_RESOURCE_INDEX = 0u;
  static constexpr uint32_t GPIOB_RESOURCE_INDEX = 1u;
  static constexpr uint32_t GPIOC_RESOURCE_INDEX = 2u;
  static constexpr uint32_t GPIOD_RESOURCE_INDEX = 3u;
  static constexpr uint32_t GPIOE_RESOURCE_INDEX = 4u;
  static constexpr uint32_t GPIOF_RESOURCE_INDEX = 5u;
  static constexpr uint32_t GPIOG_RESOURCE_INDEX = 6u;
  static constexpr uint32_t GPIOH_RESOURCE_INDEX = 7u;

  static constexpr uint32_t GPIOA_NUM_PINS = 16u;
  static constexpr uint32_t GPIOB_NUM_PINS = 16u;
  static constexpr uint32_t GPIOC_NUM_PINS = 16u;
  static constexpr uint32_t GPIOD_NUM_PINS = 16u;
  static constexpr uint32_t GPIOE_NUM_PINS = 16u;
  static constexpr uint32_t GPIOF_NUM_PINS = 16u;
  static constexpr uint32_t GPIOG_NUM_PINS = 16u;
  static constexpr uint32_t GPIOH_NUM_PINS = 2u;

  static constexpr uint8_t PRJ_MAX_PORTS         = NUM_GPIO_PERIPHS;
  static constexpr uint8_t PRJ_MAX_PINS          = NUM_GPIO_PINS;
  static constexpr uint8_t PRJ_MAX_PINS_PER_PORT = 16;

  static constexpr Chimera::GPIO::Port PRJ_LAST_PORT = Chimera::GPIO::Port::PORTH;

}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_REGISTER_HPP */