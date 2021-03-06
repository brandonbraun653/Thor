/********************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32l432kc.hpp
 *
 *  Description:
 *    GPIO definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_REGISTER_STM32L432KC_HPP
#define THOR_HW_GPIO_REGISTER_STM32L432KC_HPP

/* C++ Includes */
#include <cstdint>
#include <cstddef>

/* Chimera Includes */
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_GPIOA_PERIPH_AVAILABLE
#define STM32_GPIOB_PERIPH_AVAILABLE
#define STM32_GPIOC_PERIPH_AVAILABLE

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_GPIO_PERIPHS = 3;   /**< Supported GPIO peripherals */
  static constexpr size_t NUM_GPIO_PINS    = 25;  /**< Max available pins to be configured as GPIO */

  static constexpr RIndex_t GPIOA_RESOURCE_INDEX = 0;
  static constexpr RIndex_t GPIOB_RESOURCE_INDEX = 1;
  static constexpr RIndex_t GPIOC_RESOURCE_INDEX = 2;

  static constexpr uint8_t GPIOA_NUM_PINS = 16;
  static constexpr uint8_t GPIOB_NUM_PINS = 7;
  static constexpr uint8_t GPIOC_NUM_PINS = 2;

  static constexpr uint8_t PRJ_MAX_PORTS         = NUM_GPIO_PERIPHS;
  static constexpr uint8_t PRJ_MAX_PINS          = NUM_GPIO_PINS;
  static constexpr uint8_t PRJ_MAX_PINS_PER_PORT = 16;

  static constexpr Chimera::GPIO::Port PRJ_LAST_PORT = Chimera::GPIO::Port::PORTC;

}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_REGISTER_STM32L432KC_HPP */