/********************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32l432kc.hpp
 *
 *  Description:
 *    GPIO register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_REGISTER_STM32L432KC_HPP
#define THOR_HW_GPIO_REGISTER_STM32L432KC_HPP

/* C++ Includes */
#include <cstdint>
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

/*-------------------------------------------------
Peripheral Availability
-------------------------------------------------*/
#define STM32_GPIOA_PERIPH_AVAILABLE
#define STM32_GPIOB_PERIPH_AVAILABLE
#define STM32_GPIOC_PERIPH_AVAILABLE

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_GPIO_PERIPHS = 3;   /**< Supported GPIO peripherals */
  static constexpr size_t NUM_GPIO_PINS    = 26;  /**< Max available pins to be configured as GPIO */

  static constexpr RIndex_t GPIOA_RESOURCE_INDEX = 0;
  static constexpr RIndex_t GPIOB_RESOURCE_INDEX = 1;
  static constexpr RIndex_t GPIOC_RESOURCE_INDEX = 2;
  static constexpr RIndex_t GPIOD_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOE_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOF_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOG_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOH_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOI_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOJ_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOK_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOL_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;

  static constexpr uint8_t GPIOA_NUM_PINS = 16;
  static constexpr uint8_t GPIOB_NUM_PINS = 8;
  static constexpr uint8_t GPIOC_NUM_PINS = 2;
  static constexpr uint8_t GPIOD_NUM_PINS = 0;
  static constexpr uint8_t GPIOE_NUM_PINS = 0;
  static constexpr uint8_t GPIOF_NUM_PINS = 0;
  static constexpr uint8_t GPIOG_NUM_PINS = 0;
  static constexpr uint8_t GPIOH_NUM_PINS = 0;
  static constexpr uint8_t GPIOI_NUM_PINS = 0;
  static constexpr uint8_t GPIOJ_NUM_PINS = 0;
  static constexpr uint8_t GPIOK_NUM_PINS = 0;
  static constexpr uint8_t GPIOL_NUM_PINS = 0;

  static constexpr uint8_t PRJ_MAX_PORTS = NUM_GPIO_PERIPHS;
  static constexpr uint8_t PRJ_MAX_PINS  = NUM_GPIO_PINS;


  namespace Internal
  {
    /*-------------------------------------------------------------------------------
    PORT A CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------------------------------------
    PORT B CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------------------------------------
    PORT C CONFIGURATION
    -------------------------------------------------------------------------------*/
    /*-------------------------------------------------
    Alternate Function Array
    -------------------------------------------------*/
    static const AlternateFunc port_c14_alt_func[ 1 ] = { {
        .registerAltFunc = AF15_EVENTOUT,
        .chimeraAltFunc  = Chimera::GPIO::Alternate::EVENTOUT,
    } };

    static const AlternateFunc port_c15_alt_func[ 1 ] = { {
        .registerAltFunc = AF15_EVENTOUT,
        .chimeraAltFunc  = Chimera::GPIO::Alternate::EVENTOUT,
    } };

    /*-------------------------------------------------
    Pin Attributes Array
    -------------------------------------------------*/
    static const PinAttributes port_c_pin_attributes[ GPIOC_NUM_PINS ] = {
      { .pinID = 14, .afListSize = 1, .altFunc = port_c14_alt_func },
      { .pinID = 15, .afListSize = 1, .altFunc = port_c15_alt_func }
    };
  }

  static const PortAttributes portAttributes[ NUM_GPIO_PERIPHS ] = {
    // GPIO A
    {
      Chimera::GPIO::Port::PORTA,
      0,
      nullptr
    },

    // GPIO B
    {
      Chimera::GPIO::Port::PORTB,
      0,
      nullptr
    },

    // GPIO C
    {
      .portID = Chimera::GPIO::Port::PORTC,
      .pinListSize = 2,
      .pins = Internal::port_c_pin_attributes
    }
  };
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_REGISTER_STM32L432KC_HPP */