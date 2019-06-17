/********************************************************************************
 *   File Name:
 *    gpio_types.hpp
 *
 *   Description:
 *    Thor GPIO types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_GPIO_TYPES_HPP
#define THOR_GPIO_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <memory>

/* Thor Includes */
#include <Thor/headers.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
namespace Thor::GPIO
{
  class GPIOClass;
  using GPIOClass_sPtr = std::shared_ptr<GPIOClass>;
  using GPIOClass_uPtr = std::unique_ptr<GPIOClass>;

  typedef GPIO_TypeDef *PinPort;

  constexpr uint32_t NOALTERNATE = ( 0x08000CC8 );    // Default value for the alternate configuration var

  enum class PinNum : uint32_t
  {
    PIN_0   = GPIO_PIN_0,
    PIN_1   = GPIO_PIN_1,
    PIN_2   = GPIO_PIN_2,
    PIN_3   = GPIO_PIN_3,
    PIN_4   = GPIO_PIN_4,
    PIN_5   = GPIO_PIN_5,
    PIN_6   = GPIO_PIN_6,
    PIN_7   = GPIO_PIN_7,
    PIN_8   = GPIO_PIN_8,
    PIN_9   = GPIO_PIN_9,
    PIN_10  = GPIO_PIN_10,
    PIN_11  = GPIO_PIN_11,
    PIN_12  = GPIO_PIN_12,
    PIN_13  = GPIO_PIN_13,
    PIN_14  = GPIO_PIN_14,
    PIN_15  = GPIO_PIN_15,
    PIN_ALL = GPIO_PIN_All,

    MAX_PINS  = 16,
    NOT_A_PIN = std::numeric_limits<std::int32_t>::max()
  };

  enum class PinMode : uint32_t
  {
    INPUT              = GPIO_MODE_INPUT,
    OUTPUT_PP          = GPIO_MODE_OUTPUT_PP,
    OUTPUT_OD          = GPIO_MODE_OUTPUT_OD,
    ALT_PP             = GPIO_MODE_AF_PP,
    ALT_OD             = GPIO_MODE_AF_OD,
    ANALOG             = GPIO_MODE_ANALOG,
    IT_RISING          = GPIO_MODE_IT_RISING,
    IT_FALLING         = GPIO_MODE_IT_FALLING,
    IT_RISING_FALLING  = GPIO_MODE_IT_RISING_FALLING,
    EVT_RISING         = GPIO_MODE_EVT_RISING,
    EVT_FALLING        = GPIO_MODE_EVT_FALLING,
    EVT_RISING_FALLING = GPIO_MODE_EVT_RISING_FALLING,

    NUM_MODES,
    UNKNOWN_MODE
  };

  enum class PinSpeed : uint32_t
  {
    LOW_SPD    = GPIO_SPEED_FREQ_LOW,
    MEDIUM_SPD = GPIO_SPEED_FREQ_MEDIUM,
    HIGH_SPD   = GPIO_SPEED_FREQ_HIGH,
    ULTRA_SPD  = GPIO_SPEED_FREQ_VERY_HIGH,

    NUM_SPEEDS,
    UNKNOWN_SPEED
  };

  enum class PinPull : uint32_t
  {
    NOPULL = GPIO_NOPULL,
    PULLUP = GPIO_PULLUP,
    PULLDN = GPIO_PULLDOWN,

    NUM_PULL,
    UNKNOWN_PULL
  };

  struct Initializer
  {
    PinPort GPIOx      = GPIOA;
    PinNum pinNum      = PinNum::NOT_A_PIN;
    PinMode mode       = PinMode::INPUT;
    PinSpeed speed     = PinSpeed::MEDIUM_SPD;
    PinPull pull       = PinPull::NOPULL;
    uint32_t alternate = NOALTERNATE;
  };

}    // namespace Thor::GPIO
#endif 

#endif /* !THOR_GPIO_TYPES_HPP */