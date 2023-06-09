/******************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32f446xx.hpp
 *
 *  Description:
 *    GPIO register definitions for the STM32F446xx series chips.
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_REGISTER_HPP
#define THOR_HW_GPIO_REGISTER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/gpio>
#include <Thor/lld/common/types.hpp>
#include <cstddef>
#include <cstdint>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/
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
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_GPIO_PERIPHS = 8;
  static constexpr size_t NUM_GPIO_PINS    = 114;

  static constexpr uint32_t GPIOA_RESOURCE_INDEX = 0u;
  static constexpr uint32_t GPIOB_RESOURCE_INDEX = 1u;
  static constexpr uint32_t GPIOC_RESOURCE_INDEX = 2u;
  static constexpr uint32_t GPIOD_RESOURCE_INDEX = 3u;
  static constexpr uint32_t GPIOE_RESOURCE_INDEX = 4u;
  static constexpr uint32_t GPIOF_RESOURCE_INDEX = 5u;
  static constexpr uint32_t GPIOG_RESOURCE_INDEX = 6u;
  static constexpr uint32_t GPIOH_RESOURCE_INDEX = 7u;
  static constexpr RIndex_t GPIOI_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOJ_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOK_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t GPIOL_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;

  static constexpr uint32_t GPIOA_NUM_PINS = 16u;
  static constexpr uint32_t GPIOB_NUM_PINS = 16u;
  static constexpr uint32_t GPIOC_NUM_PINS = 16u;
  static constexpr uint32_t GPIOD_NUM_PINS = 16u;
  static constexpr uint32_t GPIOE_NUM_PINS = 16u;
  static constexpr uint32_t GPIOF_NUM_PINS = 16u;
  static constexpr uint32_t GPIOG_NUM_PINS = 16u;
  static constexpr uint32_t GPIOH_NUM_PINS = 2u;

  static constexpr uint32_t GPIOA_START_PIN = 0;
  static constexpr uint32_t GPIOB_START_PIN = 0;
  static constexpr uint32_t GPIOC_START_PIN = 0;
  static constexpr uint32_t GPIOD_START_PIN = 0;
  static constexpr uint32_t GPIOE_START_PIN = 0;
  static constexpr uint32_t GPIOF_START_PIN = 0;
  static constexpr uint32_t GPIOG_START_PIN = 0;
  static constexpr uint32_t GPIOH_START_PIN = 0;

  static constexpr uint8_t PRJ_MAX_PORTS         = NUM_GPIO_PERIPHS;
  static constexpr uint8_t PRJ_MAX_PINS          = NUM_GPIO_PINS;
  static constexpr uint8_t PRJ_MAX_PINS_PER_PORT = 16;

  static constexpr Chimera::GPIO::Port PRJ_LAST_PORT = Chimera::GPIO::Port::PORTH;

  static constexpr size_t GPIOA_PIN_RINDEX_OFFSET = 0;
  static constexpr size_t GPIOB_PIN_RINDEX_OFFSET = GPIOA_PIN_RINDEX_OFFSET + GPIOA_NUM_PINS;
  static constexpr size_t GPIOC_PIN_RINDEX_OFFSET = GPIOB_PIN_RINDEX_OFFSET + GPIOB_NUM_PINS;
  static constexpr size_t GPIOD_PIN_RINDEX_OFFSET = GPIOC_PIN_RINDEX_OFFSET + GPIOC_NUM_PINS;
  static constexpr size_t GPIOE_PIN_RINDEX_OFFSET = GPIOD_PIN_RINDEX_OFFSET + GPIOD_NUM_PINS;
  static constexpr size_t GPIOF_PIN_RINDEX_OFFSET = GPIOE_PIN_RINDEX_OFFSET + GPIOE_NUM_PINS;
  static constexpr size_t GPIOG_PIN_RINDEX_OFFSET = GPIOF_PIN_RINDEX_OFFSET + GPIOF_NUM_PINS;
  static constexpr size_t GPIOH_PIN_RINDEX_OFFSET = GPIOG_PIN_RINDEX_OFFSET + GPIOG_NUM_PINS;
  static constexpr size_t GPIOI_PIN_RINDEX_OFFSET = INVALID_RESOURCE_INDEX;
  static constexpr size_t GPIOJ_PIN_RINDEX_OFFSET = INVALID_RESOURCE_INDEX;
  static constexpr size_t GPIOK_PIN_RINDEX_OFFSET = INVALID_RESOURCE_INDEX;
  static constexpr size_t GPIOL_PIN_RINDEX_OFFSET = INVALID_RESOURCE_INDEX;
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_REGISTER_HPP */