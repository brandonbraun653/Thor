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
#include <array>
#include <cstdint>
#include <cstddef>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

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
  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t GPIOA_BASE_ADDR = Thor::System::MemoryMap::GPIOA_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOB_BASE_ADDR = Thor::System::MemoryMap::GPIOB_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOC_BASE_ADDR = Thor::System::MemoryMap::GPIOC_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOD_BASE_ADDR = Thor::System::MemoryMap::GPIOD_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOE_BASE_ADDR = Thor::System::MemoryMap::GPIOE_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOF_BASE_ADDR = Thor::System::MemoryMap::GPIOF_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOG_BASE_ADDR = Thor::System::MemoryMap::GPIOG_PERIPH_START_ADDRESS;
  static constexpr uint32_t GPIOH_BASE_ADDR = Thor::System::MemoryMap::GPIOH_PERIPH_START_ADDRESS;

  static constexpr size_t NUM_GPIO_PERIPHS = 8;

  static constexpr uint32_t GPIOA_RESOURCE_INDEX = 0u;
  static constexpr uint32_t GPIOB_RESOURCE_INDEX = 1u;
  static constexpr uint32_t GPIOC_RESOURCE_INDEX = 2u;
  static constexpr uint32_t GPIOD_RESOURCE_INDEX = 3u;
  static constexpr uint32_t GPIOE_RESOURCE_INDEX = 4u;
  static constexpr uint32_t GPIOF_RESOURCE_INDEX = 5u;
  static constexpr uint32_t GPIOG_RESOURCE_INDEX = 6u;
  static constexpr uint32_t GPIOH_RESOURCE_INDEX = 7u;

}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_REGISTER_HPP */