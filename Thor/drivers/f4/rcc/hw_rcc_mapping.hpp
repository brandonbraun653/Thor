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
#include <Thor/drivers/f4/gpio/hw_gpio_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_prj.hpp>
#include <Thor/drivers/f4/system/sys_memory_map_prj.hpp>

namespace Thor::Driver::RCC
{
  /* clang-format off */

  /**
   *  Generates the register access table for turning on GPIO clocks
   */
  static const std::array<CEConfig, Thor::Driver::GPIO::NUM_GPIO_PERIPHS> ClockConfig_GPIO = {{
    /* GPIOA */
    { reinterpret_cast<decltype(CEConfig::CER)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOAEN },
    /* GPIOB */
    { reinterpret_cast<decltype(CEConfig::CER)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOBEN },
    /* GPIOC */
    { reinterpret_cast<decltype(CEConfig::CER)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOCEN },
    /* GPIOD */
    { reinterpret_cast<decltype(CEConfig::CER)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIODEN },
    /* GPIOE */
    { reinterpret_cast<decltype(CEConfig::CER)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOEEN },
    /* GPIOF */
    { reinterpret_cast<decltype(CEConfig::CER)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOFEN },
    /* GPIOG */
    { reinterpret_cast<decltype(CEConfig::CER)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOGEN },
    /* GPIOH */
    { reinterpret_cast<decltype(CEConfig::CER)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1ENR_GPIOHEN }
  }};
  
  /** 
   *  Generates the register access table for resetting GPIO peripherals
   */
  static const std::array<PRRConfig, Thor::Driver::GPIO::NUM_GPIO_PERIPHS> ResetConfig_GPIO = {{
    /* GPIOA */
    { reinterpret_cast<decltype(PRRConfig::PRR)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOARST },
    /* GPIOB */
    { reinterpret_cast<decltype(PRRConfig::PRR)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOBRST },
    /* GPIOC */
    { reinterpret_cast<decltype(PRRConfig::PRR)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOCRST },
    /* GPIOD */
    { reinterpret_cast<decltype(PRRConfig::PRR)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIODRST },
    /* GPIOE */
    { reinterpret_cast<decltype(PRRConfig::PRR)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOERST },
    /* GPIOF */
    { reinterpret_cast<decltype(PRRConfig::PRR)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOFRST },
    /* GPIOG */
    { reinterpret_cast<decltype(PRRConfig::PRR)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOGRST },
    /* GPIOH */
    { reinterpret_cast<decltype(PRRConfig::PRR)>( RCC_BASE_ADDR + offsetof(RegisterMap, AHB1ENR) ), AHB1RSTR_GPIOHRST }
  }};


  /* clang-format on */
  }    // namespace Thor::Driver::GPIO

#endif /* !THOR_HW_RCC_MAPPING_HPP */
