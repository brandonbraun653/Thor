/********************************************************************************
 *   File Name:
 *    hw_gpio_types.hpp
 *
 *   Description:
 *    STM32 Types for the GPIO Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_TYPES_HPP
#define THOR_HW_GPIO_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/drivers/f4/gpio/hw_gpio_prj.hpp>

namespace Thor::Driver::GPIO
{
  struct RegisterMap
  {
    volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
    volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
    volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
    volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
    volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
    volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
    volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    volatile uint32_t AFR[ 2 ]; /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  };

  static RegisterMap *const GPIOA_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOA_BASE_ADDR );
  static RegisterMap *const GPIOB_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOB_BASE_ADDR );
  static RegisterMap *const GPIOC_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOC_BASE_ADDR );
  static RegisterMap *const GPIOD_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOD_BASE_ADDR );
  static RegisterMap *const GPIOE_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOE_BASE_ADDR );
  static RegisterMap *const GPIOF_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOF_BASE_ADDR );
  static RegisterMap *const GPIOG_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOG_BASE_ADDR );
  static RegisterMap *const GPIOH_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOH_BASE_ADDR );
  
}    // namespace Thor::Driver::GPIO

#endif /* !THOR_HW_GPIO_TYPEGPIOHS_HPP */
