/********************************************************************************
 *  File Name:
 *    hw_gpio_types.hpp
 *
 *  Description:
 *    STM32L4 Types for the GPIO Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_TYPES_HPP
#define THOR_HW_GPIO_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32l4x/gpio/hw_gpio_prj.hpp>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  /**
   *  Mode register configuration options
   */
  enum OPT_MODER : size_t
  {
    INPUT  = 0u, /**< Input Floating Mode               */
    OUTPUT = 1u, /**< Output Push Pull Mode             */
    AF     = 2u, /**< Alternate Function Push Pull Mode */
    ANALOG = 3u, /**< Analog Mode                       */
  };

  /**
   *  Output type register configuration options
   */
  enum OPT_OTYPER : size_t
  {
    PUSH_PULL  = 0u, /**< The GPIO will be configured as a push-pull output */
    OPEN_DRAIN = 1u  /**< The GPIO will be configured as an open-drain output */
  };

  /**
   *  Output speed register configuration options
   */
  enum OPT_OSPEEDR : size_t
  {
    LOW       = 0u, /**< IO works at 2 MHz, please refer to the product datasheet */
    MEDIUM    = 1u, /**< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
    HIGH      = 2u, /**< range 25 MHz to 100 MHz, please refer to the product datasheet  */
    VERY_HIGH = 3u  /**< range 50 MHz to 200 MHz, please refer to the product datasheet  */
  };

  /**
   *  Pull up/down register configuration options
   */
  enum OPT_PUPDR : size_t
  {
    NOPULL   = 0x00u, /**< No pull up or pull down activation */
    PULLUP   = 0x01u, /**< Pull up activation */
    PULLDOWN = 0x02u  /**< Pull down activation */
  };

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t MODER;   /**< GPIO port mode register,               Address offset: 0x00      */
    volatile uint32_t OTYPER;  /**< GPIO port output type register,        Address offset: 0x04      */
    volatile uint32_t OSPEEDR; /**< GPIO port output speed register,       Address offset: 0x08      */
    volatile uint32_t PUPDR;   /**< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    volatile uint32_t IDR;     /**< GPIO port input data register,         Address offset: 0x10      */
    volatile uint32_t ODR;     /**< GPIO port output data register,        Address offset: 0x14      */
    volatile uint32_t BSRR;    /**< GPIO port bit set/reset register,      Address offset: 0x18      */
    volatile uint32_t LCKR;    /**< GPIO port configuration lock register, Address offset: 0x1C      */
    volatile uint64_t AFR;     /**< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    volatile uint32_t BRR;     /**< GPIO port bit reset register,          Address offset: 0x28      */
  };

}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_TYPES_HPP*/
