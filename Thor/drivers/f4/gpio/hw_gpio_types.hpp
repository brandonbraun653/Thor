/********************************************************************************
 *   File Name:
 *    hw_gpio_types.hpp
 *
 *   Description:
 *    STM32F4 Types for the GPIO Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_TYPES_HPP
#define THOR_HW_GPIO_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/preprocessor.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

namespace Thor::Driver::GPIO
{
  struct RegisterMap
  {
    volatile uint32_t MODER;    /**< GPIO port mode register,               Address offset: 0x00      */
    volatile uint32_t OTYPER;   /**< GPIO port output type register,        Address offset: 0x04      */
    volatile uint32_t OSPEEDR;  /**< GPIO port output speed register,       Address offset: 0x08      */
    volatile uint32_t PUPDR;    /**< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    volatile uint32_t IDR;      /**< GPIO port input data register,         Address offset: 0x10      */
    volatile uint32_t ODR;      /**< GPIO port output data register,        Address offset: 0x14      */
    volatile uint32_t BSRR;     /**< GPIO port bit set/reset register,      Address offset: 0x18      */
    volatile uint32_t LCKR;     /**< GPIO port configuration lock register, Address offset: 0x1C      */
    volatile uint64_t AFR; /**< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  };

  static RegisterMap *const GPIOA_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOA_BASE_ADDR );
  static RegisterMap *const GPIOB_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOB_BASE_ADDR );
  static RegisterMap *const GPIOC_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOC_BASE_ADDR );
  static RegisterMap *const GPIOD_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOD_BASE_ADDR );
  static RegisterMap *const GPIOE_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOE_BASE_ADDR );
  static RegisterMap *const GPIOF_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOF_BASE_ADDR );
  static RegisterMap *const GPIOG_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOG_BASE_ADDR );
  static RegisterMap *const GPIOH_PERIPH = reinterpret_cast<RegisterMap *const>( GPIOH_BASE_ADDR );

  static constexpr uint32_t PIN_0        = 0x0001; /**< Pin 0 selected    */
  static constexpr uint32_t PIN_1        = 0x0002; /**< Pin 1 selected    */
  static constexpr uint32_t PIN_2        = 0x0004; /**< Pin 2 selected    */
  static constexpr uint32_t PIN_3        = 0x0008; /**< Pin 3 selected    */
  static constexpr uint32_t PIN_4        = 0x0010; /**< Pin 4 selected    */
  static constexpr uint32_t PIN_5        = 0x0020; /**< Pin 5 selected    */
  static constexpr uint32_t PIN_6        = 0x0040; /**< Pin 6 selected    */
  static constexpr uint32_t PIN_7        = 0x0080; /**< Pin 7 selected    */
  static constexpr uint32_t PIN_8        = 0x0100; /**< Pin 8 selected    */
  static constexpr uint32_t PIN_9        = 0x0200; /**< Pin 9 selected    */
  static constexpr uint32_t PIN_10       = 0x0400; /**< Pin 10 selected   */
  static constexpr uint32_t PIN_11       = 0x0800; /**< Pin 11 selected   */
  static constexpr uint32_t PIN_12       = 0x1000; /**< Pin 12 selected   */
  static constexpr uint32_t PIN_13       = 0x2000; /**< Pin 13 selected   */
  static constexpr uint32_t PIN_14       = 0x4000; /**< Pin 14 selected   */
  static constexpr uint32_t PIN_15       = 0x8000; /**< Pin 15 selected   */
  static constexpr uint32_t PIN_All      = 0xFFFF; /**< All pins selected */
  static constexpr uint32_t MAX_NUM_PINS = 16;

  /**
   *  Mode register configuration options
   *
   *  Elements values convention: 0xX0yz00YZ
   *    - X  : GPIO mode or EXTI Mode
   *    - y  : External IT or Event trigger detection
   *    - z  : IO configuration on External IT or Event
   *    - Y  : Output type (Push Pull or Open Drain)
   *    - Z  : IO Direction mode (Input, Output, Alternate or Analog)
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

  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  bool isGPIO( const std::uintptr_t address );
}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
#endif /* !THOR_HW_GPIO_TYPEGPIOHS_HPP */
