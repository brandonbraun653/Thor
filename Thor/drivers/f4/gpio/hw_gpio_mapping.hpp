/********************************************************************************
 *   File Name:
 *    hw_gpio_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_MAPPING_HPP
#define THOR_HW_GPIO_MAPPING_HPP

/* Chimera Includes */
#include <Chimera/container.hpp>
#include <Chimera/types/gpio_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_types.hpp>
#include <Thor/drivers/common/types/gpio_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

namespace Thor::Driver::GPIO
{
#if defined( STM32_GPIOA_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOA_PERIPH;
#endif

#if defined( STM32_GPIOB_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOB_PERIPH;
#endif

#if defined( STM32_GPIOC_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOC_PERIPH;
#endif

#if defined( STM32_GPIOD_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOD_PERIPH;
#endif

#if defined( STM32_GPIOE_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOE_PERIPH;
#endif

#if defined( STM32_GPIOF_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOF_PERIPH;
#endif

#if defined( STM32_GPIOG_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOG_PERIPH;
#endif

#if defined( STM32_GPIOH_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOH_PERIPH;
#endif

  /*------------------------------------------------
  Hardware Memory Mappings
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, Chimera::GPIO::Port> InstanceToPortMap;
  extern Chimera::Container::LightFlatMap<Chimera::GPIO::Port, RegisterMap *> PortToInstanceMap;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;

  /*------------------------------------------------
  Chimera Mapping From Config Option->Register Value
  ------------------------------------------------*/
  extern const std::array<uint32_t, static_cast<size_t>(Chimera::GPIO::Pull::NUM_OPTIONS)> PullMap;
  extern const std::array<uint32_t, static_cast<size_t>(Chimera::GPIO::Drive::NUM_OPTIONS)> ModeMap;
  extern const std::array<uint32_t, static_cast<size_t>(Thor::Driver::GPIO::Speed::NUM_OPTIONS)> SpeedMap;
  extern const std::array<uint32_t, static_cast<size_t>(Chimera::GPIO::Port::NUM_OPTIONS)> PortToIteratorMap;

  
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
   *  Initializes memory associated with mapping
   *  
   *  @return void
   */
  extern void initializeMapping();
  
  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  bool isGPIO( const std::uintptr_t address );

}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
#endif /* !THOR_HW_GPIO_MAPPING_HPP */
