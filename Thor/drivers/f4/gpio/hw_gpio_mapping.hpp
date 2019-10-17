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
  extern RegisterMap *const GPIOA_PERIPH;
  extern RegisterMap *const GPIOB_PERIPH;
  extern RegisterMap *const GPIOC_PERIPH;
  extern RegisterMap *const GPIOD_PERIPH;
  extern RegisterMap *const GPIOE_PERIPH;
  extern RegisterMap *const GPIOF_PERIPH;
  extern RegisterMap *const GPIOG_PERIPH;
  extern RegisterMap *const GPIOH_PERIPH;

  /**
   *  Maps a Chimera GPIO Pull type into the appropriate register configuration value
   */
  extern const std::array<uint32_t, static_cast<size_t>(Chimera::GPIO::Pull::NUM_OPTIONS)> PullMap;

  /**
   *  Maps a Chimera GPIO Drive type into the appropriate register configuration value
   */
  extern const std::array<uint32_t, static_cast<size_t>(Chimera::GPIO::Drive::NUM_OPTIONS)> ModeMap;

  /**
   *  Maps a Thor GPIO Speed type into the appropriate register configuration value
   */
  extern const std::array<uint32_t, static_cast<size_t>(Thor::Driver::GPIO::Speed::NUM_OPTIONS)> SpeedMap;

  /**
   *  Maps a Chimera Port enum type into a value that can be used as an
   *  access index into register configuration settings caches.
   *
   *  @note An example of this is Thor::Driver::RCC::ClockConfig_GPIO
   */
  extern const std::array<uint8_t, static_cast<size_t>(Chimera::GPIO::Port::NUM_OPTIONS)> PortToIteratorMap;

  /**
   *  Maps a GPIO peripheral into the corresponding Chimera Port enum type
   */
  extern const Chimera::Container::LightFlatMap<std::uintptr_t, Chimera::GPIO::Port> InstanceToPortMap;

  /**
   *  Maps a Chimera Port enum into the corresponding GPIO peripheral type
   */
  extern const Chimera::Container::LightFlatMap<Chimera::GPIO::Port, decltype( GPIOA_PERIPH )> PortToInstanceMap;

  /**
   *  Maps a GPIO peripheral into the corresponding resource index
   */
  extern const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;

}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
#endif /* !THOR_HW_GPIO_MAPPING_HPP */
