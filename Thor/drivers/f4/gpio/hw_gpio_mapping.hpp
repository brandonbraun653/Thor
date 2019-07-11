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

/* C++ Includes */
#include <unordered_map>

/* Chimera Includes */
#include <Chimera/types/gpio_types.hpp>

/* Driver Includes */
#include <Thor/drivers/f4/gpio/hw_gpio_types.hpp>
#include <Thor/drivers/common/types/gpio_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

namespace Thor::Driver::GPIO
{
  /**
   *  Maps a Chimera GPIO Pull type into the appropriate register configuration value
   */
  extern const std::unordered_map<Chimera::GPIO::Pull, size_t> PullMap;

  /**
   *  Maps a Chimera GPIO Drive type into the appropriate register configuration value
   */
  extern const std::unordered_map<Chimera::GPIO::Drive, size_t> ModeMap;

  /**
   *  Maps a Chimera GPIO Speed type into the appropriate register configuration value
   */
  extern const std::unordered_map<Thor::Driver::GPIO::Speed, size_t> SpeedMap;

  /**
   *  Maps a Chimera Port enum type into a value that can be used as an
   *  access index into register configuration settings caches.
   *
   *  @note An example of this is Thor::Driver::RCC::ClockConfig_GPIO
   */
  extern const std::unordered_map<Chimera::GPIO::Port, uint8_t> PortToIteratorMap;

  /**
   *  Maps a GPIO peripheral into the corresponding Chimera Port enum type
   */
  extern const std::unordered_map<std::uintptr_t, Chimera::GPIO::Port> InstanceToPortMap;

  /**
   *  Maps a Chimera Port enum into the corresponding GPIO peripheral type
   */
  extern const std::unordered_map<Chimera::GPIO::Port, decltype( GPIOA_PERIPH )> PortToInstanceMap;

}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
#endif /* !THOR_HW_GPIO_MAPPING_HPP */
