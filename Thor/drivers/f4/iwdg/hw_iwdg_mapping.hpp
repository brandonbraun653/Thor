/********************************************************************************
 *   File Name:
 *    hw_iwdg_mapping.hpp
 *
 *   Description:
 *    Mappings for the watchdog timer resources
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_IWDG_MAPPING_HPP
#define THOR_HW_IWDG_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_IWDG == 1 )

namespace Thor::Driver::IWDG
{
  extern RegisterMap *const IWDG_PERIPH;

  /**
   *  Maps a Watchdog peripheral into the corresponding resource index for lookup tables.
   */
  extern const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_IWDG_MAPPING_HPP */