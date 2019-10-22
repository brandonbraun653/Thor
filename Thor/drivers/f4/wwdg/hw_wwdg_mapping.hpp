/********************************************************************************
 *   File Name:
 *    hw_wwdg_mapping.hpp
 *
 *   Description:
 *    Mappings for the watchdog timer resources
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_WWDG_MAPPING_HPP
#define THOR_HW_WWDG_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WWDG == 1 )

namespace Thor::Driver::WWDG
{
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
  extern RegisterMap *WWDG1_PERIPH;
#endif 
  
  /*------------------------------------------------
  Peripheral Memory Mapping
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;

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
  extern bool isWWDG( const std::uintptr_t address );

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_WWDG_MAPPING_HPP */