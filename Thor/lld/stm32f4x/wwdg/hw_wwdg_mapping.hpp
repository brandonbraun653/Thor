/********************************************************************************
 *  File Name:
 *    hw_wwdg_mapping.hpp
 *
 *  Description:
 *    Mappings for the watchdog timer resources
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_WWDG_MAPPING_HPP
#define THOR_HW_WWDG_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container>

/* Driver Includes */
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_prj.hpp>
#include <Thor/lld/stm32f4x/wwdg/hw_wwdg_types.hpp>

namespace Thor::LLD::WWDG
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

#endif /* !THOR_HW_WWDG_MAPPING_HPP */