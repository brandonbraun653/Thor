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
#include <Chimera/container>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_IWDG == 1 )

namespace Thor::Driver::IWDG
{
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
  extern RegisterMap * IWDG1_PERIPH;
#endif 

  /*------------------------------------------------
  Hardware Memory Mappings
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;

  /**
   *  Initializes memory associated with mapping
   *  
   *  @return void
   */
  void initializeMapping();

  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  bool isIWDG( const std::uintptr_t address );
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
#endif /* !THOR_HW_IWDG_MAPPING_HPP */