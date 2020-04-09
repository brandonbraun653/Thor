/********************************************************************************
 *  File Name:
 *    hw_rcc_mapping.hpp
 *
 *  Description:
 *    Defines structures used to map various resources needed in the RCC driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_RCC_MAPPING_HPP
#define THOR_LLD_RCC_MAPPING_HPP

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/container>

/* Driver Includes */
#include <Thor/lld/stm32l4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_prj.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/system/sys_memory_map_prj.hpp>

namespace Thor::LLD::RCC
{
#if defined( STM32_RCC1_PERIPH_AVAILABLE )
  extern RegisterMap *RCC1_PERIPH;
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
  extern bool isRCC( const std::uintptr_t address );


  namespace LookupTables
  {
#if defined( THOR_LLD_GPIO )
    /**
     *  GPIO Peripheral Config Lookup Tables
     */
    extern void GPIOInit();
    static constexpr size_t gpioTableSize = Thor::LLD::GPIO::NUM_GPIO_PERIPHS;

    extern PCC GPIOLookup;

    extern RegisterConfig GPIO_ClockConfig[ gpioTableSize ];
    extern RegisterConfig GPIO_ResetConfig[ gpioTableSize ];
    extern ClockType_t GPIO_SourceClock[ gpioTableSize ];
#endif /* THOR_LLD_GPIO */
  }
}

#endif  /* !THOR_LLD_RCC_MAPPING_HPP */
