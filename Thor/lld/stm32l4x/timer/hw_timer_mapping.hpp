/********************************************************************************
 *  File Name:
 *    hw_timer_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_TIMER_MAPPING_HPP
#define THOR_HW_TIMER_MAPPING_HPP

/* STL Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container>
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------
  Define Memory Mapped Structs for Peripheral
  -------------------------------------------------*/
  extern RegisterMap *TIMER1_PERIPH;
  extern RegisterMap *TIMER2_PERIPH;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER3_PERIPH;
#endif

  extern RegisterMap *TIMER6_PERIPH;

#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER7_PERIPH;
#endif

  extern RegisterMap *TIMER15_PERIPH;
  extern RegisterMap *TIMER16_PERIPH;
  extern LPRegisterMap *LPTIMER1_PERIPH;
  extern LPRegisterMap *LPTIMER2_PERIPH;

  /*------------------------------------------------
  Hardware Memory Mappings
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;

  /*------------------------------------------------
  Mappings from Chimera Config Options->Register Values
  ------------------------------------------------*/
  // Need to create a mapping for which type a peripheral is on

  // extern const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Pull::NUM_OPTIONS )> PullMap;
  // ... Additional Mappings

  /*-------------------------------------------------
  Module Functions
  -------------------------------------------------*/
  /**
   *  Initializes memory associated with mapping
   *
   *  @return void
   */
  void initializeMapping();
  
}    // namespace Thor::LLD::TIMER

#endif /* !THOR_HW_TIMER_MAPPING_HPP */
