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

/* Driver Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  Memory Mapped Structs for Peripheral
  -------------------------------------------------------------------------------*/
  extern RegisterMap *TIMER1_PERIPH;
  extern RegisterMap *TIMER2_PERIPH;
  extern RegisterMap *TIMER6_PERIPH;
  extern RegisterMap *TIMER15_PERIPH;
  extern RegisterMap *TIMER16_PERIPH;
  extern LPRegisterMap *LPTIMER1_PERIPH;
  extern LPRegisterMap *LPTIMER2_PERIPH;

#if defined( STM32_TIMER3_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER3_PERIPH;
#endif

#if defined( STM32_TIMER7_PERIPH_AVAILABLE )
  extern RegisterMap *TIMER7_PERIPH;
#endif

  /*-------------------------------------------------------------------------------
  Hardware Memory Mappings
  -------------------------------------------------------------------------------*/
  extern std::array<void *, NUM_TIMER_PERIPHS> LUT_PeripheralList;

  /*-------------------------------------------------------------------------------
  Mappings from Chimera Config Values -> LLD Register Values
  -------------------------------------------------------------------------------*/
  extern DirectionConverter LUT_Direction;

  /*-------------------------------------------------------------------------------
  Module Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initializes memory associated with mapping
   *
   *  @return void
   */
  void initializeMapping();

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_HW_TIMER_MAPPING_HPP */
