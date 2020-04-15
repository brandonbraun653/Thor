/********************************************************************************
 *  File Name:
 *    hw_power_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_POWER_MAPPING_HPP
#define THOR_HW_POWER_MAPPING_HPP

/* STL Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container>
#include <Chimera/power>

/* Driver Includes */
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_types.hpp>

namespace Thor::LLD::PWR
{
  /*-------------------------------------------------
  Define Memory Mapped Structs for Peripheral
  -------------------------------------------------*/
#if defined( STM32_POWER1_PERIPH_AVAILABLE )
  extern RegisterMap *POWER1_PERIPH;
#endif

  /*------------------------------------------------
  Hardware Memory Mappings
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;

  /*-------------------------------------------------
  Module Functions
  -------------------------------------------------*/
  /**
   *  Initializes memory associated with mapping
   *
   *  @return void
   */
  void initializeMapping();
  
}    // namespace Thor::LLD::POWER

#endif /* !THOR_HW_POWER_MAPPING_HPP */
