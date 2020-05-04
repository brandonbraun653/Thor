/********************************************************************************
 *  File Name:
 *    hw_flash_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_FLASH_MAPPING_HPP
#define THOR_HW_FLASH_MAPPING_HPP

/* STL Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container>

/* Driver Includes */
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_types.hpp>

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------
  Define Memory Mapped Structs for Peripheral
  -------------------------------------------------*/
#if defined( STM32_FLASH_PERIPH_AVAILABLE )
  extern RegisterMap *FLASH_PERIPH;
#endif

  /*------------------------------------------------
  Hardware Memory Mappings
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern Thor::LLD::RIndexMap InstanceToResourceIndex;

  /*------------------------------------------------
  Mappings from Chimera Config Options->Register Values
  ------------------------------------------------*/

  /*-------------------------------------------------
  Module Functions
  -------------------------------------------------*/
  /**
   *  Initializes memory associated with mapping
   *
   *  @return void
   */
  void initializeMapping();

}    // namespace Thor::LLD::FLASH

#endif /* !THOR_HW_FLASH_MAPPING_HPP */
