/********************************************************************************
 *  File Name:
 *    hw_gpio_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_MAPPING_HPP
#define THOR_HW_GPIO_MAPPING_HPP

/* STL Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/container>
#include <Chimera/gpio>

/* Driver Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_types.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_types.hpp>

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------
  Define Memory Mapped Structs for Peripheral
  -------------------------------------------------*/
#if defined( STM32_GPIOA_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOA_PERIPH;
#endif

#if defined( STM32_GPIOB_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOB_PERIPH;
#endif

#if defined( STM32_GPIOC_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOC_PERIPH;
#endif

#if defined( STM32_GPIOD_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOD_PERIPH;
#endif

#if defined( STM32_GPIOE_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOE_PERIPH;
#endif

#if defined( STM32_GPIOH_PERIPH_AVAILABLE )
  extern RegisterMap *GPIOH_PERIPH;
#endif

  /*------------------------------------------------
  Hardware Memory Mappings
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;


  extern PortMap InstanceToPortMap;
  extern RIndexMap InstanceToResourceIndex;
  extern AlternateMap InstanceToAlternateMap;
  extern InstanceMap PortToInstanceMap;

  /*------------------------------------------------
  Mappings from Chimera Config Options->Register Values
  ------------------------------------------------*/
  extern const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Pull::NUM_OPTIONS )> PullMap;
  extern const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Drive::NUM_OPTIONS )> ModeMap;
  extern const std::array<uint32_t, static_cast<size_t>( Thor::LLD::GPIO::Speed::NUM_OPTIONS )> SpeedMap;
  extern const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Port::NUM_OPTIONS )> PortToIteratorMap;

  /*-------------------------------------------------
  Module Functions
  -------------------------------------------------*/
  /**
   *  Initializes memory associated with mapping
   *
   *  @return void
   */
  void initializeMapping();
  
}    // namespace Thor::LLD::GPIO

#endif /* !THOR_HW_GPIO_MAPPING_HPP */
