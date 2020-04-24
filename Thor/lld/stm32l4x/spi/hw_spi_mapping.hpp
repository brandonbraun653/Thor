/********************************************************************************
 *  File Name:
 *    hw_spi_mapping.hpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_MAPPING_HPP
#define THOR_HW_SPI_MAPPING_HPP

/* STL Includes */
#include <array>
#include <cstddef>

/* Chimera Includes */
#include <Chimera/container>
#include <Chimera/spi>

/* Driver Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_types.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_prj.hpp>

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------
  Define Memory Mapped Structs for Peripheral
  -------------------------------------------------*/
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
  extern RegisterMap *SPI1_PERIPH;
#endif

#if defined( STM32_SPI2_PERIPH_AVAILABLE )
  extern RegisterMap *SPI2_PERIPH;
#endif

#if defined( STM32_SPI3_PERIPH_AVAILABLE )
  extern RegisterMap *SPI3_PERIPH;
#endif

  /*------------------------------------------------
  Hardware Memory Mappings
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList;
  extern DriverInstanceList spiObjects;
  extern const DMASignalList RXDMASignals;
  extern const DMASignalList TXDMASignals;
  extern const IRQSignalList IRQSignals;
  extern Thor::LLD::RIndexMap InstanceToResourceIndex;
  extern Chimera::Container::LightFlatMap<Chimera::SPI::Channel, RegisterMap*> ChannelToInstance;

  /*------------------------------------------------
  Mappings from Chimera Config Options->Register Values
  ------------------------------------------------*/
  extern std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::BitOrder::NUM_OPTIONS )> BitOrderToRegConfig;
  extern std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::ClockMode::NUM_OPTIONS )> ClockModeToRegConfig;
  extern std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::ControlMode::NUM_OPTIONS )> ControlModeToRegConfig;
  extern std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::DataSize::NUM_OPTIONS )> DataSizeToRegConfig;

  /*-------------------------------------------------
  Module Functions
  -------------------------------------------------*/
  /**
   *  Initializes memory associated with mapping
   *
   *  @return void
   */
  void initializeMapping();

}    // namespace Thor::LLD::SPI

#endif /* !THOR_HW_SPI_MAPPING_HPP */
