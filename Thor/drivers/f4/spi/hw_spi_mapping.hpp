/********************************************************************************
 *   File Name:
 *    hw_spi_mapping.hpp
 *
 *   Description:
 *    Useful mappings for the SPI peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_MAPPING_HPP
#define THOR_HW_SPI_MAPPING_HPP

/* Chimera Includes */
#include <Chimera/container.hpp>
#include <Chimera/types/spi_types.hpp>

/* Thor Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/spi/hw_spi_types.hpp>
#include <Thor/drivers/f4/spi/hw_spi_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
  extern RegisterMap *SPI1_PERIPH;
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
  extern RegisterMap *SPI2_PERIPH;
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
  extern RegisterMap *SPI3_PERIPH;
#endif
#if defined( STM32_SPI4_PERIPH_AVAILABLE )
  extern RegisterMap *SPI4_PERIPH;
#endif 
  
  /*------------------------------------------------
  Peripheral Memory Mapping
  ------------------------------------------------*/
  extern PeriphRegisterList PeripheralList; /**< Memory mapped structs to each SPI instance */
  extern DriverInstanceList spiObjects;     /**< Driver objects for each SPI Instance */
  extern const DMASignalList RXDMASignals;  /**< RX DMA signal identifiers for each SPI instance */
  extern const DMASignalList TXDMASignals;  /**< RX DMA signal identifiers for each SPI instance */
  extern const IRQSignalList IRQSignals;    /**< IRQ signal identifiers for each SPI instance */

  /*------------------------------------------------
  Maps a SPI peripheral into the corresponding resource index
  ------------------------------------------------*/
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
  extern Chimera::Container::LightFlatMap<size_t, RegisterMap*> ChannelToInstance;

  /*------------------------------------------------
  Maps Chimera SPI configuration options into register values
  ------------------------------------------------*/
  extern const std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::BitOrder::NUM_OPTIONS )> BitOrderToRegConfig;  
  extern const std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::ClockMode::NUM_OPTIONS )> ClockModeToRegConfig;
  extern const std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::ControlMode::NUM_OPTIONS )> ControlModeToRegConfig;
  extern const std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::DataSize::NUM_OPTIONS )> DataSizeToRegConfig;


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
  extern bool isSPI( const std::uintptr_t address );
}

#endif  /* TARGET_STM32F4 && THOR_DRIVER_SPI */
#endif  /* THOR_HW_SPI_MAPPING_HPP */
