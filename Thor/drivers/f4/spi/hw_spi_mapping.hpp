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

/* Thor Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/spi/hw_spi_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
/*------------------------------------------------
  Peripheral Memory Mapping
  ------------------------------------------------*/
  static RegisterMap *const SPI1_PERIPH = reinterpret_cast<RegisterMap *const>( SPI1_BASE_ADDR );
  static RegisterMap *const SPI2_PERIPH = reinterpret_cast<RegisterMap *const>( SPI2_BASE_ADDR );
  static RegisterMap *const SPI3_PERIPH = reinterpret_cast<RegisterMap *const>( SPI3_BASE_ADDR );
  static RegisterMap *const SPI4_PERIPH = reinterpret_cast<RegisterMap *const>( SPI4_BASE_ADDR );

  /*------------------------------------------------
  Peripheral DMA Signals
  ------------------------------------------------*/
  extern DMASignalList RXDMASignals;
  extern DMASignalList TXDMASignals;

  /*------------------------------------------------
  Low Level Driver Instances
  ------------------------------------------------*/
  extern DriverInstanceList spiObjects;

  /*------------------------------------------------
  Maps a SPI peripheral into the corresponding resource index
  ------------------------------------------------*/
  extern const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
}

#endif  /* TARGET_STM32F4 && THOR_DRIVER_SPI */
#endif  /* THOR_HW_SPI_MAPPING_HPP */
