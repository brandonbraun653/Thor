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
  extern RegisterMap *const SPI1_PERIPH;
  extern RegisterMap *const SPI2_PERIPH;
  extern RegisterMap *const SPI3_PERIPH;
  extern RegisterMap *const SPI4_PERIPH;

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
