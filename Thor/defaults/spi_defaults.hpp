/********************************************************************************
 * File Name:
 *   spi_defaults.hpp
 *
 * Description:
 *   Provides definitions for external hardware configuration data
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_DEFAULTS_HPP
#define THOR_SPI_DEFAULTS_HPP

/* Thor Includes */
#include <Thor/definitions/dma_definitions.hpp>
#include <Thor/definitions/spi_definitions.hpp>
#include <Thor/types/spi_types.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
namespace Thor::SPI
{
  extern const std::array<const Thor::SPI::SPIConfig *const, Thor::SPI::MAX_SPI_CHANNELS + 1> hwConfig;

#if defined( STM32F7 ) || defined( STM32F4 )
  extern const SPI_InitTypeDef dflt_SPI_Init;
  extern const DMA_InitTypeDef dflt_DMA_Init_TX;
  extern const DMA_InitTypeDef dflt_DMA_Init_RX;
#endif
  
  /*------------------------------------------------
  DMA Request Signals
  ------------------------------------------------*/
  constexpr std::array<uint8_t, Thor::SPI::MAX_SPI_CHANNELS + 1> DMARXRequestSignal = {
    DMA::Source::NONE,      DMA::Source::S_SPI1_RX, DMA::Source::S_SPI2_RX, DMA::Source::S_SPI3_RX,
    DMA::Source::S_SPI4_RX, DMA::Source::NONE,      DMA::Source::NONE
  };

  constexpr std::array<uint8_t, Thor::SPI::MAX_SPI_CHANNELS + 1> DMATXRequestSignal = {
    DMA::Source::NONE,      DMA::Source::S_SPI1_TX, DMA::Source::S_SPI2_TX, DMA::Source::S_SPI3_TX,
    DMA::Source::S_SPI4_TX, DMA::Source::NONE,      DMA::Source::NONE
  };
}
#endif 

#endif /* !THOR_DEFAULTS_HPP */
