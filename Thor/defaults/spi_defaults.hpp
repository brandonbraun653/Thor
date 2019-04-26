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
#include <Thor/definitions/spi_definitions.hpp>
#include <Thor/types/spi_types.hpp>

namespace Thor::SPI
{
  extern const std::array<const Thor::SPI::SPIConfig *const, Thor::SPI::MAX_SPI_CHANNELS + 1> hwConfig;

#if defined( STM32F7 ) || defined( STM32F4 )
  extern const SPI_InitTypeDef dflt_SPI_Init;
  extern const DMA_InitTypeDef dflt_DMA_Init_TX;
  extern const DMA_InitTypeDef dflt_DMA_Init_RX;
#endif
}

#endif /* !THOR_DEFAULTS_HPP */
