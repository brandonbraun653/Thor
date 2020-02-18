/********************************************************************************
 *   File Name:
 *    spi_definitions.hpp
 *
 *   Description:
 *    Thor SPI Definitions
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_DEFS_HPP
#define THOR_SPI_DEFS_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::SPI
{
  constexpr uint8_t MAX_SPI_CHANNELS     = 6;
  constexpr uint8_t SPI_BUFFER_SIZE      = 32;
  constexpr uint32_t BLOCKING_TIMEOUT_MS = 100;
}    // namespace Thor::SPI

#endif /* !THOR_SPI_DEFS_HPP */