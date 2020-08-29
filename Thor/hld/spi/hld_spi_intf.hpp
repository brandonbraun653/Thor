/********************************************************************************
 *   File Name:
 *    hld_spi_intf.hpp
 *
 *   Description:
 *    Thor SPI Definitions
 *
 *   2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_DEFS_HPP
#define THOR_SPI_DEFS_HPP

/* C++ Includes */
#include <cstddef>

namespace Thor::SPI
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t MAX_SPI_CHANNELS    = 6;
  static constexpr size_t SPI_BUFFER_SIZE     = 32;
  static constexpr size_t BLOCKING_TIMEOUT_MS = 100;
}    // namespace Thor::SPI

#endif /* !THOR_SPI_DEFS_HPP */
