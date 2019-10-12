/********************************************************************************
 *   File Name:
 *    spi_types.hpp
 *
 *   Description:
 *    Common SPI types used in Thor drivers
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_DRIVER_SPI_COMMON_TYPES_HPP
#define THOR_DRIVER_SPI_COMMON_TYPES_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::Driver::SPI
{
  /**
   *  Forward declaration to ease compilation
   */
  struct RegisterMap;

  using StatusFlags_t = uint32_t;
  using ErrorFlags_t  = uint32_t;
}

#endif /* !THOR_DRIVER_SPI_COMMON_TYPES_HPP */