/********************************************************************************
 *  File Name:
 *    spi_types.hpp
 *
 *  Description:
 *    Thor SPI types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HLD_SPI_TYPES_HPP
#define THOR_HLD_SPI_TYPES_HPP

/* C++ Includes */
#include <memory>

namespace Thor::SPI
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;
}

#endif /* !THOR_HLD_SPI_TYPES_HPP */
