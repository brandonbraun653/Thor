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
  class Driver;
  using Driver_sPtr = std::shared_ptr<Driver>;
}

#endif /* !THOR_HLD_SPI_TYPES_HPP */
