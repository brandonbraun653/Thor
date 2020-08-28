/********************************************************************************
 *  File Name:
 *    hld_spi_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing SPI
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_CHIMERA_HOOKS_HPP
#define THOR_SPI_CHIMERA_HOOKS_HPP

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>

namespace Chimera::SPI::Backend
{
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  ISPI_sPtr getDriver( const Channel channel );
}    // namespace Chimera::SPI::Backend

#endif /* !THOR_SPI_CHIMERA_HOOKS_HPP */
