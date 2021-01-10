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

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>

namespace Chimera::SPI::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Channel channel );
}    // namespace Chimera::SPI::Backend

#endif /* !THOR_SPI_CHIMERA_HOOKS_HPP */
