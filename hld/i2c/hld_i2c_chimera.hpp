/******************************************************************************
 *  File Name:
 *    hld_i2c_chimera.hpp
 *
 *  Description:
 *    Chimera interface for I2C
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_I2C_CHIMERA_HOOKS_HPP
#define THOR_I2C_CHIMERA_HOOKS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/i2c>


namespace Chimera::I2C::Backend
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Driver_rPtr getDriver( const Channel channel );

}  // namespace Thor::I2C

#endif  /* !THOR_I2C_CHIMERA_HOOKS_HPP */
