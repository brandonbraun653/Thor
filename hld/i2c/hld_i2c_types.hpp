/******************************************************************************
 *  File Name:
 *    hld_i2c_types.hpp
 *
 *  Description:
 *    Thor HLD I2C types
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HLD_I2C_TYPES_HPP
#define THOR_HLD_I2C_TYPES_HPP



namespace Thor::I2C
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Driver;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

}  // namespace Thor::I2C

#endif  /* !THOR_HLD_I2C_TYPES_HPP */
