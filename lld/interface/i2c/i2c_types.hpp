/******************************************************************************
 *  File Name:
 *    i2c_types.hpp
 *
 *  Description:
 *    Thor I2C LLD interface types
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_I2C_TYPES_HPP
#define THOR_LLD_I2C_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/


namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  /*-----------------------------------------------------------------------------
  Enumerations
  -----------------------------------------------------------------------------*/
  enum class DMADirection : uint8_t
  {
    TX,
    RX,

    NUM_OPTIONS,
    UNKNOWN
  };


  enum class IRQHandlerIndex : uint8_t
  {
    EVENT,
    ERROR,

    NUM_OPTIONS,
    UNKNOWN,
  };
}  // namespace Thor::LLD::I2C

#endif  /* !THOR_LLD_I2C_TYPES_HPP */
