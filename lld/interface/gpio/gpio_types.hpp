/******************************************************************************
 *  File Name:
 *    gpio_types.hpp
 *
 *  Description:
 *    Common LLD GPIO Types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_GPIO_DRIVER_TYPES_HPP
#define THOR_LLD_GPIO_DRIVER_TYPES_HPP

/* STL Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/gpio>


namespace Thor::LLD::GPIO
{
  /*---------------------------------------------------------------------------
  Foward Declarations
  ---------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   *  Effectively defines the drive strength of the GPIO output. Actual
   *  strength depends on VDD and the connected load.
   *
   *  @note Do not change these enum values as they are used to index arrays
   */
  enum class Speed : uint8_t
  {
    LOW = 0,
    MEDIUM,
    FAST,
    HIGH,
    NUM_OPTIONS,
    UNKNOWN_SPEED
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   *  Pair of values that map the Chimera alternate functions
   *  into the processor's register value
   */
  struct AlternateFunc
  {
    Reg8_t registerAltFunc;                  /**< Register value to configure AF */
    Chimera::GPIO::Alternate chimeraAltFunc; /**< Chimera value associated with AF */
  };

  /**
   *  A collection of attributes associated with a pin
   */
  struct PinAttributes
  {
    Chimera::GPIO::Pin pinID;     /**< ID of the pin this struct is describing */
    uint8_t afListSize;           /**< How many elements are in the AF list */
    const AlternateFunc *altFunc; /**< List of alternate functions associated with the pin */
  };

  /**
   *  A collection of attributes associated with a port
   */
  struct PortAttributes
  {
    Chimera::GPIO::Port portID; /**< ID of the port this struct is describing */
    uint8_t pinListSize;        /**< How many elements are in the pin list */
    const PinAttributes *pins;  /**< List of pin attributes associated with the port */
  };

}    // namespace Thor::LLD::GPIO

#endif /* !THOR_LLD_GPIO_DRIVER_TYPES_HPP */
