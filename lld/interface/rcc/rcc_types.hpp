/******************************************************************************
 *  File Name:
 *    rcc_types.hpp
 *
 *  Description:
 *    Common LLD RCC types
 *
 *  2019-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_RCC_TYPES_HPP
#define THOR_RCC_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/clock>
#include <Chimera/common>
#include <Thor/lld/common/types.hpp>

namespace Thor::LLD::RCC
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class SystemClock;
  class PeripheralController;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t INVALID_CLOCK = 0xBAAAAAAD;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   *  Types of PLLs found in STM32 based systems
   */
  enum class PLLType : uint8_t
  {
    CORE,
    I2S,
    SAI,

    NUM_OPTIONS,
    INVALID
  };

  /**
   *  Possible outputs found in STM32 PLLs
   */
  enum class PLLOut : uint8_t
  {
    P,
    Q,
    R,

    NUM_OPTIONS,
    INVALID
  };


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   *  Configuration struct for the clock enable register
   */
  struct RegisterConfig
  {
    Reg32_t           mask; /**< Bit mask that will enable/disable the peripheral's clock */
    volatile Reg32_t *reg;  /**< Clock enable register */
  };


  struct ExternalOscillator
  {
    size_t HSEFrequency; /**< High speed external clock */
    size_t LSEFrequency; /**< Low speed external clock */
  };


  /**
   *  Peripheral Control & Config (PCC)
   *  Describes a generic set of registers and configurations for a
   *  peripheral type that allows the RCC driver to generically configure
   *  a large number of peripherals by referencing these lookup tables.
   */
  struct PCC
  {
    const uint8_t              pType;       /**< Peripheral type */
    const uint8_t              elements;    /**< Number of elements in the tables */
    const uint8_t              bfControl;   /**< Control flags if needed */
    const uint8_t              reserved;    /**< Reserved data for alignment */
    const RegisterConfig      *clock;       /**< Lookup Table Pointer: Standard clock configuration registers */
    const RegisterConfig      *clockLP;     /**< Lookup Table Pointer: Low power clock configuration registers */
    const RegisterConfig      *reset;       /**< Lookup Table Pointer: Peripheral reset registers */
    const Chimera::Clock::Bus *clockSource; /**< Lookup Table Pointer: Which system clock is used on the peripheral */

    /**
     *  Function pointer to look up a resource index given the
     *  address of a peripheral. This will be used to access
     *  the lookup tables assigned elsewhere in this structure.
     *
     *  @param[in]  address         The peripheral address
     *  @return RIndex_t
     */
    RIndex_t ( *getResourceIndex )( const std::uintptr_t address );
  };

}    // namespace Thor::LLD::RCC

#endif /* !THOR_RCC_TYPES_HPP */
