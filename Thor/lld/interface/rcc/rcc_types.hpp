/********************************************************************************
 *  File Name:
 *    rcc_types.hpp
 *
 *  Description:
 *    Common LLD RCC types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_RCC_TYPES_HPP
#define THOR_RCC_TYPES_HPP

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD::RCC
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using ClockType_t = Reg32_t;


  /**
   *  Configuration struct for the clock enable register
   */
  struct RegisterConfig
  {
    volatile Reg32_t *reg; /**< Clock enable register */
    Reg32_t mask;          /**< Bit mask that will enable/disable the peripheral's clock */
  };


  /**
   *  Peripheral Control & Config (PCC)
   *  Describes a generic set of registers and configurations for a
   *  peripheral type that allows the RCC driver to generically configure
   *  a large number of peripherals by referencing these lookup tables.
   */
  struct PCC
  {
    const RegisterConfig *clock;            /**< Lookup Table Pointer: Standard clock configuration registers */
    const RegisterConfig *clockLP;          /**< Lookup Table Pointer: Low power clock configuration registers */
    const RegisterConfig *reset;            /**< Lookup Table Pointer: Peripheral reset registers */
    const Chimera::Clock::Bus *clockSource; /**< Lookup Table Pointer: Which system clock is used on the peripheral */
    size_t elements;                        /**< Number of elements in the tables */

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
