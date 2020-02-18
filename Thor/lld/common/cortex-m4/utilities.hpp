/********************************************************************************
 *   File Name:
 *    utilities.hpp
 *
 *   Description:
 *    Cortex-M4 specific device utilities
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef CORTEX_M4_UTILITIES_HPP
#define CORTEX_M4_UTILITIES_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

namespace CortexM4
{
  /**
   *  Calculates the Peripheral bit band address of a given bit inside of a
   *  register. All of this is done at compile time so there is no additional
   *  overhead.
   *
   *  @note The formula to calculate this is taken from the Cortex-M4
   *        programmers manual.
   *
   *  @param[in]  Base    The peripheral bit-band base address
   *  @param[in]  Alias   The peripheral bit-band alias base address
   *  @param[in]  Reg     The address of the register you wish to bit-band
   *  @param[in]  Bit     The bit to calculate the address for
   */
  template<typename T>
  constexpr T PBBA( T Base, T AliasBase, T Reg, T Bit )
  {
    constexpr T _bb_base_byte_offset = Reg - Base;
    constexpr T _bit_word_offset     = ( _bb_base_byte_offset * 32 ) + ( Bit * 4 );
    constexpr T _alias_address       = AliasBase + _bit_word_offset;

    return _alias_address;
  };
}    // namespace CortexM4

#endif /* !CORTEX_M4_UTILITIES_HPP */