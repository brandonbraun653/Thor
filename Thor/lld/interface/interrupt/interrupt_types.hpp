/********************************************************************************
 *   File Name:
 *    interrupt_types.hpp
 *
 *   Description:
 *    STM32 Interrupt Types
 *
 *   2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_TYPES_INTERRUPT_HPP
#define THOR_DRIVER_TYPES_INTERRUPT_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::LLD::Interrupt
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using InterruptSignal_t = int16_t;


  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  Define numerical boundaries to the number of interrupt signals
  that a given hardware peripheral can support. These must account
  for all STM32 chips past and future.
  ------------------------------------------------*/
  static constexpr InterruptSignal_t USARTSigOffset = 10;
  static constexpr InterruptSignal_t USARTMaxSig    = 10;

  static constexpr InterruptSignal_t UARTSigOffset = USARTSigOffset + USARTMaxSig + 1;
  static constexpr InterruptSignal_t UARTMaxSig    = 10;

}    // namespace Thor::LLD::Interrupt

#endif /* !THOR_DRIVER_TYPES_INTERRUPT_HPP */
