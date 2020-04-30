/********************************************************************************
 *  File Name:
 *    hld_timer_conversion.hpp
 *
 *  Description:
 *    Utility functions for converting STM32 style timer peripheral interfaces
 *    into the types expected by Chimera timer abstractions.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

#pragma once
#ifndef THOR_HLD_TIMER_CONVERSION_HPP
#define THOR_HLD_TIMER_CONVERSION_HPP

namespace Thor::TIMER
{
  /**
   *  Checks to see if the given timer peripheral has the associated functionality.
   *  Typically this is used as a precursor for requesting a specific controller view
   *  into a timer instance.
   *
   *  @param[in]  periph    The peripheral to check
   *  @param[in]  func      The functionality to see if the peripheral supports
   *  @return bool
   */
  bool timerHasFuntionality( const Chimera::Timer::Peripheral periph, const Chimera::Timer::Function func );

  /**
   *  Gets the internal peripheral driver and converts it to a Chimera driver
   *  that supports Timer Base functionality.
   *
   *  @note If the functionality isn't supported, a nullptr will be returned.
   *
   *  @param[in]  periph    The peripheral to get the driver for
   *  @return Chimera::Timer::ITimerBase_sPtr
   */
  Chimera::Timer::ITimerBase_sPtr getTimerAsBase( const Chimera::Timer::Peripheral periph );

  /**
   *  Gets the internal peripheral driver and converts it to a Chimera driver
   *  that supports Encoder functionality.
   *
   *  @note If the functionality isn't supported, a nullptr will be returned.
   *
   *  @param[in]  periph    The peripheral to get the driver for
   *  @return Chimera::Timer::ITimerEncoder_sPtr
   */
  Chimera::Timer::ITimerEncoder_sPtr getTimerAsENC( const Chimera::Timer::Peripheral periph );

  /**
   *  Gets the internal peripheral driver and converts it to a Chimera driver
   *  that supports Input Capture functionality.
   *
   *  @note If the functionality isn't supported, a nullptr will be returned.
   *
   *  @param[in]  periph    The peripheral to get the driver for
   *  @return Chimera::Timer::ITimerInputCapture_sPtr
   */
  Chimera::Timer::ITimerInputCapture_sPtr getTimerAsIC( const Chimera::Timer::Peripheral periph );

  /**
   *  Gets the internal peripheral driver and converts it to a Chimera driver
   *  that supports Output Compare functionality.
   *
   *  @note If the functionality isn't supported, a nullptr will be returned.
   *
   *  @param[in]  periph    The peripheral to get the driver for
   *  @return Chimera::Timer::ITimerOutputCompare_sPtr
   */
  Chimera::Timer::ITimerOutputCompare_sPtr getTimerAsOC( const Chimera::Timer::Peripheral periph );

  /**
   *  Gets the internal peripheral driver and converts it to a Chimera driver
   *  that supports One Pulse functionality.
   *
   *  @note If the functionality isn't supported, a nullptr will be returned.
   *
   *  @param[in]  periph    The peripheral to get the driver for
   *  @return Chimera::Timer::ITimerOnePulse_sPtr
   */
  Chimera::Timer::ITimerOnePulse_sPtr getTimerAsOP( const Chimera::Timer::Peripheral periph );

  /**
   *  Gets the internal peripheral driver and converts it to a Chimera driver
   *  that supports PWM functionality.
   *
   *  @note If the functionality isn't supported, a nullptr will be returned.
   *
   *  @param[in]  periph    The peripheral to get the driver for
   *  @return Chimera::Timer::ITimerPWM_sPtr
   */
  Chimera::Timer::ITimerPWM_sPtr getTimerAsPWM( const Chimera::Timer::Peripheral periph );

}  // namespace Thor::TIMER

#endif  /* !THOR_HLD_TIMER_CONVERSION_HPP */
