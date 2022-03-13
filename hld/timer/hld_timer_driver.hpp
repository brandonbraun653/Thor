/********************************************************************************
 *  File Name:
 *    hld_timer_driver.hpp
 *
 *  Description:
 *    Thor HLD for Timer peripherals
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_TIMER_DRIVER_HPP
#define THOR_HLD_TIMER_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/event>
#include <Chimera/thread>
#include <Chimera/timer>
#include <Thor/hld/common/types.hpp>
#include <Thor/hld/timer/hld_timer_types.hpp>
#include <Thor/lld/common/types.hpp>
#include <cstdlib>

namespace Thor::TIMER
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initializeModule();
  Chimera::Status_t resetModule();
  void incrementSystemTick();
  size_t millis();
  size_t micros();
  void delayMilliseconds( const size_t val );
  void delayMicroseconds( const size_t val );
  void blockDelayMillis( const size_t ms );
  void blockDelayMicros( const size_t ms );

  namespace Factory
  {
    Chimera::Timer::ITimer *build( const Chimera::Timer::TimerInterface type, const Chimera::Timer::Instance periph );
  }
}    // namespace Thor::TIMER

#endif /* !THOR_HLD_TIMER_DRIVER_HPP */