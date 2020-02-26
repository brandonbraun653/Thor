/********************************************************************************
 *  File Name:
 *    hld_timer_driver.hpp
 *
 *  Description:
 *    Thor high level driver for Timer
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_TIMER_DRIVER_HPP 
#define THOR_HLD_TIMER_DRIVER_HPP 

/* STL Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::Timer
{
  Chimera::Status_t initialize();

  Chimera::Status_t reset();

  void incrementSystemTick();

  size_t millis();

  void delayMilliseconds( const size_t val );

  void delayMicroseconds( const size_t val );

}

#endif  /* !THOR_HLD_TIMER_DRIVER_HPP */