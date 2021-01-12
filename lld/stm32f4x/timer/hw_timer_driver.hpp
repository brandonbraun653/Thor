/********************************************************************************
 *  File Name:
 *    lld_timer_driver.hpp
 *
 *  Description:
 *    Thor Low Level Timer Driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_HPP
#define THOR_LLD_TIMER_HPP

/* STL Includes */
#include <cstdlib>

namespace Thor::LLD::Timer
{
  size_t millis();
  size_t micros();
  void delayMilliseconds( const size_t ms );
  void delayMicroseconds( const size_t us );
  void blockDelayMillis( const size_t ms );
  void blockDelayMicros( const size_t ms );
}

#endif  /* !THOR_LLD_TIMER_HPP */
