/********************************************************************************
 *  File Name:
 *    hld_timer_driver.hpp
 *
 *	 Description:
 *    Thor High Level Driver 
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_TIMER_CHIMERA_HOOKS_HPP
#define THOR_TIMER_CHIMERA_HOOKS_HPP

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

namespace Chimera::Timer
{
  namespace Backend
  {
    Chimera::Status_t initialize();

    Chimera::Status_t reset();

    size_t millis();

    void delayMilliseconds( const size_t val );

    void delayMicroseconds( const size_t val );
  }

  
}    // namespace Chimera::TIMER::Backend

#endif /* !THOR_TIMER_CHIMERA_HOOKS_HPP */