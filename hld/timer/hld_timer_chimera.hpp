/********************************************************************************
 *  File Name:
 *    hld_timer_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for Thor Timer
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
    size_t micros();
    void delayMilliseconds( const size_t val );
    void delayMicroseconds( const size_t val );
  }    // namespace Backend
}    // namespace Chimera::Timer

#endif /* !THOR_TIMER_CHIMERA_HOOKS_HPP */
