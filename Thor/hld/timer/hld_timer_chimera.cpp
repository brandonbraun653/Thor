/********************************************************************************
 *  File Name:
 *    hld_timer_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera Timer driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>
#include <Thor/hld/timer/hld_timer_chimera.hpp>

namespace Chimera::Timer::Backend
{
  Chimera::Status_t registerDriver( Chimera::Timer::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_TIMER )
    registry.isSupported       = true;
    registry.initialize        = initialize;
    registry.reset             = reset;
    registry.delayMicroseconds = delayMicroseconds;
    registry.delayMilliseconds = delayMilliseconds;
    registry.millis            = millis;
    return Chimera::CommonStatusCodes::OK;
#else
    registry.isSupported       = false;
    registry.initialize        = nullptr;
    registry.reset             = nullptr;
    registry.delayMicroseconds = nullptr;
    registry.delayMilliseconds = nullptr;
    registry.millis            = nullptr;
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
#endif /* THOR_HLD_TIMER */
  }

  Chimera::Status_t initialize()
  {
    return Thor::Timer::initialize();
  }

  Chimera::Status_t reset()
  {
    return Thor::Timer::reset();
  }

  size_t millis()
  {
    return Thor::Timer::millis();
  }

  void delayMilliseconds( const size_t val )
  {
    Thor::Timer::delayMilliseconds( val );
  }

  void delayMicroseconds( const size_t val )
  {
    Thor::Timer::delayMicroseconds( val );
  }

}    // namespace Chimera::Timer::Backend