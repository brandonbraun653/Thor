/********************************************************************************
 *  File Name:
 *    hld_timer_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera Timer driver hooks
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/timer>
#include <Thor/cfg>
#include <Thor/hld/timer/hld_timer_chimera.hpp>
#include <Thor/lld/interface/inc/timer>
#include <Thor/timer>
#include <memory>


namespace Chimera::Timer
{
  namespace Backend
  {
    Chimera::Status_t initialize()
    {
      return Thor::TIMER::initializeModule();
    }

    Chimera::Status_t reset()
    {
      return Thor::TIMER::resetModule();
    }

    size_t millis()
    {
      return Thor::TIMER::millis();
    }

    size_t micros()
    {
      return Thor::TIMER::micros();
    }

    void delayMilliseconds( const size_t val )
    {
      Thor::TIMER::delayMilliseconds( val );
    }

    void delayMicroseconds( const size_t val )
    {
      Thor::TIMER::delayMicroseconds( val );
    }

    void blockDelayMilliseconds( const size_t val )
    {
      Thor::TIMER::blockDelayMillis( val );
    }

    void blockDelayMicroseconds( const size_t val )
    {
      Thor::TIMER::blockDelayMicros( val );
    }

    Chimera::Status_t registerDriver( Chimera::Timer::Backend::DriverConfig &registry )
    {
      registry.isSupported            = true;
      registry.initialize             = initialize;
      registry.reset                  = reset;
      registry.delayMicroseconds      = delayMicroseconds;
      registry.delayMilliseconds      = delayMilliseconds;
      registry.millis                 = millis;
      registry.micros                 = micros;
      registry.blockDelayMicroseconds = blockDelayMicroseconds;
      registry.blockDelayMilliseconds = blockDelayMilliseconds;
      registry.build                  = Thor::TIMER::Factory::build;

      return Chimera::Status::OK;
    }
  }    // namespace Backend
}    // namespace Chimera::Timer
