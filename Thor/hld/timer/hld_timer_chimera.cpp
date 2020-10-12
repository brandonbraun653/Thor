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
#include <Thor/hld/timer/hld_timer_prv_driver.hpp>
#include <Thor/lld/interface/timer/timer_detail.hpp>

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

    Chimera::Status_t registerDriver( Chimera::Timer::Backend::DriverConfig &registry )
    {
      /*-------------------------------------------------
      Some functionality will always be enabled
      -------------------------------------------------*/
      registry.isSupported       = true;
      registry.initialize        = initialize;
      registry.reset             = reset;
      registry.delayMicroseconds = delayMicroseconds;
      registry.delayMilliseconds = delayMilliseconds;
      registry.millis            = millis;
      registry.micros            = micros;

      return Chimera::Status::OK;
    }
  }    // namespace Backend

}    // namespace Chimera::Timer
     // namespace Chimera::Timer