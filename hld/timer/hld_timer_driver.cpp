/********************************************************************************
 *  File Name:
 *    hld_timer_driver.cpp
 *
 *  Description:
 *    Driver implementation
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/common>
#include <Thor/cfg>
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/interface/inc/timer>


namespace Chimera::Timer::Backend
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static size_t s_driver_initialized;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initializeModule()
  {
    /*-------------------------------------------------------------------------
    Prevent re-initialization from occurring
    -------------------------------------------------------------------------*/
    auto result = Chimera::Status::OK;
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return result;
    }

    /*-------------------------------------------------------------------------
    Initialize the low level drivers
    -------------------------------------------------------------------------*/
    result = Thor::LLD::TIMER::initializeModule();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }

  static Chimera::Status_t resetModule()
  {
    return Chimera::Status::OK;
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::Timer::Backend::DriverConfig &registry )
  {
    registry.isSupported            = true;
    registry.initialize             = Chimera::Timer::Backend::initializeModule;
    registry.reset                  = Chimera::Timer::Backend::resetModule;
    registry.delayMicroseconds      = Thor::LLD::TIMER::delayMicroseconds;
    registry.delayMilliseconds      = Thor::LLD::TIMER::delayMilliseconds;
    registry.millis                 = Thor::LLD::TIMER::millis;
    registry.micros                 = Thor::LLD::TIMER::micros;
    registry.blockDelayMicroseconds = Thor::LLD::TIMER::blockDelayMicros;
    registry.blockDelayMilliseconds = Thor::LLD::TIMER::blockDelayMillis;

    return Chimera::Status::OK;
  }
}    // namespace Chimera::Timer::Backend
