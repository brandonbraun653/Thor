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
#include <Thor/timer>


namespace Thor::TIMER
{
  // Tracks if the module data has been initialized correctly
  static size_t s_driver_initialized;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initializeModule()
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

  Chimera::Status_t resetModule()
  {
    return Chimera::Status::OK;
  }

  size_t millis()
  {
    return Thor::LLD::TIMER::millis();
  }

  size_t micros()
  {
    return Thor::LLD::TIMER::micros();
  }

  void delayMilliseconds( const size_t ms )
  {
    Thor::LLD::TIMER::delayMilliseconds( ms );
  }

  void delayMicroseconds( const size_t us )
  {
    Thor::LLD::TIMER::delayMicroseconds( us );
  }

  void blockDelayMillis( const size_t ms )
  {
    Thor::LLD::TIMER::blockDelayMillis( ms );
  }

  void blockDelayMicros( const size_t us )
  {
    Thor::LLD::TIMER::blockDelayMicros( us );
  }
}    // namespace Thor::TIMER
