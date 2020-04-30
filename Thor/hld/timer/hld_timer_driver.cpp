/********************************************************************************
 *  File Name:
 *    hld_timer_driver.cpp
 *
 *  Description:
 *    Driver implementation
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>
#include <Thor/hld/timer/hld_timer_prv_driver.hpp>
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/interface/timer/timer_detail.hpp>


#if defined( THOR_HLD_TIMER )

namespace Thor::TIMER
{
  // Tracks if the module data has been initialized correctly
  static size_t s_driver_initialized;

  /*-------------------------------------------------------------------------------
  Chimera Free Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*------------------------------------------------
    Prevent re-initialization from occurring
    ------------------------------------------------*/
    auto result = Chimera::CommonStatusCodes::OK;
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return result;
    }

    /*------------------------------------------------
    Initialize driver memory
    ------------------------------------------------*/
    result |= initializeAdvanced();
    result |= initializeBasic();
    result |= initializeGeneral();
    result |= initializeLowPower();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }

  Chimera::Status_t reset()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  void incrementSystemTick()
  {
    Thor::LLD::TIMER::incrementSystemTick();
  }

  size_t millis()
  {
    return Thor::LLD::TIMER::millis();
  }

  void delayMilliseconds( const size_t ms )
  {
    Thor::LLD::TIMER::delayMilliseconds( ms );
  }

  void delayMicroseconds( const size_t us )
  {
    Thor::LLD::TIMER::delayMicroseconds( us );
  }

  /*-------------------------------------------------------------------------------
  Driver Free Functions
  -------------------------------------------------------------------------------*/
  bool isInitialized()
  {
    return s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY;
  }

}

#endif /* THOR_HLD_TIMER */
