/********************************************************************************
 *  File Name:
 *    hld_timer_driver.cpp
 *
 *  Description:
 *    Driver implementation 
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>
#include <Thor/lld/interface/timer/timer.hpp>


#if defined( THOR_HLD_TIMER )

namespace Thor::Timer
{
  Chimera::Status_t initialize()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t reset()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  void incrementSystemTick()
  {
    Thor::LLD::Timer::incrementSystemTick();
  }

  size_t millis()
  {
    return Thor::LLD::Timer::millis();
  }

  void delayMilliseconds( const size_t ms )
  {
    Thor::LLD::Timer::delayMilliseconds( ms );
  }

  void delayMicroseconds( const size_t us )
  {
    Thor::LLD::Timer::delayMicroseconds( us );
  }

}

#endif /* THOR_HLD_TIMER */
