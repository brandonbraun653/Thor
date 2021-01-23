/********************************************************************************
 *  File Name:
 *    lld_timer_driver.cpp
 *
 *  Description:
 *    Thor low level timer driver implementation
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>
#include <Thor/lld/common/cortex-m4/system_time.hpp>

#if defined( TARGET_STM32F4 )

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  Chimera Required Free Functions
  -------------------------------------------------------------------------------*/
  size_t millis()
  {
    return CortexM4::SYSTick::getMilliseconds();
  }


  size_t micros()
  {
    return CortexM4::SYSTick::getMicroseconds();
  }


  void delayMilliseconds( const size_t ms )
  {
#if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
    vTaskDelay( pdMS_TO_TICKS( ms ) );
#else
#pragma message( "delayMilliseconds() has no implementation" )
#endif
  }

  void delayMicroseconds( const size_t us )
  {
#if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
    vTaskDelay( pdMS_TO_TICKS( us / 1000 ) );
#else
#pragma message( "delayMicroseconds() has no implementation" )
#endif
  }

  void blockDelayMillis( const size_t ms )
  {
    size_t startTick = millis();
    while( ( millis() - startTick ) < ms )
    {
      asm volatile("nop");
    }
  }


  void blockDelayMicros( const size_t us )
  {
    size_t startTick = micros();
    while( ( micros() - startTick ) < us )
    {
      asm volatile("nop");
    }

#if defined( DEBUG )
    volatile size_t actualDiff = micros() - startTick;
    volatile int error         = static_cast<int>( us ) - static_cast<int>( actualDiff );
#endif
  }
}

#endif  /* TARGET_STM32F4 && THOR_LLD_TIMER */