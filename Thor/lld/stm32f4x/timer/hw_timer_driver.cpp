/********************************************************************************
 * File Name:
 *    lld_timer_driver.cpp
 *
 * Description:
 *    Thor low level timer driver implementation
 *
 * 2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_TIMER )

namespace Thor::LLD::Timer
{  
  static size_t systemTick = 0u;

  void incrementSystemTick()
  {
    systemTick++;
  }

  size_t millis()
  {
    return systemTick;
  }

  void delayMilliseconds( const size_t ms )
  {
    #if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
    vTaskDelay( pdMS_TO_TICKS( ms ) );
    #else
    #pragma message("delayMilliseconds() has no implementation")
    #endif
  }

  void delayMicroseconds( const size_t us )
  {
    #if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
    vTaskDelay( pdMS_TO_TICKS( us * 1000 ) );
    #else
    #pragma message("delayMicroseconds() has no implementation")
    #endif 
  }
}

#endif  /* TARGET_STM32F4 && THOR_LLD_TIMER */