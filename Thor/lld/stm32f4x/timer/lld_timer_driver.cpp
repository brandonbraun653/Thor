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


/* Thor Includes */
#include <Thor/timer>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_TIMER )

namespace Thor::LLD::Timer
{
  size_t millis()
  {
    #pragma message("Millis won't work yet")
    return 0;
  }

  void delayMilliseconds( const size_t ms )
  {
    #pragma message("delayMilliseconds won't work yet")
  }

  void delayMicroseconds( const size_t us )
  {
    #pragma message("delayMicroseconds won't work yet")
  }
}

#endif  /* TARGET_STM32F4 && THOR_LLD_TIMER */