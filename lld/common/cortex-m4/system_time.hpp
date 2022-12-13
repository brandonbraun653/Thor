/******************************************************************************
 *  File Name:
 *    system_time.hpp
 *
 *  Description:
 *    Timing functions used with the SysTick interrupt
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef CORTEX_M4_SYSTEM_TIME_HPP
#define CORTEX_M4_SYSTEM_TIME_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/cfg>

#if defined( CORTEX_M4 )

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   *  Variable that holds the current value of the SYSCLK configuration. This value
   *  is updated anytime the clock configuration is updated. C-linkage is required
   *  for integration with FreeRTOS and some CMSIS functionality.
   */
  extern uint32_t SystemCoreClock;

#ifdef __cplusplus
}
#endif

namespace CortexM4::Clock
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   *  Updates the system wide core clock cache value
   *
   *  @param[in]  clockHz     The current system core clock frequency in Hz
   *  @return void
   */
  void updateCoreClockCache( const size_t clockHz );
}

namespace CortexM4::SYSTick
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   *  Stub function to be called somewhere inside the SYSTick ISR
   *  to increment the system's elapsed millisecond count.
   *
   *  @param[in]  period      Period of the SYSTick interrupt in milliseconds
   */
  void onTickIncrement( const size_t period );

  /**
   *  Gets the number of milliseconds elapsed since SYSTick power up. This
   *  value will overflow (on a 32-bit system) every:
   *    ~1,200 hrs
   *    ~50 days
   *
   *  @return size_t
   */
  size_t getMilliseconds();

  /**
   *  Gets the number of microseconds elapsed since SYSTick power up. This
   *  value will overflow (on a 32-bit system) every:
   *    ~4,295 sec
   *    ~71.58 min
   *    ~1.2 hr
   *
   *  @return size_t
   */
  size_t getMicroseconds();

}    // namespace CortexM4

#endif /* CORTEX_M4 */
#endif /* !CORTEX_M4_SYSTEM_TIME_HPP */
