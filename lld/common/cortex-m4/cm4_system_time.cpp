/******************************************************************************
 *  File Name:
 *    cm4_system_time.cpp
 *
 *  Description:
 *    Implements system time functionality with the SYSTick module
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Project Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/register.hpp>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/rcc/rcc_detail.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

  uint32_t SystemCoreClock = 16000000;

#ifdef __cplusplus
}
#endif

namespace CortexM4::Clock
{
  float SCCPeriod_ms = 0.0f;
  float SCCPeriod_us = 0.0f;
  float SCCPeriod_ns = 0.0f;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void updateCoreClockCache( const size_t clockHz )
  {
    SystemCoreClock = clockHz;
    SCCPeriod_ms = ( 1.0f / static_cast<float>( clockHz ) ) * 1000.0f;
    SCCPeriod_us = ( 1.0f / static_cast<float>( clockHz ) ) * 1000000.0f;
    SCCPeriod_ns = ( 1.0f / static_cast<float>( clockHz ) ) * 1000000000.0f;
  }
}


namespace CortexM4::SYSTick
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static volatile size_t sMillisTick = 0u;
  static volatile size_t sMicrosTick = 0u;
  static volatile size_t sLastMicros = 0u;
  static volatile size_t sLastMillis = 0u;
  static volatile size_t sPeriod = 0u;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void onTickIncrement( const size_t period )
  {
    sPeriod = period;

    /*-------------------------------------------------------------------------
    Only update if the current tick hasn't already been modified. It's quite
    possible that the `getMicroseconds` function has already updated the tick.
    This "clock" is only as accurate as the ISR gets serviced. Since it's a
    low priority ISR...it's not super accurate.
    -------------------------------------------------------------------------*/
    if( sLastMillis == sMillisTick )
    {
      sMillisTick += period;
      sMicrosTick += period * 1000;
    }

    sLastMillis = sMillisTick;
  }


  size_t getMilliseconds()
  {
    return sMillisTick;
  }


  size_t getMicroseconds()
  {
    /*-------------------------------------------------------------------------
    The SYSTICK used for system time is a down-counter, so the delta between
    the reload value and the current tick should be elapsed time.

    This timer is driven by the system core clock.
    -------------------------------------------------------------------------*/
    size_t load = *NVIC_REG_SYSTICK_LOAD;
    size_t val  = *NVIC_REG_SYSTICK_VAL;
    size_t microsOffset = static_cast<size_t>( static_cast<float>( load - val ) * Clock::SCCPeriod_us );

    /*-------------------------------------------------------------------------
    Account for the timer rollover that hasn't been captured yet in the SYSTICK
    ISR handler onTickIncrement().
    -------------------------------------------------------------------------*/
    if( sMicrosTick + microsOffset < sLastMicros )
    {
      sMillisTick += sPeriod;
      sMicrosTick += sPeriod * 1000;
    }

    /*-------------------------------------------------------------------------
    Use the most recent notion of time to calculate the current microsecond
    -------------------------------------------------------------------------*/
    sLastMicros = sMicrosTick + microsOffset;
    return sLastMicros;
  }
}  // namespace CortexM4
