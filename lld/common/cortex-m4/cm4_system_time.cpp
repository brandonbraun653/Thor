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
  static size_t sMillisTick = 0u;
  static size_t sMicrosTick = 0u;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void onTickIncrement( const size_t period )
  {
    sMillisTick += period;
    sMicrosTick += period * 1000;
  }


  size_t getMilliseconds()
  {
    return sMillisTick;
  }


  size_t getMicroseconds()
  {
    /*-------------------------------------------------------------------------
    Use the delta between the reload value and the
    current value to figure out #clocks since last IRQ.

    This timer is driven by the system core clock.
    -------------------------------------------------------------------------*/
    float elapsedTicks  = static_cast<float>( *NVIC_REG_SYSTICK_LOAD - *NVIC_REG_SYSTICK_VAL );
    size_t microsOffset = static_cast<size_t>( elapsedTicks * Clock::SCCPeriod_us );

    return sMicrosTick + microsOffset;
  }
}  // namespace CortexM4
