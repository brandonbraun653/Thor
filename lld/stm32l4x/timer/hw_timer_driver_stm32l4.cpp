/********************************************************************************
 *  File Name:
 *    hw_timer_driver_STM32L4.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series TIMER hardware.
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/timer>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_driver.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_prj.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>

#if defined( TARGET_STM32L4 )
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

}    // namespace Thor::LLD::TIMER

#endif /* TARGET_STM32L4 && THOR_DRIVER_TIMER */
