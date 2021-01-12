/********************************************************************************
 *  File Name:
 *    freertos_hooks_prj.cpp
 *
 *  Description:
 *    Implements Thor hooks into common FreeRTOS callback functions
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/system>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>

#if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )

namespace Chimera::Threading::FreeRTOS
{
  void ApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
  {
    Chimera::insert_debug_breakpoint();
    Chimera::System::softwareReset();
    while ( 1 ) {}
  }

  void ApplicationTickHook()
  {
    /*-------------------------------------------------
    Calculate the update rate for the SYSTick
    -------------------------------------------------*/
    constexpr size_t sysTickPeriodMS = 1000 / configTICK_RATE_HZ;
    static_assert( ( 1000 % configTICK_RATE_HZ ) == 0, "FreeRTOS tick rate must yield whole number tick intervals" );


#if defined( CORTEX_M4 )
    CortexM4::SYSTick::onTickIncrement( sysTickPeriodMS );
#else
#error "Missing SYSTick hook for target processor core"
#endif
  }

  void ApplicationMallocFailedHook()
  {
    Chimera::insert_debug_breakpoint();
    Chimera::System::softwareReset();
    while ( 1 ) {}
  }

  void ApplicationIdleHook()
  {
  }

}    // namespace Chimera::Threading::FreeRTOS


#endif /* USING_FREERTOS || USING_FREERTOS_THREADS */
