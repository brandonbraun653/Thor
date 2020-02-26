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

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>

#if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )

namespace Chimera::Threading::FreeRTOS
{
  void ApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
  {
    Chimera::insert_debug_breakpoint();
    while ( 1 ) {}
  }

  void ApplicationTickHook()
  {
    Thor::Timer::incrementSystemTick();
  }

  void ApplicationMallocFailedHook()
  {
    Chimera::insert_debug_breakpoint();
    while ( 1 ) {}
  }

  void ApplicationIdleHook()
  {
  }

}    // namespace Chimera::Threading::FreeRTOS


#endif /* USING_FREERTOS || USING_FREERTOS_THREADS */
