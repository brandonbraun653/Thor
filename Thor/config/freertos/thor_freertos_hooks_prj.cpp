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
#include <Thor/headers.hpp>
#include <Thor/thor.hpp>

namespace Chimera::Threading::FreeRTOS
{
  void ApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
  {
    CHIMERA_INSERT_BREAKPOINT;
    while ( 1 ) {}
  }

  void ApplicationTickHook()
  {
    Thor::prjIncSysTick();
  }

  void ApplicationMallocFailedHook()
  {
    CHIMERA_INSERT_BREAKPOINT;
    while ( 1 ) {}
  }

  void ApplicationIdleHook()
  {
  }

}    // namespace Chimera::Threading::FreeRTOS
