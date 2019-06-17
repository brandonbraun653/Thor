/********************************************************************************
 *  File Name:
 *    freertos_hooks_prj.cpp
 *
 *  Description:
 *    Implements Thor hooks into common FreeRTOS functions
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/headers.hpp>

/* Chimera Includes */
#include <Chimera/modules/freertos/freertos_hooks.hpp>

namespace Chimera::Modules::FreeRTOS
{
  void ApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
  {
    while ( 1 )
    {
    }
  }

  void ApplicationTickHook()
  {
    //HAL_IncTick();
  }

  void ApplicationMallocFailedHook()
  {
    while ( 1 )
    {
    }
  }

  void ApplicationIdleHook()
  {
    
  }

}  // namespace Chimera::Modules::FreeRTOS