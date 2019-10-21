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
#include <Thor/thor.hpp>

/* Chimera Includes */
#include <Chimera/interface/compiler_intf.hpp>
#include <Chimera/modules/freertos/freertos_hooks.hpp>

#define WIN32_LEAN_AND_MEAN
#define _X64_
#include "debugapi.h"

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
    Thor::prjIncSysTick();
  }

  void ApplicationMallocFailedHook()
  {
    DebugBreak();

    while ( 1 )
    {
    }
  }

  void ApplicationIdleHook()
  {
    
  }

}  // namespace Chimera::Modules::FreeRTOS