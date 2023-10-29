/******************************************************************************
 *  File Name:
 *    freertos_hooks_prj.cpp
 *
 *  Description:
 *    Implements Thor hooks into common FreeRTOS callback functions
 *
 *  2019-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/logging>
#include <Chimera/common>
#include <Chimera/thread>
#include <Chimera/system>
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/system_time.hpp>

#if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )

namespace Chimera::Thread::FreeRTOS
{
  void ApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
  {
    /*-------------------------------------------------------------------------
    Pull some task information
    -------------------------------------------------------------------------*/
    TaskStatus_t xTaskDetails;
    vTaskGetTaskInfo( xTask, &xTaskDetails, pdTRUE, eInvalid );

    /*-------------------------------------------------------------------------
    Alert the debugger if connected, else reset
    -------------------------------------------------------------------------*/
    Chimera::insert_debug_breakpoint();
    Chimera::System::softwareReset();
    while ( 1 ) {}
  }

  void ApplicationTickHook()
  {
    /*-------------------------------------------------------------------------
    Calculate the update rate for the SYSTick
    -------------------------------------------------------------------------*/
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
    /*-------------------------------------------------------------------------
    Put the processor into a low power state until something happens.

    // TODO: This is currently disabled because of some discoveries with USB
    // TODO: and the STM32F4. It seems that the USB peripheral is not waking
    // TODO: up the CPU from the WFI instruction, causing failed enumerations.
    -------------------------------------------------------------------------*/
#if defined( CORTEX_M4 )
    // __asm volatile( "dsb" );  /* Ensure memory transactions complete */
    // __asm volatile( "wfi" );  /* Sleep */
    // __asm volatile( "isb" );  /* Force the wfi execution before anything else */
#endif

  }

}    // namespace Chimera::Thread::FreeRTOS


#endif /* USING_FREERTOS || USING_FREERTOS_THREADS */
