#include <Thor/thor.hpp>
#include <Thor/macro.hpp>
#include <Thor/defaults.hpp>
#include <Thor/exti.hpp>
#include <Thor/print.hpp>

#if defined( USING_FREERTOS )
#include "FreeRTOS.h"
#include "task.h"
#endif

void ThorInit()
{
  /* This absolutely must be called first to setup the HAL system properly */
  HAL_Init();

  /* Set the clock and peripheral settings to max performance */
  ThorSystemClockConfig();

  /* Enforce the system interrupt priority structure */
  HAL_NVIC_SetPriorityGrouping( Thor::Defaults::Interrupt::SYSTEM_NVIC_PRIORITY_GROUPING );


#if WRITE_BUFFERING_DISABLED
  DISABLE_WRITE_BUFFERING;
#endif


/* Set up the EXTI handler for passing messages from
 * from high priority to low priority interrupts. */
#ifdef USING_FREERTOS
  setupEXTI0_Interrupt();
#endif

#if USE_SERIAL_DEBUG_OUTPUT && !defined( USING_VISUALGDB_PROFILER )
//  setupSTDIO();
#endif
}

#if defined( USING_CHIMERA )
void cSystemInit()
{
  ThorInit();
}
#endif

namespace Thor
{
  uint32_t millis()
  {
    return HAL_GetTick();
  }

  void delayMilliseconds( uint32_t ms )
  {
#if defined( USING_FREERTOS )
    vTaskDelay( pdMS_TO_TICKS( ms ) );
#else
    HAL_Delay( ms );
#endif
  }

  // TODO: use a timer for this
  void delayMicroseconds( uint32_t us )
  {
  }

}    // namespace Thor
