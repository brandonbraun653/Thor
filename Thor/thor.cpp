/********************************************************************************
 *  File Name:
 *    thor.cpp
 *
 *  Description:
 *    Implements core common functions to Thor
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#include <Thor/thor.hpp>
#include <Thor/macro.hpp>
#include <Thor/defaults.hpp>
#include <Thor/print.hpp>

#if defined( USING_FREERTOS )
#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
}
#endif
#endif

void ThorInit()
{
  /* This absolutely must be called first to setup the HAL system properly */
  HAL_Init();

  /* Set the clock and peripheral settings to max performance */
  ThorSystemClockConfig();

  /* Enforce the system interrupt priority structure */
  HAL_NVIC_SetPriorityGrouping( Thor::Interrupt::SYSTEM_NVIC_PRIORITY_GROUPING );


#if WRITE_BUFFERING_DISABLED
  DISABLE_WRITE_BUFFERING;
#endif


/* Set up the EXTI handler for passing messages from
 * from high priority to low priority interrupts. */
#ifdef USING_FREERTOS
  // setupEXTI0_Interrupt();
#endif

#if USE_SERIAL_DEBUG_OUTPUT && !defined( USING_VISUALGDB_PROFILER )
//  setupSTDIO();
#endif
}

void cSystemInit()
{
  ThorInit();
}

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
