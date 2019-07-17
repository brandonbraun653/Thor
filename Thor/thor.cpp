/********************************************************************************
 *  File Name:
 *    thor.cpp
 *
 *  Description:
 *    Implements core common functions to Thor
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/interface/compiler_intf.hpp>

/* Thor Includes */
#include <Thor/thor.hpp>
#include <Thor/macro.hpp>
#include <Thor/print.hpp>
#include <Thor/definitions/interrupt_definitions.hpp>

#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
void ThorInit()
{
  /* This absolutely must be called first to setup the HAL system properly */
#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
  HAL_Init();
#endif 

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

#endif

namespace Thor
{
  static size_t systemTick = 0u;

  WEAKDECL void prjIncSysTick()
  {
    systemTick++;
  }

  size_t millis()
  {
    // return HAL_GetTick();
    return systemTick;
  }

  void delayMilliseconds( const size_t ms )
  {
#if defined( USING_FREERTOS )
    vTaskDelay( pdMS_TO_TICKS( ms ) );
#else
    HAL_Delay( ms );
#endif
  }

  // TODO: use a timer for this
  void delayMicroseconds( const size_t us )
  {
  }
}    // namespace Thor