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
#include <Chimera/thread>
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/thor.hpp>
#include <Thor/macro.hpp>
#include <Thor/print.hpp>
#include <Thor/definitions/interrupt_definitions.hpp>

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