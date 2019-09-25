/********************************************************************************
 *   File Name:
 *    hw_wwdg_driver_stm32f4.cpp
 *
 *   Description:
 *    Window watchdog driver for the STM32F4 series chips
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */

/* Watchdog Includes */
#include <Thor/drivers/f4/wwdg/hw_wwdg_driver.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_mapping.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_prj.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WATCHDOG == 1 )

namespace Thor::Driver::WWDG
{
  static const uint8_t counterMax    = 0x7F;
  static const uint8_t counterMin    = 0x40;
  static const uint8_t counterMask   = 0x3F;
  static const uint8_t counterRange  = counterMax - counterMin;
  static const uint8_t numPrescalers = 4;

  /**
   *   Calculates the actual watchdog timeout to the precision of 1mS
   *
   *   @param[in]  pckl1       The clock frequency of PCLK in Hz
   *   @param[in]  prescaler   The watchdog prescaler value as given in the register (0, 1, 2, 3)
   *   @param[in]  counter     The starting value of the countdown timer
   *   @return Number of milliseconds until a timeout occurs
   */
  static uint32_t calculateTimeout_mS( const uint32_t pclk1, const uint8_t prescaler, const uint8_t counter );
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
