/********************************************************************************
 *  File Name:
 *    hw_power_driver_stm32f4.cpp
 *
 *  Description:
 *    Implements the low level driver for the PWR peripheral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_PWR )

namespace Thor::LLD::PWR
{
  void initialize()
  {
    // Integration stub
  }
}    // namespace Thor::LLD::PWR

#endif /* TARGET_STM32F4 && THOR_DRIVER_PWR */
