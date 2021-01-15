/********************************************************************************
 *  File Name:
 *    hw_gpio_register_stm32f446xx.cpp
 *
 *  Description:
 *    Explicit STM32F446xx GPIO data and routines
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/gpio>
#include <Thor/lld/interface/inc/rcc>

#if defined( STM32F446xx ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
}    // namespace Thor::LLD::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
