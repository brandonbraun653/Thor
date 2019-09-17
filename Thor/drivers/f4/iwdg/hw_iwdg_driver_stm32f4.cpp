/********************************************************************************
 *   File Name:
 *    hw_iwdg_driver_stm32f4.cpp
 *
 *   Description:
 *    Independent watchdog driver for the STM32F4 series family
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */

/* Watchdog Includes */
#include <Thor/drivers/f4/iwdg/hw_iwdg_driver.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_mapping.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_prj.hpp>
#include <Thor/drivers/f4/iwdg/hw_iwdg_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_WATCHDOG == 1 )

namespace Thor::Driver::IWDG
{

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
