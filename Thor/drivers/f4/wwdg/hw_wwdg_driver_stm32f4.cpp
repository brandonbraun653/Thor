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

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_WATCHDOG */
