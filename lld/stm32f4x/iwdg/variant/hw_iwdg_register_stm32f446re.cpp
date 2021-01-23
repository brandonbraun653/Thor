/********************************************************************************
 *  File Name:
 *    hw_iwdg_register_stm32f446re.cpp
 *
 *  Description:
 *    Explicit STM32F446xx IWDG data and routines
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/clock>
#include <Chimera/watchdog>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/stm32f4x/iwdg/hw_iwdg_types.hpp>
#include <Thor/lld/stm32f4x/iwdg/variant/hw_iwdg_register_stm32f446re.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_IWDG ) && defined( STM32F446xx )

namespace Thor::LLD::IWDG
{
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_IWDG */
