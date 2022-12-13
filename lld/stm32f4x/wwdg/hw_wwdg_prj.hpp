/******************************************************************************
 *  File Name:
 *    hw_wwdg_prj.hpp
 *
 *  Description:
 *    Imports the register definitions for the watchdog hardware.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_WWDG_PROJECT_HPP
#define THOR_HW_WWDG_PROJECT_HPP


/*------------------------------------------------
All STM32L4 devices
------------------------------------------------*/
#include <Thor/lld/stm32f4x/wwdg/variant/hw_wwdg_register_stm32f4xxxx.hpp>

/*------------------------------------------------
Chip specific STM32L4 devices
------------------------------------------------*/
#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/wwdg/variant/hw_wwdg_register_stm32f446re.hpp>
#endif

#endif /* !THOR_HW_WWDG_PROJECT_HPP */
