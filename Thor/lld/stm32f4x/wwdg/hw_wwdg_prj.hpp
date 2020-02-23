/********************************************************************************
 *  File Name:
 *    hw_wwdg_prj.hpp
 *
 *  Description:
 *    Imports the register definitions for the watchdog hardware.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_WWDG_PROJECT_HPP
#define THOR_HW_WWDG_PROJECT_HPP

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/wwdg/variant/hw_wwdg_register_stm32f446xx.hpp>
#endif

#endif /* !THOR_HW_WWDG_PROJECT_HPP */
