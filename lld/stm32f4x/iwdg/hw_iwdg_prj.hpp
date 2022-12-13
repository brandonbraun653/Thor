/******************************************************************************
 *  File Name:
 *    hw_iwdg_prj.hpp
 *
 *  Description:
 *    Imports the register definitions for the watchdog hardware.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#ifndef THOR_HW_IWDG_PROJECT_HPP
#define THOR_HW_IWDG_PROJECT_HPP

/*------------------------------------------------
All STM32L4 devices
------------------------------------------------*/
#include <Thor/lld/stm32f4x/iwdg/variant/hw_iwdg_register_stm32f4xxxx.hpp>

/*------------------------------------------------
Chip specific STM32L4 devices
------------------------------------------------*/
#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/iwdg/variant/hw_iwdg_register_stm32f446re.hpp>
#endif

#endif /* !THOR_HW_IWDG_PROJECT_HPP */