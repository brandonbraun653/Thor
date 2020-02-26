/********************************************************************************
 *   File Name:
 *    hw_iwdg_prj.hpp
 *
 *   Description:
 *    Imports the register definitions for the watchdog hardware.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#ifndef THOR_HW_IWDG_PROJECT_HPP
#define THOR_HW_IWDG_PROJECT_HPP

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/iwdg/variant/hw_iwdg_register_stm32f446xx.hpp>
#endif

#endif /* !THOR_HW_IWDG_PROJECT_HPP */