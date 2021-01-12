/********************************************************************************
 *  File Name:
 *    hw_sys_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_SYS_PROJECT_HPP
#define THOR_HW_SYS_PROJECT_HPP

/*------------------------------------------------
All STM32F4 devices
------------------------------------------------*/
//#include <Thor/lld/stm32l4x/system/variant/hw_sys_register_stm32f4xxxx.hpp>

/*------------------------------------------------
Chip specific STM32F4 devices
------------------------------------------------*/
#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/system/variant/hw_sys_register_stm32f446xx.hpp>
#endif


#endif  /* !THOR_HW_SYS_PROJECT_HPP */
