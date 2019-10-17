/********************************************************************************
 *   File Name:
 *    hw_nvic_prj.hpp
 *
 *   Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_HW_DRIVER_NVIC_PROJECT_HPP
#define THOR_HW_DRIVER_NVIC_PROJECT_HPP

/* Driver Includes */
#include <Thor/preprocessor.hpp>

#if defined( CORTEX_M4 )
#include <Thor/drivers/common/cmsis/core/include/core_cm4.h>
#endif

#if defined( STM32F446xx )
#include <Thor/drivers/f4/nvic/hw_nvic_register_stm32f446xx.hpp>
#endif

#if defined( _SIM ) && ( defined( WIN32 ) || defined( WIN64 ) )
#include <Thor/drivers/common/cmsis/core/include/cmsis_windows.h>
#endif 

#endif /* !THOR_HW_DRIVER_NVIC_HPP */