/********************************************************************************
 *  File Name:
 *    hw_nvic_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_HW_DRIVER_NVIC_PROJECT_HPP
#define THOR_HW_DRIVER_NVIC_PROJECT_HPP

/* Driver Includes */
#include <Thor/hld/common/preprocessor.hpp>

#if defined( CORTEX_M4 ) && defined( EMBEDDED )
#include <Thor/lld/common/cmsis/core/include/core_cm4.h>
#elif defined( _SIM )
#include <Thor/lld/common/cmsis/core/include/cmsis_windows.h>
#else
#pragma message( "Unknown NVIC core definitions" )
#endif

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/nvic/variant/hw_nvic_register_stm32f446xx.hpp>
#endif

#endif /* !THOR_HW_DRIVER_NVIC_HPP */