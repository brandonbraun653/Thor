/********************************************************************************
 *   File Name:
 *    configuration.hpp
 *
 *   Description:
 *    CMSIS configuration options for the various supported Thor chips
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_CMSIS_CONFIGURATION_HPP
#define THOR_CMSIS_CONFIGURATION_HPP

/* Thor Includes */
#include <Thor/preprocessor.hpp>

#if defined( CORTEX_M4 )
#include <Thor/drivers/f4/interrupt/hw_it_prj.hpp>

#if defined( _EMBEDDED )
#define __CM4_REV 0x0001U         /*!< Core revision r0p1                            */
#define __MPU_PRESENT 1U          /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS 4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig 0U /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT 1U          /*!< FPU present */
#endif

#endif



#endif /* !THOR_CMSIS_CONFIGURATION_HPP */