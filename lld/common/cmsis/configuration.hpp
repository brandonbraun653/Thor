/******************************************************************************
 *  File Name:
 *    configuration.hpp
 *
 *  Description:
 *    CMSIS configuration options for the various supported Thor chips
 *
 *  2019-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_CMSIS_CONFIGURATION_HPP
#define THOR_CMSIS_CONFIGURATION_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/hld/common/preprocessor.hpp>

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/interrupt/hw_interrupt_prj.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_prj.hpp>
#else
#pragma message( "Unknown target. Interrupt definitions for CMSIS will be missing." )
#endif

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
#if defined( CORTEX_M4 ) && defined( EMBEDDED )

#define __CM4_REV 0x0001U         /**< Core revision r0p1                            */
#define __MPU_PRESENT 1U          /**< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS 4U       /**< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig 0U /**< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT 1U          /**< FPU present */
#endif

#endif /* !THOR_CMSIS_CONFIGURATION_HPP */