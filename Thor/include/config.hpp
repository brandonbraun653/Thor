/** @file */
#pragma once
#ifndef THOR_CONFIG_HPP
#define THOR_CONFIG_HPP

//Ensures various compiler options and necessary dependencies are met before building
#include <Thor/include/preprocessor.hpp>

/*-------------------------------------------
* Include the needed headers 
*-------------------------------------------*/
#if defined(USING_CHIMERA)
#include <Chimera/chimera.hpp>
#endif

#if defined(TARGET_STM32F7)
#include "stm32f7xx_hal.h"
#endif

#if defined(TARGET_STM32F4)
#include "stm32f4xx_hal.h"
#endif

#if defined(USING_FREERTOS)
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#if configUSE_TICK_HOOK != 1
	#warning Please set "configUSE_TICK_HOOK" in FreeRTOSConfig.h or some HAL Libs will break.
#endif
#endif

/*-------------------------------------------
* Configure various embedded Thor features 
*-------------------------------------------*/

//SERIAL DEBUG SETTINGS 
#define USE_SERIAL_DEBUG_OUTPUT		1			/** Enables/Disables the use of a serial channel for rerouting printf() messages */
#define SERIAL_DEBUG_CHANNEL		1			/** Defines which default serial channel should be used for printf() */
#define SERIAL_DEBUG_BAUDRATE		115200		/** Sets the default baud rate to use for printf() */


//PROGRAM DEBUG SETTINGS

/*! @def WRITE_BUFFERING_DISABLED 
 *	@brief Disables write buffer during default memory map access. (Default 0)
 *
 *	This causes all BusFaults to be precise BusFaults, but decreases performance because any store to memory must
 *	complete before the processor can execute the next instruction.
 *
 *	@note	If the IMPRECISERR bit is set in the BFSR register on a Hard Fault, enabling this macro should cause the error to become precise, thus
 *			loading the value of the offending instruction BFAR register. Currently only supported on Cortex-M3/M4.
 */
#define WRITE_BUFFERING_DISABLED	1

#endif /* !THOR_CONFIG_HPP */