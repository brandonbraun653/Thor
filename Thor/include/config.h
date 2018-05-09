#pragma once
#ifndef THOR_CONFIG_H_
#define THOR_CONFIG_H_
/*-------------------------------------------
* Check for supported compiler options:
* https://clang.llvm.org/docs/LanguageExtensions.html
*-------------------------------------------*/
#ifndef __has_include
#error FATAL ERROR: Please use a compiler that supports __has_include(), such as Clang or MSVC 2015 Update 2 or higher
#endif

#if !__cpp_exceptions
#error FATAL ERROR: Please enable exceptions. This is required for some of the Boost libraries.
#endif 


/*-------------------------------------------
* Check for existence of various files
*-------------------------------------------*/
/* Thor Files */
//#if !(__has_include("thor_peripherals.hpp"))
//#error FATAL ERROR: Please create a "thor_peripherals.hpp" file to define which peripherals are used in the build
//#else//#include "thor_peripherals.hpp"
//#endif

/* Chimera Files */
#if __has_include("Chimera/chimera.hpp")
#define USING_CHIMERA
#include <Chimera\chimera.hpp>
#endif 

/* STM32 Files */
#if __has_include("stm32f7xx.h")			/* STM32F7 SERIES DEVICES */
	#if __has_include("stm32f7xx_hal.h")
		#define TARGET_STM32F7
		#define HAL_AVAILABLE
		#include "stm32f7xx_hal.h"
	#else
		#error Please include the HAL driver for STM32F7
	#endif

#elif __has_include("stm32f4xx.h")			/* STM32F4 SERIES DEVICES */
	#if __has_include("stm32f4xx_hal.h")
		#define TARGET_STM32F4
		#define HAL_AVAILABLE
		#include "stm32f4xx_hal.h"
	#else	
		#error Please include the HAL driver for STM32F4
	#endif

#else
	#error Target Device Not Supported Yet
#endif

/* FreeRTOS Files */
#if __has_include("FreeRTOS.h")
	#if !defined(USING_FREERTOS)
	#warning FreeRTOS detected but not enabled in Thor libraries. To enable, add USE_FREERTOS in the project properties preprocessor tab.
	#endif

	#include "FreeRTOS.h"
	#include "FreeRTOSConfig.h"
	#if configUSE_TICK_HOOK != 1
		#warning Please set "configUSE_TICK_HOOK" in FreeRTOSConfig.h or some HAL Libs will break.
	#endif 
#endif

#if __has_include("erpc_config.h")
	#define USING_ERPC
#endif 

/*-------------------------------------------
* Check for needed preprocessor definitions 
*-------------------------------------------*/
/* Allows some of the libraries to take advantage of the low level STM32
 * driver definitions and functions. */
#ifndef USE_FULL_LL_DRIVER
#error Please define USE_FULL_LL_DRIVER in the project properties preprocessor tab.
#endif 

/* Prevents accidental undefined matrix initialization values */
#ifndef EIGEN_INITIALIZE_MATRICES_BY_ZERO
#error Please define EIGEN_INITIALIZE_MATRICES_BY_ZERO in the project properties preprocessor tab.
#endif 


#ifdef DEBUG
	/* Allows painless discovery of non-included interrupt handlers. Instead of going to the HardFault handler, the 
	 * program is redirected to a weak declaration of the missing handler and looped infinitely.*/
	#ifndef DEBUG_DEFAULT_INTERRUPT_HANDLERS
	#error Please define DEBUG_DEFAULT_INTERRUPT_HANDLERS in the project properties preprocessor tab for the DEBUG build configuration.
	#endif 
#else
	#ifndef EIGEN_NO_DEBUG
	#error Please define EIGEN_NO_DEBUG in the project properties preprocessor tab for the RELEASE build configuration.
	#endif 
#endif 

#endif