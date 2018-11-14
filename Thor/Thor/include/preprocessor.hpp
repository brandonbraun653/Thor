#pragma once
#ifndef THOR_PREPROCESSOR_HPP
#define THOR_PREPROCESSOR_HPP

/*-----------------------------------------------------
Check for supported compiler options:
https://clang.llvm.org/docs/LanguageExtensions.html
-----------------------------------------------------*/
#ifndef __has_include
    #error FATAL ERROR: Please use a compiler that supports __has_include(), such as Clang or MSVC 2015 Update 2 or higher
#endif

#if !__cpp_exceptions
    #error FATAL ERROR: Please enable exceptions. This is required for some of the Boost libraries.
#endif

/*-----------------------------------------------------
Check for Project Libraries
-----------------------------------------------------*/

/*-------------------------------------------
STM32
-------------------------------------------*/
#if __has_include("stm32f7xx.h")			/* STM32F7 SERIES DEVICES */
    #if __has_include("stm32f7xx_hal.h")
        #define TARGET_STM32F7
        #define HAL_AVAILABLE
        #define CORTEX_M7
    #else
        #error Please include the HAL driver for STM32F7
    #endif

#elif __has_include("stm32f4xx.h")			/* STM32F4 SERIES DEVICES */
    #if __has_include("stm32f4xx_hal.h")
        #define TARGET_STM32F4
        #define HAL_AVAILABLE
        #define CORTEX_M4
    #else
        #error Please include the HAL driver for STM32F4
    #endif

#else
    #error Target Device Not Supported Yet
#endif

#ifndef USE_FULL_LL_DRIVER
    #error Please define USE_FULL_LL_DRIVER in the project properties preprocessor tab.
#endif

/*-------------------------------------------
FreeRTOS
-------------------------------------------*/
#if __has_include("FreeRTOS.h")
    #if !defined(USING_FREERTOS)
        #define USING_FREERTOS
    #endif
#endif

/*-------------------------------------------
Chimera
-------------------------------------------*/
#if __has_include("Chimera/chimera.hpp")
    #ifndef USING_CHIMERA
        #define USING_CHIMERA
    #endif
#endif

/*-------------------------------------------
ERPC
-------------------------------------------*/
#if __has_include("Thor/erpc/config/erpc_config.h")
    #ifndef USING_ERPC
        #define USING_ERPC
    #endif
#endif

/*-------------------------------------------
Visual GDB
-------------------------------------------*/
#if __has_include("FastSemihosting.cpp") || __has_include("InstrumentingProfiler.cpp") || __has_include("SamplingProfiler.cpp")
    #ifndef USING_VISUALGDB_PROFILER
        #define USING_VISUALGDB_PROFILER
    #endif

    #ifndef VISUALGDB_PROJECT
        #define VISUALGDB_PROJECT
    #endif
#endif


#endif