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

#if __cpp_exceptions
#error FATAL ERROR: Please disable exceptions.
#endif

/*-----------------------------------------------------
Check for Project Libraries
-----------------------------------------------------*/

/*-------------------------------------------
STM32
-------------------------------------------*/
#if __has_include( "stm32f7xx.h" ) /* STM32F7 SERIES DEVICES */
#if __has_include( "stm32f7xx_hal.h" )
#define TARGET_STM32F7
#define HAL_AVAILABLE
#define CORTEX_M7
#else
#error Please include the HAL driver for STM32F7
#endif

#elif __has_include( "stm32f4xx.h" ) /* STM32F4 SERIES DEVICES */
#if __has_include( "stm32f4xx_hal.h" )
#define TARGET_STM32F4
#define HAL_AVAILABLE
#define CORTEX_M4
#else
#error Please include the HAL driver for STM32F4
#endif

#elif !defined( EXECUTING_CPPCHECK )
#error Target Device Not Supported Yet
#endif

#ifndef USE_FULL_LL_DRIVER
#error Please define USE_FULL_LL_DRIVER in the compiler preprocessor
#endif

/*-------------------------------------------
Boost
-------------------------------------------*/
#ifndef BOOST_NO_EXCEPTIONS
#error Please define BOOST_NO_EXCEPTIONS in the compiler preprocessor
#endif

/*-------------------------------------------
CppUTest
-------------------------------------------*/
#if __has_include( "CppUTest/CommandLineTestRunner.h" )
#if !defined( CPPUTEST_MEM_LEAK_DETECTION_DISABLED )
#error Please define CPPUTEST_MEM_LEAK_DETECTION_DISABLED in the compiler preprocessor. Thor cannnot handle Unit Testing with leak detection.
#endif
#endif

/*-------------------------------------------
FreeRTOS
-------------------------------------------*/
#if __has_include( "FreeRTOS.h" ) && __has_include( "tasks.c" )
#if !defined( USING_FREERTOS )
#define USING_FREERTOS
#endif
#endif

/*-------------------------------------------
Chimera
-------------------------------------------*/
#if __has_include( "Chimera/chimera.hpp" )
#ifndef USING_CHIMERA
#define USING_CHIMERA
#endif
#endif

/*-------------------------------------------
ERPC
-------------------------------------------*/
#if __has_include( "Thor/erpc/config/erpc_config.h" )
#ifndef USING_ERPC
#define USING_ERPC
#endif
#endif

/*-------------------------------------------
Visual GDB
-------------------------------------------*/
#if __has_include( "FastSemihosting.cpp" ) && __has_include( "InstrumentingProfiler.cpp" ) &&           \
                                                             __has_include( "SamplingProfiler.cpp" ) && \
                                                                            __has_include( "ProfilerDriver_STM32_HAL.cpp" )
#ifndef USING_VISUALGDB_PROFILER
#define USING_VISUALGDB_PROFILER
#endif

#ifndef VISUALGDB_PROJECT
#define VISUALGDB_PROJECT
#endif
#endif


#endif
