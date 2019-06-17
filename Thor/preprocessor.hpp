/********************************************************************************
 * File Name:
 *   preprocessor.hpp
 *
 * Description:
 *   Performs some preprocessor goodness to help with automatic configuration of
 *   the Thor library.
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

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

#if __cpp_exceptions && ( !defined( _WIN32 ) && !defined( _WIN64 ) )
#error FATAL ERROR: Please disable exceptions.
#endif

/*-------------------------------------------
Selects the hardware driver implementation. At the moment the possible options are 
the STM32HAL or a custom driver. If the preprocessor can't find the required
resources, an error message will be presented to the user when they try to build.

STM32 Driver:     THOR_STM32HAL_DRIVERS=1
Custom Driver:    THOR_CUSTOM_DRIVERS=1
-------------------------------------------*/
#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )

/*-------------------------------------------------
STM32HAL_F7
-------------------------------------------------*/
#if __has_include( "stm32f7/Device/include/stm32f7xx.h" )
#define TARGET_STM32F7
#define CORTEX_M7

#if !defined( STM32F767xx ) // || !defined( <some_other_chip> )
#error Please define a supported STM32F7 series device in the project preprocessor (or add the def for a new one)
#endif

#endif /* STM32F7 HAL DRIVER */

/*-------------------------------------------------
STM32HAL_F4
-------------------------------------------------*/
#if __has_include( "stm32f4/Device/include/stm32f4xx.h" ) 
#define TARGET_STM32F4
#define CORTEX_M4

#if !defined( STM32F446xx ) // || !defined( <some_other_chip> )
#error Please define a supported STM32F4 series device in the project preprocessor (or add the def for a new one)
#endif

#endif /* STM32F4 HAL DRIVER */

#if !defined( USE_FULL_LL_DRIVER ) && !defined( GMOCK_TEST )
#error Please define USE_FULL_LL_DRIVER in the compiler preprocessor
#endif

#elif defined( THOR_CUSTOM_DRIVERS ) && ( THOR_CUSTOM_DRIVERS == 1 )

/*-------------------------------------------------
Custom STM32F4 Driver
-------------------------------------------------*/
#if __has_include( "Thor/drivers/f4/common/definitions.hpp" )
#define TARGET_STM32F4
#define CORTEX_M4

#if !defined( STM32F446xx ) // || !defined( <some_other_chip> )
#error Please define a supported STM32F4 series device in the project preprocessor (or add the def for a new one)
#endif

#endif /* STM32F4 CUSTOM DRIVER */

/*-------------------------------------------------
Custom STM32F4 Driver
-------------------------------------------------*/
// TODO

#endif /* !THOR_CUSTOM_DRIVERS */

#if !defined( TARGET_STM32F4 ) && !defined( TARGET_STM32F7 )
#error No detected STM32 device. Please add to your project build system.
#endif

/*-------------------------------------------
Boost
-------------------------------------------*/
#ifndef BOOST_NO_EXCEPTIONS
/* Thor does not support exceptions due to cost overhead */
#error Please define BOOST_NO_EXCEPTIONS in the compiler preprocessor
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
#if !defined( USING_CHIMERA )
#define USING_CHIMERA
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

/*-------------------------------------------
GTest & GMock:
  It was discovered when upgrading the project to VS2019, something broke in the MSVC toolchain that caused some new compiler
  errors on previously working Thor code. There were missing definitions for several Windows types, despite the offending files
  not actually using anything related to Windows. Eventually, the fix was found on the great SOF and applied here.

  https://stackoverflow.com/questions/257134/weird-compile-error-dealing-with-winnt-h
-------------------------------------------*/
#if defined( SIM ) || defined( GMOCK_TEST )
#ifndef __wtypes_h__
#include <wtypes.h>
#endif

#ifndef __WINDEF_
#include <windef.h>
#endif

#ifndef _WINUSER_
#include <winuser.h>
#endif

#ifndef __RPC_H__
#include <rpc.h>
#endif
#endif /* GTEST_TEST || GMOCK_TEST */


#endif  /* !THOR_PREPROCESSOR_HPP */
