/******************************************************************************
 * File Name:
 *   preprocessor.hpp
 *
 * Description:
 *   Performs some preprocessor goodness to help with automatic configuration of
 *   the Thor library.
 *
 * 2019-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_PREPROCESSOR_HPP
#define THOR_PREPROCESSOR_HPP

/*-----------------------------------------------------------------------------
Check for supported compiler options:
https://clang.llvm.org/docs/LanguageExtensions.html
-----------------------------------------------------------------------------*/
#ifndef __has_include
#error FATAL ERROR: Please use a compiler that supports __has_include(), such as Clang or MSVC 2015 Update 2 or higher
#endif

/*-----------------------------------------------------------------------------
Include Mock Configurations for CppUMockGen. This seems to be the only way to
get the generator to work with the Thor project structure that configures the
software using preprocessor definitions.
-----------------------------------------------------------------------------*/
#if __has_include( "CppUMockGenConfig.hpp")
#include "CppUMockGenConfig.hpp"
#endif

/*-----------------------------------------------------------------------------
Custom STM32F4 Driver
-----------------------------------------------------------------------------*/
#if defined( TARGET_STM32F4 )

#ifndef CORTEX_M4
#define CORTEX_M4
#endif

#if !defined( STM32F446xx ) // || !defined( <some_other_chip> )
#error Please define a supported STM32F4 series device in the project preprocessor (or add the def for a new one)
#endif

#endif /* TARGET_STM32F4 */

/*-----------------------------------------------------------------------------
Custom STM32L4 Driver
-----------------------------------------------------------------------------*/
#if defined( TARGET_STM32L4 )

#ifndef CORTEX_M4
#define CORTEX_M4
#endif

#if !defined( STM32L432xx ) // || !defined( <some_other_chip> )
#error Please define a supported STM32F4 series device in the project preprocessor (or add the def for a new one)
#endif

#endif /* TARGET_STM32F4 */

#if !defined( TARGET_STM32F4 ) && !defined( TARGET_STM32L4 ) && !defined( TARGET_STM32F7 )
#error No detected STM32 device. Please add to your project build system.
#endif

/*-----------------------------------------------------------------------------
Visual GDB
-----------------------------------------------------------------------------*/
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

/*-----------------------------------------------------------------------------
GTest & GMock:
  It was discovered when upgrading the project to VS2019, something broke in the MSVC toolchain that caused some new compiler
  errors on previously working Thor code. There were missing definitions for several Windows types, despite the offending files
  not actually using anything related to Windows. Eventually, the fix was found on the great SOF and applied here.

  https://stackoverflow.com/questions/257134/weird-compile-error-dealing-with-winnt-h
-----------------------------------------------------------------------------*/
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
