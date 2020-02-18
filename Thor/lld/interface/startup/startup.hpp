/********************************************************************************
 *  File Name:
 *    startup.hpp
 *  
 *  Description:
 *    Includes chip level startup functions
 *  
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_STARTUP_CONFIG_HPP
#define THOR_STARTUP_CONFIG_HPP

#include <Thor/preprocessor.hpp>

/*-------------------------------------------------
Using the custom STM32 driver
-------------------------------------------------*/
#if defined( THOR_CUSTOM_DRIVERS ) && ( THOR_CUSTOM_DRIVERS == 1 )

#if defined( TARGET_STM32F4 )
#include <Thor/drivers/f4/startup/startup_stm32f4xxxx.hpp>
#endif

/*-------------------------------------------------
Using an unsupported driver?
-------------------------------------------------*/
#else
#error Unknown Thor startup driver implementation
#endif /* THOR_CUSTOM_DRIVERS */

#endif  /* !THOR_STARTUP_CONFIG_HPP */