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

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/startup/startup_stm32f4xxxx.hpp>
#endif

#endif  /* !THOR_STARTUP_CONFIG_HPP */