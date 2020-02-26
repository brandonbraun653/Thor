/********************************************************************************
 *  File Name:
 *    timer.hpp
 *  
 *  Description:
 *    Common header for Thor TIMER that configures the driver based on which
 *    chip family is being compiled against.
 *  
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_TIMER_CONFIG_HPP
#define THOR_TIMER_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/timer/hw_timer_driver.hpp>
#endif

#endif /* !THOR_TIMER_CONFIG_HPP */