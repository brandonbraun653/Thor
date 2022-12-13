/******************************************************************************
 *  File Name:
 *    hw_timer_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_TIMER_PROJECT_HPP
#define THOR_HW_TIMER_PROJECT_HPP

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/timer/variant/hw_timer_register_stm32f4xxxx.hpp>
#include <Thor/lld/stm32f4x/timer/variant/hw_timer_register_stm32f446xx.hpp>
#endif

#endif /* !THOR_HW_TIMER_PROJECT_HPP */
