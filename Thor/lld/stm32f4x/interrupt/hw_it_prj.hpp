/********************************************************************************
 *   File Name:
 *    hw_it_prj.hpp
 *
 *   Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_INTERRUPT_PROJECT_HPP
#define THOR_HW_INTERRUPT_PROJECT_HPP

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/interrupt/variant/hw_it_register_stm32f446xx.hpp>
#endif

#endif /* !THOR_HW_INTERRUPT_PROJECT_HPP */