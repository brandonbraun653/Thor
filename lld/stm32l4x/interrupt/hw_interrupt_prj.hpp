/******************************************************************************
 *  File Name:
 *    hw_interrupt_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_INTERRUPT_PROJECT_HPP
#define THOR_HW_INTERRUPT_PROJECT_HPP

#if defined( STM32L432xx )
#include <Thor/lld/stm32l4x/interrupt/variant/hw_interrupt_register_stm32l432kc.hpp>
#endif

#endif /* !THOR_HW_INTERRUPT_PROJECT_HPP */
