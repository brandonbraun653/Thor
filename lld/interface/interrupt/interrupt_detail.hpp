/******************************************************************************
 *  File Name:
 *    interrupt_detail.hpp
 *
 *  Description:
 *    Common header for Thor Interrupt that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_INTERRUPT_DETAIL_HPP
#define THOR_INTERRUPT_DETAIL_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/common/cortex-m4/interrupts.hpp>
#include <Thor/lld/stm32f4x/interrupt/hw_interrupt_prj.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/common/cortex-m4/interrupts.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_prj.hpp>
#else
#pragma message( "interrupt_detail.hpp: Unknown target for LLD" )
#endif

#endif /* THOR_INTERRUPT_DETAIL_HPP */
