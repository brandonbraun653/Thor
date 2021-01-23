/********************************************************************************
 *  File Name:
 *    nvic_detail.hpp
 *
 *  Description:
 *    Common header to pull in the interrupt drivers for the Cortex cores
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_CORTEX_INTERRUPT_DETAIL_HPP
#define THOR_CORTEX_INTERRUPT_DETAIL_HPP

#if defined( TARGET_STM32F4 ) || defined( TARGET_STM32L4 )
#include <Thor/lld/common/cortex-m4/interrupts.hpp>
#else
#pragma message( "nvic_detail.hpp: Unknown target for LLD" )
#endif

#endif /* THOR_CORTEX_INTERRUPT_DETAIL_HPP */
