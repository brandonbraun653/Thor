/********************************************************************************
 *   File Name:
 *    hw_uart_stubs.hpp
 *
 *   Description:
 *    Additional definitions needed for Thor to compile, but not actually used by
 *    the STM32F4 drivers.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_UART_STUBS_HPP
#define THOR_HW_UART_STUBS_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )

extern void UART7_IRQHandler();
extern void UART8_IRQHandler();

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
#endif /* !THOR_HW_UART_TYPES_HPP */