/********************************************************************************
 *   File Name:
 *    startup_stm32f4xxxx.hpp
 *
 *   Description:
 *    High level startup information
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_STARTUP_STM32F4XXXX_HPP
#define THOR_STARTUP_STM32F4XXXX_HPP

/*------------------------------------------------
ISR vector table symbol as defined in the chip specific startup file. This 
is exported due to the Thor project being compiled into a static library. If
the symbol isn't referenced anywhere in the main project that the library
links into, it will be thrown out and the hardware effectively won't know how
to reset.
------------------------------------------------*/
extern void *g_pfnVectors;
#define THOR_SYSTEM_ISR_VECTOR_SYMBOL ( g_pfnVectors )

#endif /* !THOR_STARTUP_STM32F4XXXX_HPP */