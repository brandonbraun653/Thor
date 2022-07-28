/********************************************************************************
 *  File Name:
 *    timer_interrupt_vectors.hpp
 *
 *  Description:
 *    Provides ISR routine function prototypes that are common between all STM32
 *    families.
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_TIMER_INTERRUPT_VECTORS_HPP
#define THOR_TIMER_INTERRUPT_VECTORS_HPP

#if defined( __cplusplus )
extern "C"
{
#endif

void TIM1_BRK_TIM9_IRQHandler();
void TIM1_UP_TIM10_IRQHandler();
void TIM1_TRG_COM_TIM11_IRQHandler();
void TIM1_CC_IRQHandler();
void TIM2_IRQHandler();
void TIM3_IRQHandler();
void TIM6_IRQHandler();
void TIM7_IRQHandler();
void LPTIM1_IRQHandler();
void LPTIM2_IRQHandler();

#if defined( __cplusplus )
}
#endif

#endif /* !THOR_TIMER_INTERRUPT_VECTORS_HPP */
