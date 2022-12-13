/******************************************************************************
 *  File Name:
 *    exti_interrupt_vectors.hpp
 *
 *  Description:
 *    Provides ISR routine function prototypes that are common between all STM32
 *    families. These actually need to be implemented in whichever driver is
 *    used for Thor.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_EXTI_INTERRUPT_VECTORS_HPP
#define THOR_LLD_EXTI_INTERRUPT_VECTORS_HPP

#if defined( __cplusplus )
extern "C"
{
#endif

  void EXTI0_IRQHandler();
  void EXTI1_IRQHandler();
  void EXTI2_IRQHandler();
  void EXTI3_IRQHandler();
  void EXTI4_IRQHandler();
  void EXTI9_5_IRQHandler();
  void EXTI15_10_IRQHandler();

#if defined( __cplusplus )
}
#endif


#endif  /* !THOR_LLD_EXTI_INTERRUPT_VECTORS_HPP */
