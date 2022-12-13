/******************************************************************************
 *  File Name:
 *    adc_interrupt_vectors.hpp
 *
 *  Description:
 *    Provides ISR routine function prototypes that are common between all STM32
 *    families.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_ADC_INTERRUPT_VECTORS_HPP
#define THOR_ADC_INTERRUPT_VECTORS_HPP

#if defined( __cplusplus )
extern "C"
{
#endif

  void ADC_IRQHandler();

#if defined( __cplusplus )
}
#endif

#endif /* !THOR_ADC_INTERRUPT_VECTORS_HPP */
