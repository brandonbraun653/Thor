/********************************************************************************
*   File Name:
*       dma_interrupt_vectors.hpp
*
*   Description:
*       Provides ISR routine function prototypes that are common between all STM32
*       families. These actually need to be implemented in whichever driver is 
*       used for Thor.
*
*   2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

#pragma once
#ifndef THOR_DMA_INTERRUPT_VECTORS_HPP
#define THOR_DMA_INTERRUPT_VECTORS_HPP

#if defined( __cplusplus )
extern "C"
{
#endif

  void DMA1_Stream0_IRQHandler();
  void DMA1_Stream1_IRQHandler();
  void DMA1_Stream2_IRQHandler();
  void DMA1_Stream3_IRQHandler();
  void DMA1_Stream4_IRQHandler();
  void DMA1_Stream5_IRQHandler();
  void DMA1_Stream6_IRQHandler();
  void DMA1_Stream7_IRQHandler();

  void DMA2_Stream0_IRQHandler();
  void DMA2_Stream1_IRQHandler();
  void DMA2_Stream2_IRQHandler();
  void DMA2_Stream3_IRQHandler();
  void DMA2_Stream4_IRQHandler();
  void DMA2_Stream5_IRQHandler();
  void DMA2_Stream6_IRQHandler();
  void DMA2_Stream7_IRQHandler();

#if defined( __cplusplus )
}
#endif

#endif /* !THOR_DMA_INTERRUPT_VECTORS_HPP */ 