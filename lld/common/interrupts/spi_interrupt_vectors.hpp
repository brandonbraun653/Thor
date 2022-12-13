/******************************************************************************
 *  File Name:
 *    spi_interrupt_vectors.hpp
 *
 *  Description:
 *    Provides ISR routine function prototypes that are common between all STM32
 *    families. These actually need to be implemented in whichever driver is
 *    used for Thor.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_SPI_INTERRUPT_VECTORS_HPP
#define THOR_SPI_INTERRUPT_VECTORS_HPP

#if defined( __cplusplus )
extern "C"
{
#endif

  void SPI1_IRQHandler();
  void SPI2_IRQHandler();
  void SPI3_IRQHandler();
  void SPI4_IRQHandler();
  void SPI5_IRQHandler();
  void SPI6_IRQHandler();

#if defined( __cplusplus )
}
#endif

#endif /* !THOR_SPI_INTERRUPT_VECTORS_HPP */
