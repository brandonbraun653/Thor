/********************************************************************************
 *  File Name:
 *    dma.hpp
 *
 *  Description:
 *    Thor implementation of the DMA driver
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_HPP
#define THOR_DMA_HPP

/* C++ Includes */
#include <array>

/* Boost Includes */
#include <boost/function.hpp>

/* Thor Includes */
#include <Thor/definitions/dma_definitions.hpp>
#include <Thor/definitions/serial_definitions.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined( DMA1 )
  extern void DMA1_Stream0_IRQHandler();
  extern void DMA1_Stream1_IRQHandler();
  extern void DMA1_Stream2_IRQHandler();
  extern void DMA1_Stream3_IRQHandler();
  extern void DMA1_Stream4_IRQHandler();
  extern void DMA1_Stream5_IRQHandler();
  extern void DMA1_Stream6_IRQHandler();
  extern void DMA1_Stream7_IRQHandler();
#endif

#if defined( DMA2 )
  extern void DMA2_Stream0_IRQHandler();
  extern void DMA2_Stream1_IRQHandler();
  extern void DMA2_Stream2_IRQHandler();
  extern void DMA2_Stream3_IRQHandler();
  extern void DMA2_Stream4_IRQHandler();
  extern void DMA2_Stream5_IRQHandler();
  extern void DMA2_Stream6_IRQHandler();
  extern void DMA2_Stream7_IRQHandler();
#endif

#ifdef __cplusplus
}
#endif

namespace Thor::DMA
{
  extern std::array<boost::function<void( void )>, Source::S_NUM_DMA_REQUESTORS> requestHandlers;
}    // namespace Thor::DMA

#endif /* !THOR_DMA_HPP */
