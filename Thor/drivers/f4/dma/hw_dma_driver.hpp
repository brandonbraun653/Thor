/********************************************************************************
 *   File Name:
 *    hw_dma_driver.hpp
 *
 *   Description:
 *    STM32 Driver for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

 #pragma once
#ifndef THOR_HW_DMA_DRIVER_HPP
#define THOR_HW_DMA_DRIVER_HPP

/* C++ Includes */

/* Chimera Includes */
#include <Chimera/types/dma_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/common/interrupts/dma_interrupt_vectors.hpp>
#include <Thor/drivers/model/dma_model.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
  class Driver
  {
  public:

  protected:
    friend void (::DMA1_Stream0_IRQHandler)();
    friend void (::DMA1_Stream1_IRQHandler)();
    friend void (::DMA1_Stream2_IRQHandler)();
    friend void (::DMA1_Stream3_IRQHandler)();
    friend void (::DMA1_Stream4_IRQHandler)();
    friend void (::DMA1_Stream5_IRQHandler)();
    friend void (::DMA1_Stream6_IRQHandler)();
    friend void (::DMA1_Stream7_IRQHandler)();
    friend void (::DMA2_Stream0_IRQHandler)();
    friend void (::DMA2_Stream1_IRQHandler)();
    friend void (::DMA2_Stream2_IRQHandler)();
    friend void (::DMA2_Stream3_IRQHandler)();
    friend void (::DMA2_Stream4_IRQHandler)();
    friend void (::DMA2_Stream5_IRQHandler)();
    friend void (::DMA2_Stream6_IRQHandler)();
    friend void (::DMA2_Stream7_IRQHandler)();

  private:
  
  };
}    // namespace Thor::Driver::DMA

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
#endif /* !THOR_HW_DMA_DRIVER_HPP */
