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
#include <Chimera/threading.hpp>
#include <Chimera/types/dma_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/model/dma_model.hpp>
#include <Thor/drivers/f4/dma/hw_dma_types.hpp>
#include <Thor/drivers/common/interrupts/dma_interrupt_vectors.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
  namespace Internal
  {
    class Stream : public StreamModel, public Chimera::Threading::Lockable
    {
    public:
      Stream();
      ~Stream();

      Chimera::Status_t attach( StreamX *const peripheral, RegisterMap *const parent ) final override;

      Chimera::Status_t attachISRWakeup( SemaphoreHandle_t wakeup ) final override;

      Chimera::Status_t configure( StreamConfig *const config, TCB *const controlBlock ) final override;

      Chimera::Status_t start() final override;

      Chimera::Status_t pause() final override;

      Chimera::Status_t abort() final override;

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

      void IRQHandler(const uint8_t channel, const uint8_t request);

    private:
      StreamX *stream;
      RegisterMap *parent;
      TCB *controlBlock;
      size_t streamIndex;
      SemaphoreHandle_t wakeupSignal;
    };
  }


  class Driver : public PeripheralModel
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attach( RegisterMap *const peripheral );

    Chimera::Status_t clockEnable();

    Chimera::Status_t clockDisable();

    Chimera::Status_t reset();

    Chimera::Status_t init();

  private:
    RegisterMap *periph;
  };

}    // namespace Thor::Driver::DMA

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
#endif /* !THOR_HW_DMA_DRIVER_HPP */
