#pragma once
#ifndef INTERRUPT_H_
#define INTERRUPT_H_

/* Boost Includes */
#include <boost/function.hpp>
#include <boost/container/vector.hpp>

/* Thor Includes */
#include <Thor/macro.hpp>
#include <Thor/config.hpp>
#include <Thor/definitions.hpp>
#include <Thor/types.hpp>

#ifdef USING_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#endif

namespace Thor
{
  namespace Interrupt
  {
    // DO NOT MOVE THIS. DMA handlers currently depend on it...
    enum TriggerSource
    {
      SRC_UART,
      SRC_UART1,
      SRC_UART1_TX,
      SRC_UART1_RX,
      SRC_UART2,
      SRC_UART2_TX,
      SRC_UART2_RX,
      SRC_UART3,
      SRC_UART3_TX,
      SRC_UART3_RX,
      SRC_UART4,
      SRC_UART4_TX,
      SRC_UART4_RX,
      SRC_UART5,
      SRC_UART5_TX,
      SRC_UART5_RX,
      SRC_UART7,
      SRC_UART7_TX,
      SRC_UART7_RX,
      SRC_UART8,
      SRC_UART8_TX,
      SRC_UART8_RX,
      SRC_USART,
      SRC_USART1,
      SRC_USART1_TX,
      SRC_USART1_RX,
      SRC_USART2,
      SRC_USART2_TX,
      SRC_USART2_RX,
      SRC_USART3,
      SRC_USART3_TX,
      SRC_USART3_RX,
      SRC_USART6,
      SRC_USART6_TX,
      SRC_USART6_RX,

      SRC_SPI,
      SRC_SPI1,
      SRC_SPI2,
      SRC_SPI3,
      SRC_SPI4,
      SRC_SPI5,
      SRC_SPI6,

      TOTAL_TRIGGER_SOURCES,
      TRIGGER_SOURCE_UNKNOWN

    };

    namespace DMA
    {
      struct PeriphConfig
      {
        TriggerSource peripheral_type     = TRIGGER_SOURCE_UNKNOWN;
        TriggerSource peripheral_instance = TRIGGER_SOURCE_UNKNOWN;
        Thor::DMA::TransferDirection direction =
            Thor::DMA::TransferDirection::TRANSFER_DIRECTION_UNDEFINED;
        uint32_t channel_selection = 0u;
      };

      class DMAHandler
      {
      public:
        bool calculatePeriphSource( PeriphConfig &output );
        void updateDMASource( uint32_t periph, uint32_t stream );
        void copyRegisters( uint32_t REG_LISR, uint32_t REG_HISR, uint32_t REG_SxCR, uint32_t REG_SxPAR );
        void IRQHandler();

        DMAHandler()  = default;
        ~DMAHandler() = default;

      private:
        uint32_t DMA_Periph, DMA_Stream, DMA_Channel;
        uint32_t REG_DMA_LISR, REG_DMA_HISR;
        uint32_t REG_DMA_SxCR, REG_DMA_SxPAR;
      };

      class DMAManagerBase
      {
      public:
        /** User defined function that specifies exactly which callback to use
         *	@param[in]	pConfig		Container of information describing the DMA source and type*/
        virtual void requestCallback( PeriphConfig pConfig ) = 0;

        void attachCallback_TXDMA( size_t periphNum, func_void func );
        void attachCallback_RXDMA( size_t periphNum, func_void func );
        void removeCallback_TXDMA( size_t periphNum );
        void removeCallback_RXDMA( size_t periphNum );
        void executeCallback_TXDMA( size_t periphNum );
        void executeCallback_RXDMA( size_t periphNum );

        DMAManagerBase( const size_t numCallbacks );
        ~DMAManagerBase() = default;

      private:
        boost::container::vector<func_void> txdmaCallbacks;
        boost::container::vector<func_void> rxdmaCallbacks;
      };
    }    // namespace DMA

    namespace Serial
    {
      class Serial_DMAHandlerManager : public Interrupt::DMA::DMAManagerBase
      {
      public:
        void requestCallback( Interrupt::DMA::PeriphConfig pConfig ) override;

        Serial_DMAHandlerManager() : DMAManagerBase( Thor::Serial::MAX_SERIAL_CHANNELS + 1 ){};
        ~Serial_DMAHandlerManager() = default;

      private:
      };
    }    // namespace Serial

    namespace SPI
    {
      class SPI_DMAHandlerManager : public Interrupt::DMA::DMAManagerBase
      {
      public:
        void requestCallback( Interrupt::DMA::PeriphConfig pConfig ) override;

        SPI_DMAHandlerManager() : DMAManagerBase( Thor::SPI::MAX_SPI_CHANNELS + 1 ){};
        ~SPI_DMAHandlerManager() = default;

      private:
      };
    }    // namespace SPI
  }      // namespace Interrupt
}    // namespace Thor

extern Thor::Interrupt::Serial::Serial_DMAHandlerManager serialDMAManager;
extern Thor::Interrupt::SPI::SPI_DMAHandlerManager spiDMAManager;


#ifdef __cplusplus
extern "C"
{
#endif

#if defined( DMA1 )
  void DMA1_Stream0_IRQHandler();
  void DMA1_Stream1_IRQHandler();
  void DMA1_Stream2_IRQHandler();
  void DMA1_Stream3_IRQHandler();
  void DMA1_Stream4_IRQHandler();
  void DMA1_Stream5_IRQHandler();
  void DMA1_Stream6_IRQHandler();
  void DMA1_Stream7_IRQHandler();
#endif

#if defined( DMA2 )
  void DMA2_Stream0_IRQHandler();
  void DMA2_Stream1_IRQHandler();
  void DMA2_Stream2_IRQHandler();
  void DMA2_Stream3_IRQHandler();
  void DMA2_Stream4_IRQHandler();
  void DMA2_Stream5_IRQHandler();
  void DMA2_Stream6_IRQHandler();
  void DMA2_Stream7_IRQHandler();
#endif

  void USART1_IRQHandler();
  void USART2_IRQHandler();
  void USART3_IRQHandler();
  void USART6_IRQHandler();

  void UART1_IRQHandler();
  void UART2_IRQHandler();
  void UART3_IRQHandler();
  void UART4_IRQHandler();
  void UART5_IRQHandler();
  void UART6_IRQHandler();
  void UART7_IRQHandler();
  void UART8_IRQHandler();

  void SPI1_IRQHandler();
  void SPI2_IRQHandler();
  void SPI3_IRQHandler();
  void SPI4_IRQHandler();
  void SPI5_IRQHandler();
  void SPI6_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif
