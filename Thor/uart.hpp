/********************************************************************************
 * File Name:
 *   uart.hpp
 *
 * Description:
 *   UART interface for Thor
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_UART_HPP
#define THOR_UART_HPP

/* C/C++ Includes */
#include <stdlib.h>
#include <stdint.h>

/* Boost Includes */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/container/static_vector.hpp>
#pragma GCC diagnostic pop

/* Chimera Includes */
#include <Chimera/interface.hpp>

/* Thor Includes */
#include <Thor/config.hpp>
#include <Thor/definitions.hpp>
#include <Thor/defaults.hpp>
#include <Thor/gpio.hpp>
#include <Thor/interrupt.hpp>

#if defined( USING_FREERTOS )
#include "FreeRTOS.h"
#include "semphr.h"
#include <Thor/exti.hpp>
#endif

/** @namespace Thor */
namespace Thor
{
  /** @namespace Thor::UART */
  namespace UART
  {
    #if 0

    class UARTClass : public Chimera::Serial::Interface
    {
    public:
      Chimera::Status_t begin( const Modes txMode, const Modes rxMode ) final override;

      Chimera::Status_t end() final override;

      Chimera::Status_t configure( const uint32_t baud, const Chimera::Serial::CharWid width,
                                   const Chimera::Serial::Parity parity, const Chimera::Serial::StopBits stop,
                                   const Chimera::Serial::FlowControl flow ) final override;

      Chimera::Status_t setBaud( const uint32_t buad ) final override;

      Chimera::Status_t setMode( const SubPeripheral periph, const Modes mode ) final override;

      Chimera::Status_t write( const uint8_t *const buffer, const size_t length,
                               const uint32_t timeout_mS = 500 ) final override;

      Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

#if defined( USING_FREERTOS )
      Chimera::Status_t attachEventNotifier( const Event event, SemaphoreHandle_t *const semphr ) final override;

      Chimera::Status_t removeEventNotifier( const Event event, SemaphoreHandle_t *const semphr ) final override;
#endif

    private:
      UARTClass( const uint8_t channel, const Thor::Serial::SerialPins &pinConfig );

    public:
      /** A factory method to create a new UARTClass object.
       *
       *	This method intentionally replaces the typical constructor due to the need to register
       *	the shared_ptr with a static_vector that allows runtime deduction of which class to call
       *	inside of an ISR. This is done for simplicity.
       *
       *	@param[in] channel Hardware peripheral channel number (i.e. 1 for UART1, 4 for UART4, etc)
       *	@return Shared pointer to the new object
       **/
      static std::shared_ptr<UARTClass> create( const uint8_t channel, const Thor::Serial::SerialPins &pinConfig );
      ~UARTClass();

      /** Easily references buffered data for TX or RX */
      struct UARTPacket
      {
        uint8_t *data_ptr  = nullptr; /**< Contains the buffer address where data is stored */
        uint16_t bytesRead = 0;       /**< Number of bytes already read from the packet (currently used in eRPC calls) */
        size_t length      = 0;       /**< Number of bytes contained in data_ptr */
      };


    private:
      friend void(::HAL_UART_TxCpltCallback )( UART_HandleTypeDef *UsartHandle );
      friend void(::HAL_UART_RxCpltCallback )( UART_HandleTypeDef *UsartHandle );
      friend void(::UART1_IRQHandler )( void );
      friend void(::UART2_IRQHandler )( void );
      friend void(::UART3_IRQHandler )( void );
      friend void(::UART4_IRQHandler )( void );
      friend void(::UART5_IRQHandler )( void );
      friend void(::UART6_IRQHandler )( void );
      friend void(::UART7_IRQHandler )( void );
      friend void(::UART8_IRQHandler )( void );


      int uart_channel;        /**< Numerical representation of the UART instance, zero is invalid */
      bool tx_complete = true; /**< Indicates if a transmission has been completed */
      bool rx_complete = true; /**< Indicates if a reception has been completed */
      bool RX_ASYNC    = true; /**< Enables/Disables asynchronous reception of data */
      Thor::Modes txMode;      /**< Logs which mode the TX peripheral is currently in */
      Thor::Modes rxMode;      /**< Logs which mode the RX peripheral is currently in */

      struct UARTClassStatus
      {
        bool gpio_enabled              = false;
        bool uart_enabled              = false;
        bool dma_enabled_tx            = false;
        bool dma_enabled_rx            = false;
        bool uart_interrupts_enabled   = false;
        bool dma_interrupts_enabled_tx = false;
        bool dma_interrupts_enabled_rx = false;
      } UART_PeriphState; /**< Flags that allow more precise configuring of the low level hardware during init/de-init */

      /*-------------------------------
       * Object Pointers / Handles
       *------------------------------*/
      UART_HandleTypeDef uart_handle;
      DMA_HandleTypeDef hdma_uart_tx;
      DMA_HandleTypeDef hdma_uart_rx;
      Thor::GPIO::GPIOClass_sPtr tx_pin;
      Thor::GPIO::GPIOClass_sPtr rx_pin;

      /* Local copy of interrupt settings */
      IT_Initializer ITSettings_HW, ITSettings_DMA_TX, ITSettings_DMA_RX;

      /*-------------------------------
       * Low Level Setup/Teardown Functions
       *------------------------------*/
      void IRQHandler();
      void IRQHandler_TXDMA();
      void IRQHandler_RXDMA();

      void UART_GPIO_Init();
      void UART_GPIO_DeInit();

      void UART_Init();
      void UART_DeInit();
      void UART_EnableClock();
      void UART_DisableClock();
      void UART_DMA_EnableClock();
      void UART_EnableInterrupts();
      void UART_DisableInterrupts();

      void UART_DMA_Init( const Thor::SubPeripheral &periph );
      void UART_DMA_DeInit( const Thor::SubPeripheral &periph );
      void UART_DMA_EnableIT( const Thor::SubPeripheral &periph );
      void UART_DMA_DisableIT( const Thor::SubPeripheral &periph );

      /*-------------------------------
       * Error Handler Functions
       *------------------------------*/
      void UART_OverrunHandler();
    };

    #endif
  }    // namespace UART

  extern void UART_EnableIT_IDLE( UART_HandleTypeDef *UartHandle );
  extern void UART_DisableIT_IDLE( UART_HandleTypeDef *UartHandle );
  extern void UART_ClearIT_IDLE( UART_HandleTypeDef *UartHandle );

}    // namespace Thor
#endif /* !UART_H_ */
