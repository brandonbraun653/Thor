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

/* C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <memory>

/* Boost Includes */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#include <boost/bind.hpp>
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

namespace Thor
{
  namespace UART
  {
    class UARTClass;
    using UARTClass_sPtr = std::shared_ptr<UARTClass>;
    using UARTClass_uPtr = std::unique_ptr<UARTClass>;

    struct UARTPacket
    {
      uint8_t *data_ptr  = nullptr; /**< Contains the buffer address where data is stored */
      uint16_t bytesRead = 0; /**< Number of bytes already read from the packet (currently used in eRPC calls) */
      size_t length      = 0; /**< Number of bytes contained in data_ptr */
    };

    class UARTClass : public Chimera::Serial::Interface
    {
    public:
      ~UARTClass() = default;

      /** A factory method to create a new UARTClass object.
       *
       *	This method intentionally replaces the typical constructor due to the need to register
       *	the shared_ptr with a background manager that allows runtime deduction of which UART fired 
       *  an ISR.
       *
       *	@param[in] channel      Hardware peripheral channel number (i.e. 1 for UART1, 4 for UART4, etc)
       *	@return UARTClass_sPtr
       *
       *  |  Return Value |             Explanation             |
       *  |:-------------:|:-----------------------------------:|
       *  |  Empty Object | The channel given was out of range  |
       *  | Filled Object | The object was created successfully |
       */
      static UARTClass_sPtr create( const uint8_t channel );

      Chimera::Status_t assignHW(const uint8_t channel, Chimera::Serial::IOPins pins) final override;

      Chimera::Status_t begin( const Chimera::Serial::Modes txMode, const Chimera::Serial::Modes rxMode ) final override;

      Chimera::Status_t end() final override;

      Chimera::Status_t configure( const uint32_t baud, const Chimera::Serial::CharWid width,
                                   const Chimera::Serial::Parity parity, const Chimera::Serial::StopBits stop,
                                   const Chimera::Serial::FlowControl flow ) final override;

      Chimera::Status_t setBaud( const uint32_t baud ) final override;

      Chimera::Status_t setMode( const Chimera::Serial::SubPeripheral periph, const Chimera::Serial::Modes mode ) final override;

      Chimera::Status_t write( const uint8_t *const buffer, const size_t length,
                               const uint32_t timeout_mS = 500 ) final override;

      Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

#if defined( USING_FREERTOS )
      Chimera::Status_t attachEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr ) final override;

      Chimera::Status_t removeEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr ) final override;
#endif

      Chimera::Status_t enableBuffering(const Chimera::Serial::SubPeripheral periph, boost::circular_buffer<uint8_t> *const buffer) final override;

      Chimera::Status_t disableBuffering(const Chimera::Serial::SubPeripheral periph) final override;

    private:
      /*------------------------------------------------
      The constructor is deleted so that object creation is handled by the static 'create' method.
      In order write generic ISR handlers, there needs to be a way to determine which object fired
      the ISR. This is done by registering each class on creation with a small vector that tracks 
      instantiated UART objects. Then a runtime lookup is done via the STM32 HAL UsartHandle ID.
      ------------------------------------------------*/
      UARTClass() = default;

      /*------------------------------------------------
      Allows the C STM32 HAL and ISR functions access this class
      ------------------------------------------------*/
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
      Chimera::Serial::Modes txMode; /**< Logs which mode the TX peripheral is currently in */
      Chimera::Serial::Modes rxMode;      /**< Logs which mode the RX peripheral is currently in */

      boost::circular_buffer<uint8_t> *txBuffer;
      boost::circular_buffer<uint8_t> *rxBuffer;

      uint32_t asyncRXDataSize = 0u; /* I think this is used to track when a new unexpected Async RX starts?? */

      struct UARTClassStatus
      {
        bool gpio_enabled              = false;
        bool uart_enabled              = false;
        bool tx_buffering_enabled      = false;
        bool rx_buffering_enabled      = false;
        bool dma_enabled_tx            = false;
        bool dma_enabled_rx            = false;
        bool uart_interrupts_enabled   = false;
        bool dma_interrupts_enabled_tx = false;
        bool dma_interrupts_enabled_rx = false;
      } UART_PeriphState; /**< Flags that allow more precise configuring of the low level hardware during init/de-init */


      UART_HandleTypeDef uart_handle;
      DMA_HandleTypeDef hdma_uart_tx;
      DMA_HandleTypeDef hdma_uart_rx;
      Thor::GPIO::GPIOClass_sPtr tx_pin;
      Thor::GPIO::GPIOClass_sPtr rx_pin;

      /* Local copy of interrupt settings */
      IT_Initializer ITSettings_HW, ITSettings_DMA_TX, ITSettings_DMA_RX;

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

      void UART_DMA_Init( const Chimera::Serial::SubPeripheral &periph );
      void UART_DMA_DeInit(const Chimera::Serial::SubPeripheral &periph);
      void UART_DMA_EnableIT(const Chimera::Serial::SubPeripheral &periph);
      void UART_DMA_DisableIT(const Chimera::Serial::SubPeripheral &periph);

      void UART_OverrunHandler();
    };
  }    // namespace UART

  extern void UART_EnableIT_IDLE( UART_HandleTypeDef *UartHandle );
  extern void UART_DisableIT_IDLE( UART_HandleTypeDef *UartHandle );
  extern void UART_ClearIT_IDLE( UART_HandleTypeDef *UartHandle );

}    // namespace Thor
#endif /* !UART_H_ */
