#pragma once
#ifndef THOR_USART_HPP
#define THOR_USART_HPP

/* C/C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <memory>

/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/circular_buffer.hpp>

/* Thor Includes */
#include <Thor/config.hpp>
#include <Thor/definitions.hpp>
#include <Thor/defaults.hpp>
#include <Thor/gpio.hpp>
#include <Thor/interrupt.hpp>

#if 0
/** @namespace Thor */
namespace Thor
{
  /** @namespace Thor::Peripheral::USART */
  namespace USART
  {
    class USARTClass;

    typedef std::shared_ptr<USARTClass> USARTClass_sPtr;
    typedef std::unique_ptr<USARTClass> USARTClass_uPtr;

    class USARTClass : public Thor::Serial::SerialInterface
    {
    public:
      /**
       *   Initializes with a given baud rate and TX/RX modes.
       *
       *	@param[in]  baud	    Desired baud rate. Accepts standard rates from Thor::Serial::BaudRate
       *	@param[in]  tx_mode	    Sets the TX mode to Blocking, Interrupt, or DMA from Thor::Serial::Modes
       *	@param[in]  rx_mode	    Sets the RX mode to Blocking, Interrupt, or DMA from Thor::Serial::Modes
       *	@return	Thor::Status
       */
      Thor::Status begin( const uint32_t baud = 115200u, const Thor::Modes tx_mode = Thor::Modes::BLOCKING,
                          const Thor::Modes rx_mode = Thor::Modes::BLOCKING ) override;

      /**
       *   Places the specified peripheral into a given mode
       *
       *   @param[in]  periph	    Explicitly states which peripheral subsystem (TX or RX) to set from
       *Thor::Peripheral::Serial::SubPeripheral
       *	@param[in]  mode	    The corresponding mode for the peripheral to enter, from Thor::Peripheral::Serial::Modes
       *	@return	Thor::Status
       *
       *	@note When setting the RX peripheral to IT or DMA mode, it automatically enables asynchronous data reception
       */
      Thor::Status setMode( const Thor::SubPeripheral periph, const Thor::Modes mode ) override;

      /**
       *   Sets a new baud rate
       *
       *   @param[in]  baud        Buad rate to be set
       *   @return Thor::Status
       */
      Thor::Status setBaud( const uint32_t baud ) override;

      /**
       *   Writes data to the serial output gpio
       *
       *	@param[in]  val		    Pointer to an array of data to be sent out
       *	@param[in]  length	    The length of data to be sent out
       *	@return	Thor::Status
       */
      Thor::Status write( const uint8_t *const val, const size_t length ) override;

      /**
       *   Commands the RX peripheral to read a single transmission of known length into the provided buffer.
       *
       *   @param[out] buff	    An external buffer to write the received data to
       *	@param[in]  length	    The number of bytes to be received
       *
       *	@note Only use this for receptions that have a fixed, known length. For transmissions that last longer than
       *		  the given 'length' value, it will simply be ignored and lost forever. Poor data.
       */
      Thor::Status read( uint8_t *const buff, size_t length ) override;


      void end() override;


#if defined( USING_FREERTOS )
      /**
       *   Attaches a semaphore to a specific trigger source. When an event is triggered on that source,
       *	the semaphore will be 'given' to and any task waiting on that semaphore will become unblocked.
       *
       *	@param[in]  trig		The source to be triggered on, of type Thor::Interrupt::Trigger
       *	@param[in]  semphr	    The address of the semaphore that will be 'given' to upon triggering
       *   @return void
       **/
      void attachThreadTrigger( const Thor::Interrupt::Trigger trig, SemaphoreHandle_t *const semphr ) override;

      /**
       *   Removes a trigger source
       *
       *   @param[in]  trig	    The source to be removed, of type Thor::Interrupt::Trigger
       */
      void removeThreadTrigger( const Thor::Interrupt::Trigger trig ) override;
#endif

    private:
      USARTClass( const uint8_t channel, const Thor::Serial::SerialPins &pinConfig );

    public:
      static std::shared_ptr<USARTClass> create( const uint8_t channel, const Thor::Serial::SerialPins &pinConfig );
      ~USARTClass();

    private:
      friend void(::HAL_USART_TxCpltCallback )( USART_HandleTypeDef *UsartHandle );
      friend void(::HAL_USART_RxCpltCallback )( USART_HandleTypeDef *UsartHandle );
      friend void(::USART1_IRQHandler )( void );
      friend void(::USART2_IRQHandler )( void );
      friend void(::USART3_IRQHandler )( void );
      // friend void(::USART4_IRQHandler)(void);
      // friend void(::USART5_IRQHandler)(void);
      friend void(::USART6_IRQHandler )( void );
      // friend void(::USART7_IRQHandler)(void);
      // friend void(::USART8_IRQHandler)(void);

      int usartChannel;        /* Which peripheral hardware channel this class is mapped to (ie USART1, USART2, etc ...) */
      bool tx_complete = true; /**< Indicates if a transmission has been completed */
      bool rx_complete = true; /**< Indicates if a reception has been completed */
      bool RX_ASYNC    = true; /**< Enables/Disables asynchronous reception of data */

      Thor::Modes txMode; /**< Logs which mode the TX peripheral is currently in */
      Thor::Modes rxMode; /**< Logs which mode the RX peripheral is currently in */


      struct USARTClassStatus
      {
        bool gpio_enabled              = false;
        bool usart_enabled             = false;
        bool dma_enabled_tx            = false;
        bool dma_enabled_rx            = false;
        bool usart_interrupts_enabled  = false;
        bool dma_interrupts_enabled_tx = false;
        bool dma_interrupts_enabled_rx = false;

        bool rxOverrun = false;
      } periphState;


      USART_HandleTypeDef usartHandle;
      DMA_HandleTypeDef hdma_usart_tx;
      DMA_HandleTypeDef hdma_usart_rx;

      IT_Initializer ITSettings_HW;
      IT_Initializer ITSettings_DMA_TX;
      IT_Initializer ITSettings_DMA_RX;

      Thor::Peripheral::GPIO::GPIOClass tx_pin;
      Thor::Peripheral::GPIO::GPIOClass rx_pin;

      uint8_t *userRxBuffer;
      uint8_t *userTxBuffer;

      uint8_t rxBuffer[ Thor::USART::USART_QUEUE_BUFFER_SIZE ];
      uint8_t txBuffer[ Thor::USART::USART_QUEUE_BUFFER_SIZE ];

      void IRQHandler();
      void IRQHandler_TXDMA();
      void IRQHandler_RXDMA();


      void USART_GPIO_Init();
      void USART_GPIO_DeInit();

      void USART_Init();
      void USART_DeInit();
      void USART_EnableClock();
      void USART_DisableClock();
      void USART_DMA_EnableClock();
      void USART_EnableInterrupts();
      void USART_DisableInterrupts();

      void USART_DMA_Init( const Thor::SubPeripheral &periph );
      void USART_DMA_DeInit( const Thor::SubPeripheral &periph );
      void USART_DMA_EnableIT( const Thor::SubPeripheral &periph );
      void USART_DMA_DisableIT( const Thor::SubPeripheral &periph );

      void USART_OverrunHandler();
    };
  }    // namespace USART
}    // namespace Thor

#endif

#endif    // !USART_H_
