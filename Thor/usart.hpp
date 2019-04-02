/********************************************************************************
 * File Name:
 *   usart.hpp
 *
 * Description:
 *   USART interface for Thor
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_USART_HPP
#define THOR_USART_HPP

/* C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <memory>

/* Boost Includes */
#include <boost/circular_buffer.hpp>

/* Thor Includes */
#include <Thor/definitions.hpp>
#include <Thor/gpio.hpp>

#ifdef __cplusplus
extern "C"
{
#endif
  extern void USART1_IRQHandler();
  extern void USART2_IRQHandler();
  extern void USART3_IRQHandler();
  extern void USART6_IRQHandler();
#ifdef __cplusplus
}
#endif

namespace Thor
{
  namespace USART
  {
    class USARTClass;
    using USARTClass_sPtr = std::shared_ptr<USARTClass>;
    using USARTClass_uPtr = std::unique_ptr<USARTClass>;

    class USARTClass : public Chimera::Serial::Interface
    {
    public:
      ~USARTClass();

      /**
       *  A factory method used to dynamically create a new UARTClass object.
       *
       *	This method intentionally replaces the typical constructor due to the need to register
       *	the instance with a private manager that allows runtime deduction of which UART object
       *	triggered an ISR to be called.
       *
       *	@param[in]  channel         Hardware peripheral channel number (i.e. 1 for UART1, 4 for UART4, etc)
       *  @param[in]  bufferSize      Size of the internal buffers for queuing up transfer data
       *	@return UARTClass_sPtr
       *
       *  |  Return Value |             Explanation             |
       *  |:-------------:|:-----------------------------------:|
       *  |  Empty Object | The channel given was out of range  |
       *  | Filled Object | The object was created successfully |
       */
      static USARTClass_sPtr create( const uint8_t channel, const uint16_t bufferSize = 1u );

      Chimera::Status_t assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins ) final override;

      Chimera::Status_t begin( const Chimera::Serial::Modes txMode, const Chimera::Serial::Modes rxMode ) final override;

      Chimera::Status_t end() final override;

      Chimera::Status_t configure( const uint32_t baud, const Chimera::Serial::CharWid width,
                                   const Chimera::Serial::Parity parity, const Chimera::Serial::StopBits stop,
                                   const Chimera::Serial::FlowControl flow ) final override;

      Chimera::Status_t setBaud( const uint32_t baud ) final override;

      Chimera::Status_t setMode( const Chimera::Serial::SubPeripheral periph,
                                 const Chimera::Serial::Modes mode ) final override;

      Chimera::Status_t write( const uint8_t *const buffer, const size_t length,
                               const uint32_t timeout_mS = 500 ) final override;

      Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

      Chimera::Status_t flush( const Chimera::Serial::SubPeripheral periph ) final override;

      Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len ) final override;

#if defined( USING_FREERTOS )
      Chimera::Status_t attachEventNotifier( const Chimera::Serial::Event event,
                                             SemaphoreHandle_t *const semphr ) final override;

      Chimera::Status_t removeEventNotifier( const Chimera::Serial::Event event,
                                             SemaphoreHandle_t *const semphr ) final override;
#endif

      Chimera::Status_t enableBuffering( const Chimera::Serial::SubPeripheral periph,
                                         boost::circular_buffer<uint8_t> *const buffer ) final override;

      Chimera::Status_t disableBuffering( const Chimera::Serial::SubPeripheral periph ) final override;

      bool available( size_t *const bytes = nullptr ) final override;


    private:
      /*------------------------------------------------
      The constructor is deleted so that object creation is handled by the static 'create' method.
      In order write generic ISR handlers, there needs to be a way to determine which object fired
      the ISR. This is done by registering each class on creation with a small vector that tracks
      instantiated UART objects. Then a runtime lookup is done via the STM32 HAL UsartHandle ID.
      ------------------------------------------------*/
      USARTClass();

      /*------------------------------------------------
      Allows the C STM32 HAL and ISR functions access this class
      ------------------------------------------------*/
      friend void( ::HAL_USART_TxCpltCallback )( USART_HandleTypeDef *UsartHandle );
      friend void( ::HAL_USART_RxCpltCallback )( USART_HandleTypeDef *UsartHandle );
      friend void( ::USART1_IRQHandler )( void );
      friend void( ::USART2_IRQHandler )( void );
      friend void( ::USART3_IRQHandler )( void );
      friend void( ::USART6_IRQHandler )( void );

      int usart_channel; /* Which peripheral hardware channel this class is mapped to (ie USART1, USART2, etc ...) */
      bool tx_complete       = true; /**< Indicates if a transmission has been completed */
      bool rx_complete       = true; /**< Indicates if a reception has been completed */
      bool AUTO_ASYNC_RX     = true; /**< Enables/Disables asynchronous reception of data */
      bool hardware_assigned = false;

      Chimera::Serial::Modes txMode; /**< Logs which mode the TX peripheral is currently in */
      Chimera::Serial::Modes rxMode; /**< Logs which mode the RX peripheral is currently in */

      boost::circular_buffer<uint8_t> *txUserBuffer;
      boost::circular_buffer<uint8_t> *rxUserBuffer;

      uint8_t *rxInternalBuffer;
      uint16_t rxInternalBufferSize;

      uint8_t *txInternalBuffer;
      uint16_t txInternalBufferSize;

      uint32_t asyncRXDataSize = 0u;

      uint8_t dmaRXReqSig;
      uint8_t dmaTXReqSig;

      struct USARTClassStatus
      {
        bool gpio_enabled              = false;
        bool enabled                   = false;
        bool tx_buffering_enabled      = false;
        bool rx_buffering_enabled      = false;
        bool dma_enabled_tx            = false;
        bool dma_enabled_rx            = false;
        bool interrupts_enabled        = false;
        bool dma_interrupts_enabled_tx = false;
        bool dma_interrupts_enabled_rx = false;

        bool rxOverrun = false;
      } PeripheralState;


      USART_HandleTypeDef usart_handle;
      DMA_HandleTypeDef hdma_usart_tx;
      DMA_HandleTypeDef hdma_usart_rx;

      Thor::GPIO::GPIOClass_sPtr tx_pin;
      Thor::GPIO::GPIOClass_sPtr rx_pin;

      Thor::Interrupt::Initializer ITSettings_HW;
      Thor::Interrupt::Initializer ITSettings_DMA_TX;
      Thor::Interrupt::Initializer ITSettings_DMA_RX;

      void assignRXBuffer( uint8_t *const buffer, const uint16_t size );
      void assignTXBuffer( uint8_t *const buffer, const uint16_t size );

      bool setWordLength( USART_InitTypeDef &initStruct, const Chimera::Serial::CharWid width );
      bool setParity( USART_InitTypeDef &initStruct, const Chimera::Serial::Parity parity );
      bool setStopBits( USART_InitTypeDef &initStruct, const Chimera::Serial::StopBits stopBits );

      std::array<void ( USARTClass::* )( const Chimera::Serial::SubPeripheral ), 3> modeChangeFuncPtrs;
      void setBlockingMode( const Chimera::Serial::SubPeripheral periph );
      void setInterruptMode( const Chimera::Serial::SubPeripheral periph );
      void setDMAMode( const Chimera::Serial::SubPeripheral periph );

      std::array<Chimera::Status_t ( USARTClass::* )( uint8_t *const, const size_t, const uint32_t ), 3> readFuncPtrs;
      Chimera::Status_t readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
      Chimera::Status_t readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
      Chimera::Status_t readDMA( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );

      std::array<Chimera::Status_t ( USARTClass::* )( const uint8_t *const, const size_t, const uint32_t ), 3> writeFuncPtrs;
      Chimera::Status_t writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
      Chimera::Status_t writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
      Chimera::Status_t writeDMA( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );

      void IRQHandler();
      void IRQHandler_TXDMA();
      void IRQHandler_RXDMA();

      void USART_GPIO_Init();
      void USART_GPIO_DeInit();

      Chimera::Status_t USART_Init();
      void USART_DeInit();
      void USART_EnableClock();
      void USART_DisableClock();
      void USART_DMA_EnableClock();
      void USART_EnableInterrupts();
      void USART_DisableInterrupts();

      void USART_DMA_Init( const Chimera::Serial::SubPeripheral periph );
      void USART_DMA_DeInit( const Chimera::Serial::SubPeripheral periph );
      void USART_DMA_EnableIT( const Chimera::Serial::SubPeripheral periph );
      void USART_DMA_DisableIT( const Chimera::Serial::SubPeripheral periph );

      void USART_OverrunHandler();
    };
  }    // namespace USART
}    // namespace Thor

#endif /* !THOR_USART_HPP */
