/********************************************************************************
 * File Name:
 *    usart.hpp
 *
 * Description:
 *    USART interface for Thor. This file supports the top level interface layer
 *    that all drivers for the underlying hardware must conform to.
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
#include <boost/circular_buffer_fwd.hpp>

/* Chimera Includes */
#include <Chimera/buffer.hpp>
#include <Chimera/interface/serial_intf.hpp>
#include <Chimera/types/event_types.hpp>

/* Thor Includes */
#include <Thor/drivers/Usart.hpp>
#include <Thor/gpio.hpp>
#include <Thor/types/interrupt_types.hpp>


namespace Thor::USART
{
#if ( THOR_CUSTOM_DRIVERS == 1 ) && ( THOR_DRIVER_USART == 1 )

  class USARTClass : public Chimera::Serial::Interface
  {
  public:
    USARTClass();
    ~USARTClass();

    Chimera::Status_t assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins ) final override;

    Chimera::Status_t begin( const Chimera::Hardware::SubPeripheralMode,
                             const Chimera::Hardware::SubPeripheralMode rxMode ) final override;

    Chimera::Status_t end() final override;

    Chimera::Status_t configure( const Chimera::Serial::COMConfig &config ) final override;

    Chimera::Status_t setBaud( const uint32_t baud ) final override;

    Chimera::Status_t setMode( const Chimera::Hardware::SubPeripheral periph,
                               const Chimera::Hardware::SubPeripheralMode mode ) final override;

    Chimera::Status_t write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

    Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

    Chimera::Status_t flush( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len ) final override;

    Chimera::Status_t enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                       boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                       const uint32_t hwBufferSize ) final override;

    Chimera::Status_t disableBuffering( const Chimera::Hardware::SubPeripheral periph ) final override;

    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID ) final override;

    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout ) final override;

    bool available( size_t *const bytes = nullptr ) final override;

    void await( const Chimera::Event::Trigger event ) final override;

    void await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier ) final override;

    void postISRProcessing() final override;
  };

  using USARTClass_sPtr = std::shared_ptr<USARTClass>;
  using USARTClass_uPtr = std::unique_ptr<USARTClass>;

#endif /* THOR_CUSTOM_DRIVERS && THOR_DRIVER_USART */


#if ( THOR_STM32HAL_DRIVERS == 1 ) && ( THOR_DRIVER_USART == 1 )

  namespace STM32HAL
  {
    /**
     *  Class that can be used to interact with a USART hardware peripheral on any
     *  supported STM32 series devices. The intent is to allow the software engineer
     *  to easily consume the USART hardware with minimal effort. By default this
     *  class is designed to be thread safe, but the actual reality of this depends
     *  upon whether or not the low level driver takes advantage of the inherited
     *  RTOS functionality.
     *
     *  @note Documenation for all public functions can be found in the inherited
     *        interface class. All low level drivers that implement this class must
     *        follow the behavior specified in the function documentation in order
     *        to not break higher level code that builds on this module.
     */
    class USARTClass : public Chimera::Serial::Interface
    {
    public:
      USARTClass();
      ~USARTClass();

      Chimera::Status_t assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins ) final override;

      Chimera::Status_t begin( const Chimera::Hardware::SubPeripheralMode,
                               const Chimera::Hardware::SubPeripheralMode rxMode ) final override;

      Chimera::Status_t end() final override;

      Chimera::Status_t configure( const Chimera::Serial::COMConfig &config ) final override;

      Chimera::Status_t setBaud( const uint32_t baud ) final override;

      Chimera::Status_t setMode( const Chimera::Hardware::SubPeripheral periph,
                                 const Chimera::Hardware::SubPeripheralMode mode ) final override;

      Chimera::Status_t write( const uint8_t *const buffer, const size_t length,
                               const uint32_t timeout_mS = 500 ) final override;

      Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

      Chimera::Status_t flush( const Chimera::Hardware::SubPeripheral periph ) final override;

      void postISRProcessing() final override;

      Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len ) final override;

#if defined( USING_FREERTOS )
      Chimera::Status_t attachNotifier( const Chimera::Event::Trigger event, SemaphoreHandle_t *const semphr ) final override;

      Chimera::Status_t detachNotifier( const Chimera::Event::Trigger event, SemaphoreHandle_t *const semphr ) final override;
#endif

      Chimera::Status_t enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                         boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                         const uint32_t hwBufferSize ) final override;

      Chimera::Status_t disableBuffering( const Chimera::Hardware::SubPeripheral periph ) final override;

      bool available( size_t *const bytes = nullptr ) final override;

      void await( const Chimera::Event::Trigger event ) final override;

      void await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier ) final override;

    private:
/*------------------------------------------------
Allows the C STM32 HAL and ISR functions access this class
------------------------------------------------*/
#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
      friend void(::HAL_USART_TxCpltCallback )( USART_HandleTypeDef *UsartHandle );
      friend void(::HAL_USART_RxCpltCallback )( USART_HandleTypeDef *UsartHandle );
#endif

      friend void(::USART1_IRQHandler )( void );
      friend void(::USART2_IRQHandler )( void );
      friend void(::USART3_IRQHandler )( void );
      friend void(::USART6_IRQHandler )( void );

      int usart_channel;         /* Which peripheral hardware channel this class is mapped to (ie USART1, USART2, etc ...) */
      bool tx_complete   = true; /**< Indicates if a transmission has been completed */
      bool rx_complete   = true; /**< Indicates if a reception has been completed */
      bool AUTO_ASYNC_RX = true; /**< Enables/Disables asynchronous reception of data */

      Chimera::Hardware::SubPeripheralMode txMode; /**< Logs which mode the TX peripheral is currently in */
      Chimera::Hardware::SubPeripheralMode rxMode; /**< Logs which mode the RX peripheral is currently in */

      Chimera::Buffer::DoubleBuffer<USARTClass> txBuffers;
      Chimera::Buffer::DoubleBuffer<USARTClass> rxBuffers;

      uint32_t asyncRXDataSize = 0u;

      uint8_t dmaRXReqSig;
      uint8_t dmaTXReqSig;

#if defined( USING_FREERTOS )
      SemaphoreHandle_t *rxCompleteWakeup;
      SemaphoreHandle_t *txCompleteWakeup;
      volatile EventBits_t eventBits;
      volatile bool isrPostProcessingComplete;

      SemaphoreHandle_t awaitEventRXComplete;
      SemaphoreHandle_t awaitEventTXComplete;
#endif

      USART_HandleTypeDef usart_handle;
      DMA_HandleTypeDef hdma_usart_tx;
      DMA_HandleTypeDef hdma_usart_rx;

      Thor::GPIO::GPIOClass_sPtr tx_pin;
      Thor::GPIO::GPIOClass_sPtr rx_pin;

      Thor::Interrupt::Initializer ITSettings_HW;
      Thor::Interrupt::Initializer ITSettings_DMA_TX;
      Thor::Interrupt::Initializer ITSettings_DMA_RX;

      struct USARTClassStatus
      {
        bool configured                = false;
        bool gpio_enabled              = false;
        bool hardware_assigned         = false;
        bool tx_buffering_enabled      = false;
        bool rx_buffering_enabled      = false;
        bool dma_enabled_tx            = false;
        bool dma_enabled_rx            = false;
        bool interrupts_enabled        = false;
        bool dma_interrupts_enabled_tx = false;
        bool dma_interrupts_enabled_rx = false;

        bool rxOverrun = false;
      } PeripheralState;


      std::array<void ( USARTClass::* )( const Chimera::Hardware::SubPeripheral ), 3> modeChangeFuncPtrs;
      void setBlockingMode( const Chimera::Hardware::SubPeripheral periph );
      void setInterruptMode( const Chimera::Hardware::SubPeripheral periph );
      void setDMAMode( const Chimera::Hardware::SubPeripheral periph );

      std::array<Chimera::Status_t ( USARTClass::* )( uint8_t *const, const size_t, const uint32_t ), 3> readFuncPtrs;
      Chimera::Status_t readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
      Chimera::Status_t readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
      Chimera::Status_t readDMA( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );

      std::array<Chimera::Status_t ( USARTClass::* )( const uint8_t *const, const size_t, const uint32_t ), 3> writeFuncPtrs;
      Chimera::Status_t writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
      Chimera::Status_t writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );
      Chimera::Status_t writeDMA( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS );

      bool setWordLength( USART_InitTypeDef &initStruct, const Chimera::Serial::CharWid width );
      bool setParity( USART_InitTypeDef &initStruct, const Chimera::Serial::Parity parity );
      bool setStopBits( USART_InitTypeDef &initStruct, const Chimera::Serial::StopBits stopBits );

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

      void USART_DMA_Init( const Chimera::Hardware::SubPeripheral periph );
      void USART_DMA_DeInit( const Chimera::Hardware::SubPeripheral periph );
      void USART_DMA_EnableIT( const Chimera::Hardware::SubPeripheral periph );
      void USART_DMA_DisableIT( const Chimera::Hardware::SubPeripheral periph );

      void USART_OverrunHandler();
    };
  }    // namespace STM32HAL

  using USARTClass_sPtr = std::shared_ptr<STM32HAL::USARTClass>;
  using USARTClass_uPtr = std::unique_ptr<STM32HAL::USARTClass>;

#endif /* THOR_STM32HAL_DRIVERS && THOR_DRIVER_USART */

}    // namespace Thor::USART
#endif /* !THOR_USART_HPP */
