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
#include <boost/circular_buffer_fwd.hpp>

/* Chimera Includes */
#include <Chimera/buffer.hpp>
#include <Chimera/interface/serial_intf.hpp>

/* Thor Includes */
#include <Thor/gpio.hpp>
#include <Thor/types/interrupt_types.hpp>

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

namespace Thor::USART
{
  class USARTClass;
  using USARTClass_sPtr = std::shared_ptr<USARTClass>;
  using USARTClass_uPtr = std::unique_ptr<USARTClass>;

  class USARTClass : public Chimera::Serial::Interface
  {
  public:
    USARTClass();
    ~USARTClass();

    Chimera::Status_t assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins ) final override;

    Chimera::Status_t begin( const Chimera::Serial::Modes txMode, const Chimera::Serial::Modes rxMode ) final override;

    Chimera::Status_t end() final override;

    Chimera::Status_t configure( const uint32_t baud, const Chimera::Serial::CharWid width,
                                 const Chimera::Serial::Parity parity, const Chimera::Serial::StopBits stop,
                                 const Chimera::Serial::FlowControl flow ) final override;

    Chimera::Status_t setBaud( const uint32_t baud ) final override;

    Chimera::Status_t setMode( const Chimera::Hardware::SubPeripheral periph,
                               const Chimera::Serial::Modes mode ) final override;

    Chimera::Status_t write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

    Chimera::Status_t read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS = 500 ) final override;

    Chimera::Status_t flush( const Chimera::Hardware::SubPeripheral periph ) final override;

    void postISRProcessing() final override;
    
    Chimera::Status_t readAsync( uint8_t *const buffer, const size_t len ) final override;

    Chimera::Status_t attachNotifier( const Chimera::Event::Trigger_t event,
                                           SemaphoreHandle_t *const semphr ) final override;

    Chimera::Status_t detachNotifier( const Chimera::Event::Trigger_t event,
                                           SemaphoreHandle_t *const semphr ) final override;

    Chimera::Status_t enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                       boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                       const uint32_t hwBufferSize ) final override;
    
    Chimera::Status_t disableBuffering( const Chimera::Hardware::SubPeripheral periph ) final override;
    
    bool available( size_t *const bytes = nullptr ) final override;

  private:
    /*------------------------------------------------
    Allows the C STM32 HAL and ISR functions access this class
    ------------------------------------------------*/
    friend void(::HAL_USART_TxCpltCallback )( USART_HandleTypeDef *UsartHandle );
    friend void(::HAL_USART_RxCpltCallback )( USART_HandleTypeDef *UsartHandle );
    friend void(::USART1_IRQHandler )( void );
    friend void(::USART2_IRQHandler )( void );
    friend void(::USART3_IRQHandler )( void );
    friend void(::USART6_IRQHandler )( void );

    int usart_channel;             /* Which peripheral hardware channel this class is mapped to (ie USART1, USART2, etc ...) */
    bool tx_complete       = true; /**< Indicates if a transmission has been completed */
    bool rx_complete       = true; /**< Indicates if a reception has been completed */
    bool AUTO_ASYNC_RX     = true; /**< Enables/Disables asynchronous reception of data */
    bool hardware_assigned = false;

    volatile uint32_t event_bits = 0u; /* Tracks ISR events so we can respond to them */
    
    Chimera::Serial::Modes txMode; /**< Logs which mode the TX peripheral is currently in */
    Chimera::Serial::Modes rxMode; /**< Logs which mode the RX peripheral is currently in */

    Chimera::Buffer::DoubleBuffer<USARTClass> txBuffers;
    Chimera::Buffer::DoubleBuffer<USARTClass> rxBuffers;

    uint32_t asyncRXDataSize = 0u;

    uint8_t dmaRXReqSig;
    uint8_t dmaTXReqSig;

    SemaphoreHandle_t *rxCompleteWakeup;
    SemaphoreHandle_t *txCompleteWakeup;
    
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
}    // namespace Thor::USART

#endif /* !THOR_USART_HPP */
