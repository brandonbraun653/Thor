/********************************************************************************
 * File Name:
 *   thor_usart.cpp
 *
 * Description:
 *   Implements the USART interface for Thor
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>

/* Thor Includes */
#include <Thor/core.hpp>
#include <Thor/defaults/serial_defaults.hpp>
#include <Thor/definitions/serial_definitions.hpp>
#include <Thor/dma.hpp>
#include <Thor/headers.hpp>
#include <Thor/usart.hpp>

/* FreeRTOS Includes */
#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
}
#endif

/* Mock Includes */
#if defined( GMOCK_TEST )
#include "mock_stm32_hal_usart.hpp"
#endif

using namespace Thor;
using namespace Thor::Serial;
using namespace Thor::USART;
using namespace Thor::GPIO;
using namespace Thor::Interrupt;
using namespace Chimera::Serial;

/*------------------------------------------------
Local Constants
------------------------------------------------*/
static constexpr uint32_t EVENT_BIT_RX_COMPLETE = 1u << 0;
static constexpr uint32_t EVENT_BIT_TX_COMPLETE = 1u << 1;

/*------------------------------------------------
Local Functions Declarations
------------------------------------------------*/
static constexpr bool isChannelSupported( const uint8_t channel );
static void USART1ISRPostProcessor( void *argument );
static void USART2ISRPostProcessor( void *argument );
static void USART3ISRPostProcessor( void *argument );
static void USART6ISRPostProcessor( void *argument );

static uint32_t usartClockMask( USART_TypeDef *const instance );
static volatile uint32_t *getUsartClockReg( USART_TypeDef *const instance );
static USARTClass *const getUSARTClassRef( USART_TypeDef *const instance );

/*------------------------------------------------
Local Data
------------------------------------------------*/
static std::array<USARTClass *, MAX_SERIAL_CHANNELS + 1> usartObjects;

/* Post Processor Thread Handles */
static std::array<TaskHandle_t, MAX_SERIAL_CHANNELS + 1> isrPPHandle;

/* Post Processor Thread Wakeup Signals */
static std::array<SemaphoreHandle_t, MAX_SERIAL_CHANNELS + 1> isrPPWakeup;

/* Post Processor Thread Function Pointers */ /* clang-format off */
static std::array<Chimera::Function::void_func_void_ptr, MAX_SERIAL_CHANNELS + 1> isrPPThread = { 
  nullptr, USART1ISRPostProcessor, USART2ISRPostProcessor, USART3ISRPostProcessor,
  nullptr, nullptr, USART6ISRPostProcessor, nullptr, nullptr
}; /* clang-format on */


namespace Thor::USART
{
  inline void USART_ClearIT_IDLE( USART_HandleTypeDef *const UsartHandle )
  {
#if defined( STM32F7 )
    __HAL_USART_CLEAR_IT( UsartHandle, USART_CLEAR_IDLEF );
#endif
  }

  inline void USART_EnableIT_IDLE( USART_HandleTypeDef *const UsartHandle )
  {
    USART_ClearIT_IDLE( UsartHandle );

#if defined( SIM )
#else
    __HAL_USART_ENABLE_IT( UsartHandle, USART_IT_IDLE );
#endif /* SIM */
  }

  inline void USART_DisableIT_IDLE( USART_HandleTypeDef *const UsartHandle )
  {
    USART_ClearIT_IDLE( UsartHandle );
#if defined( SIM )
#else
    __HAL_USART_DISABLE_IT( UsartHandle, USART_IT_IDLE );
#endif /* SIM */
  }

  USARTClass::USARTClass()
  {
    using namespace Chimera::Hardware;

    /*------------------------------------------------
    Register the mode change function pointers
    ------------------------------------------------*/
    modeChangeFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::BLOCKING ) ]  = &USARTClass::setBlockingMode;
    modeChangeFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::INTERRUPT ) ] = &USARTClass::setInterruptMode;
    modeChangeFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::DMA ) ]       = &USARTClass::setDMAMode;

    /*------------------------------------------------
    Register the read function pointers
    ------------------------------------------------*/
    readFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::BLOCKING ) ]  = &USARTClass::readBlocking;
    readFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::INTERRUPT ) ] = &USARTClass::readInterrupt;
    readFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::DMA ) ]       = &USARTClass::readDMA;

    /*------------------------------------------------
    Register the write function pointers
    ------------------------------------------------*/
    writeFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::BLOCKING ) ]  = &USARTClass::writeBlocking;
    writeFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::INTERRUPT ) ] = &USARTClass::writeInterrupt;
    writeFuncPtrs[ static_cast<uint8_t>( SubPeripheralMode::DMA ) ]       = &USARTClass::writeDMA;

    AUTO_ASYNC_RX = false;

    dmaRXReqSig = Thor::DMA::Source::NONE;
    dmaTXReqSig = Thor::DMA::Source::NONE;

    rxCompleteWakeup = nullptr;
    txCompleteWakeup = nullptr;

    txMode = Chimera::Hardware::SubPeripheralMode::UNKNOWN_MODE;
    rxMode = Chimera::Hardware::SubPeripheralMode::UNKNOWN_MODE;

#if defined( GMOCK_TEST )
    if ( !STM32HAL_Mock::usartMockObj )
    {
      STM32HAL_Mock::usartMockObj = std::make_shared<STM32HAL_Mock::USARTNiceMock>();
    }
#endif /* GMOCK_TEST */
  }

  USARTClass::~USARTClass()
  {
    usartObjects[ usart_channel ] = nullptr;
  }

  Chimera::Status_t USARTClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !isChannelSupported( channel ) )
    {
      error                             = Chimera::CommonStatusCodes::NOT_SUPPORTED;
      PeripheralState.hardware_assigned = false;
    }
    else
    {
      asyncRXDataSize = 0;

      /*------------------------------------------------
      Assign the default handle settings
      ------------------------------------------------*/
      usartObjects[ channel ] = this;
      usart_channel           = channel;
      usart_handle.Init       = dflt_USART_Init;
      usart_handle.Instance   = const_cast<USART_TypeDef *>( hwConfig[ usart_channel ]->instance );

#if defined( STM32F7 )
      usart_handle.AdvancedInit = Defaults::Serial::dflt_USART_AdvInit;
#endif

      /*------------------------------------------------
      Copy over the interrupt settings information
      ------------------------------------------------*/
      ITSettings_HW     = hwConfig[ usart_channel ]->IT_HW;
      ITSettings_DMA_TX = hwConfig[ usart_channel ]->dmaIT_TX;
      ITSettings_DMA_RX = hwConfig[ usart_channel ]->dmaIT_RX;

      dmaTXReqSig = Thor::Serial::DMATXRequestSignal[ usart_channel ];
      dmaRXReqSig = Thor::Serial::DMARXRequestSignal[ usart_channel ];

      /*------------------------------------------------
      Initialize the GPIO pins
      ------------------------------------------------*/
      const auto tx_port    = Thor::GPIO::convertPort( pins.tx.port );
      const auto tx_pin_num = Thor::GPIO::convertPinNum( pins.tx.pin );

      if ( !tx_port || ( tx_pin_num == PinNum::NOT_A_PIN ) )
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
      else
      {
        tx_pin = std::make_shared<Thor::GPIO::GPIOClass>();
        tx_pin->initAdvanced( tx_port, tx_pin_num, PinSpeed::ULTRA_SPD, pins.tx.alternate );
      }

      const auto rx_port    = Thor::GPIO::convertPort( pins.rx.port );
      const auto rx_pin_num = Thor::GPIO::convertPinNum( pins.rx.pin );

      if ( !rx_port || ( rx_pin_num == PinNum::NOT_A_PIN ) )
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
      else
      {
        rx_pin = std::make_shared<Thor::GPIO::GPIOClass>();
        rx_pin->initAdvanced( rx_port, rx_pin_num, PinSpeed::ULTRA_SPD, pins.rx.alternate );
      }

      USART_GPIO_Init();
      PeripheralState.hardware_assigned = true;
    }

    return error;
  }

  Chimera::Status_t USARTClass::begin( const Chimera::Hardware::SubPeripheralMode txMode,
                                       const Chimera::Hardware::SubPeripheralMode rxMode )
  {
    using namespace Chimera::Hardware;

    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Transition to the desired user operating mode
    ------------------------------------------------*/
    error = setMode( Chimera::Hardware::SubPeripheral::TX, txMode );
    if ( error == Chimera::CommonStatusCodes::OK )
    {
      error = setMode( Chimera::Hardware::SubPeripheral::RX, rxMode );
    }

    /*------------------------------------------------
    Register the ISR post processor thread
    ------------------------------------------------*/
    if ( ( error == Chimera::CommonStatusCodes::OK ) && isrPPThread[ usart_channel ] )
    {
      isrPPWakeup[ usart_channel ] = xSemaphoreCreateBinary();
      isrPPHandle[ usart_channel ] = nullptr;

      Chimera::Threading::addThread( isrPPThread[ usart_channel ], "", 500, NULL, 5, &isrPPHandle[ usart_channel ] );
    }

    return error;
  }

  Chimera::Status_t USARTClass::end()
  {
    using namespace Chimera::Hardware;

    USART_DeInit();
    USART_GPIO_DeInit();
    USART_DisableInterrupts();
    USART_DMA_DeInit( Chimera::Hardware::SubPeripheral::TX );
    USART_DMA_DeInit( Chimera::Hardware::SubPeripheral::RX );

    txMode = SubPeripheralMode::UNKNOWN_MODE;
    rxMode = SubPeripheralMode::UNKNOWN_MODE;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t USARTClass::configure( const uint32_t baud, const Chimera::Serial::CharWid width,
                                           const Chimera::Serial::Parity parity, const Chimera::Serial::StopBits stop,
                                           const Chimera::Serial::FlowControl flow )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !PeripheralState.hardware_assigned )
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      bool result = true;

      USART_InitTypeDef init = usart_handle.Init;

      init.BaudRate = baud;
      result |= setWordLength( init, width );
      result |= setParity( init, parity );
      result |= setStopBits( init, stop );

      if ( result )
      {
        error = USART_Init();
      }
      else
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
    }

    return error;
  }

  Chimera::Status_t USARTClass::setBaud( const uint32_t baud )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !PeripheralState.hardware_assigned )
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      /*------------------------------------------------
      Copy, modify, write
      ------------------------------------------------*/
      USART_InitTypeDef init = usart_handle.Init;
      init.BaudRate          = baud;
      usart_handle.Init      = init;

      /*------------------------------------------------
      Clear the hardware config and re-initialize with new settings
      ------------------------------------------------*/
      USART_DeInit();
      USART_Init();
    }

    return error;
  }

  Chimera::Status_t USARTClass::setMode( const Chimera::Hardware::SubPeripheral periph,
                                         const Chimera::Hardware::SubPeripheralMode mode )
  {
    using namespace Chimera::Hardware;

    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;
    auto iter               = static_cast<uint8_t>( mode );

    /*------------------------------------------------
    Only bother changing modes if we currently aren't there
    ------------------------------------------------*/
    if ( !( ( ( periph == SubPeripheral::TX ) && ( txMode == mode ) ) ||
            ( ( periph == SubPeripheral::RX ) && ( rxMode == mode ) ) ) )
    {
      /*------------------------------------------------
      No matter what, the hardware must be assigned, configured, and transitioning to a valid mode.
      ------------------------------------------------*/
      if ( iter >= modeChangeFuncPtrs.size() )
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
      else if ( !PeripheralState.hardware_assigned || !PeripheralState.configured )
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }

      /*------------------------------------------------
      Validate the buffering initialization if transitioning to asynchronous mode
      ------------------------------------------------*/
      if ( ( periph == SubPeripheral::TX ) &&
           ( ( mode == SubPeripheralMode::INTERRUPT ) || ( mode == SubPeripheralMode::DMA ) ) &&
           !PeripheralState.tx_buffering_enabled )
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }
      else if ( ( periph == SubPeripheral::RX ) &&
                ( ( mode == SubPeripheralMode::INTERRUPT ) || ( mode == SubPeripheralMode::DMA ) ) &&
                !PeripheralState.rx_buffering_enabled )
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }

      /*------------------------------------------------
      Assuming all our checks "check out", we can switch modes
      ------------------------------------------------*/
      if ( error == Chimera::CommonStatusCodes::OK )
      {
        ( this->*( modeChangeFuncPtrs[ iter ] ) )( periph );
      }
    }

    return error;
  }

  Chimera::Status_t USARTClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;
    auto iter               = static_cast<uint8_t>( txMode );

    if ( !PeripheralState.gpio_enabled || !PeripheralState.configured )
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else if ( iter >= writeFuncPtrs.size() )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      error = ( this->*( writeFuncPtrs[ iter ] ) )( buffer, length, timeout_mS );
    }

    return error;
  }

  Chimera::Status_t USARTClass::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;
    auto iter               = static_cast<uint8_t>( rxMode );

    if ( !PeripheralState.gpio_enabled || !PeripheralState.configured )
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else if ( iter >= readFuncPtrs.size() )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      error = ( this->*( readFuncPtrs[ iter ] ) )( buffer, length, timeout_mS );
    }

    return error;
  }

  Chimera::Status_t USARTClass::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      error = txBuffers.flush();
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      error = rxBuffers.flush();
    }
    else
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    return error;
  }

  void USARTClass::postISRProcessing()
  {
    using namespace Chimera::Hardware;

    const uint32_t cached_event = event_bits;

    if ( cached_event & EVENT_BIT_TX_COMPLETE )
    {
      event_bits &= ~EVENT_BIT_TX_COMPLETE;

      tx_complete = true;

      /*------------------------------------------------
      Transmit more data if we have any
      ------------------------------------------------*/
      if ( !txBuffers.external->empty() )
      {
        size_t bytesToWrite = std::min( txBuffers.external->size(), static_cast<size_t>( txBuffers.internalSize ) );
        memset( txBuffers.internal, 0, txBuffers.internalSize );

        for ( uint32_t x = 0; x < bytesToWrite; x++ )
        {
          txBuffers.internal[ x ] = txBuffers.external->front();
          txBuffers.external->pop_front();
        }

        write( txBuffers.internal, bytesToWrite );
      }
      else
      {
        /*------------------------------------------------
        Let user threads know the transfer completed
        ------------------------------------------------*/
        if ( txCompleteWakeup )
        {
          BaseType_t xHigherPriorityTaskWoken = pdFALSE;
          xSemaphoreGiveFromISR( *txCompleteWakeup, &xHigherPriorityTaskWoken );
          portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }
      }
    }
    else if ( cached_event & EVENT_BIT_RX_COMPLETE )
    {
      event_bits &= ~EVENT_BIT_RX_COMPLETE;

      if ( rxMode == SubPeripheralMode::INTERRUPT )
      {
        if ( AUTO_ASYNC_RX )
        {
          /*------------------------------------------------
          We unexpectedly got some data, so dump it into the user's buffer for them
          to read later. Don't need to check if the buffer is valid as:

          1. ISR routine. We want AFAP.
          2. The buffer's existence is a requirement for this ISR to even execute.
          ------------------------------------------------*/
          for ( size_t x = 0; x < asyncRXDataSize; x++ )
          {
            rxBuffers.external->push_back( rxBuffers.internal[ x ] );
          }
          asyncRXDataSize = 0;
        }
        else
        {
          /*------------------------------------------------
          The user explicitly requested for data to be read
          ------------------------------------------------*/
          uint16_t actuallyRead = usart_handle.RxXferSize - usart_handle.RxXferCount;
          for ( uint16_t x = 0; x < actuallyRead; x++ )
          {
            rxBuffers.external->push_back( rxBuffers.internal[ x ] );
          }
        }

        /*------------------------------------------------
        Go back to async RX listening
        ------------------------------------------------*/
        __HAL_USART_ENABLE_IT( &usart_handle, USART_IT_RXNE );
      }
      else if ( rxMode == SubPeripheralMode::DMA )
      {
        /*------------------------------------------------
        Calculate how many bytes were received by looking at remaining RX buffer space.
                    num_received = bufferMaxSize - bufferSizeRemaining
        ------------------------------------------------*/
        auto num_received = static_cast<uint32_t>( usart_handle.RxXferSize - usart_handle.hdmarx->Instance->NDTR );

        /*------------------------------------------------
        Copy out the data
        ------------------------------------------------*/
        for ( size_t x = 0; x < num_received; x++ )
        {
          rxBuffers.external->push_back( rxBuffers.internal[ x ] );
        }

        /*------------------------------------------------
        Go back to async RX listening
        ------------------------------------------------*/
        memset( rxBuffers.internal, 0, rxBuffers.internalSize );
        Thor::USART::USART_EnableIT_IDLE( &usart_handle );
        HAL_USART_Receive_DMA( &usart_handle, rxBuffers.internal, rxBuffers.internalSize );
      }

      /*------------------------------------------------
      Exit the ISR ensuring that we can still receive asynchronous data
      ------------------------------------------------*/
      rx_complete   = true;
      AUTO_ASYNC_RX = true;

      /*------------------------------------------------
      Let user threads know the transfer completed
      ------------------------------------------------*/
      if ( rxCompleteWakeup )
      {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR( *rxCompleteWakeup, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
      }
    }
  }

  Chimera::Status_t USARTClass::readAsync( uint8_t *const buffer, const size_t len )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !buffer )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      size_t bytesRead = 0;

      if ( auto tmp = rxBuffers.get(); tmp )
      {
        while ( !tmp->empty() && ( bytesRead < len ) )
        {
          buffer[ bytesRead ] = tmp->front();
          tmp->pop_front();
          bytesRead++;
        }
      }


      if ( bytesRead != len )
      {
        error = Chimera::CommonStatusCodes::EMPTY;
      }
    }

    return error;
  }

  Chimera::Status_t USARTClass::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                 boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                                 const uint32_t hwBufferSize )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      txBuffers.assign( userBuffer, hwBuffer, hwBufferSize );
      PeripheralState.tx_buffering_enabled = true;
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      rxBuffers.assign( userBuffer, hwBuffer, hwBufferSize );
      PeripheralState.rx_buffering_enabled = true;
    }
    else
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    return error;
  }

  Chimera::Status_t USARTClass::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      PeripheralState.tx_buffering_enabled = false;
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      PeripheralState.tx_buffering_enabled = false;
    }
    else
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    return error;
  }

  Chimera::Status_t USARTClass::attachNotifier( const Chimera::Event::Trigger_t event, SemaphoreHandle_t *const semphr )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !semphr )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else if ( event == Chimera::Event::Trigger_t::READ_COMPLETE )
    {
      rxCompleteWakeup = semphr;
    }
    else if ( event == Chimera::Event::Trigger_t::WRITE_COMPLETE )
    {
      txCompleteWakeup = semphr;
    }
    else
    {
      error = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    return error;
  }

  Chimera::Status_t USARTClass::detachNotifier( const Chimera::Event::Trigger_t event, SemaphoreHandle_t *const semphr )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( event == Chimera::Event::Trigger_t::READ_COMPLETE )
    {
      rxCompleteWakeup = nullptr;
    }
    else if ( event == Chimera::Event::Trigger_t::WRITE_COMPLETE )
    {
      txCompleteWakeup = nullptr;
    }
    else
    {
      error = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    return error;
  }

  bool USARTClass::available( size_t *const bytes )
  {
    bool retval = false;

    if ( auto tmp = rxBuffers.get(); tmp && !tmp->empty() )
    {
      retval = true;

      if ( bytes )
      {
        *bytes = tmp->size();
      }
    }

    return retval;
  }

  bool USARTClass::setWordLength( USART_InitTypeDef &initStruct, const Chimera::Serial::CharWid width )
  {
    switch ( static_cast<uint8_t>( width ) )
    {
      case static_cast<uint8_t>( CharWid::CW_8BIT ):
      default:
        initStruct.WordLength = USART_WORDLENGTH_8B;
        break;
    }

    /* All cases are handled, so it will always return true */
    return true;
  }

  bool USARTClass::setParity( USART_InitTypeDef &initStruct, const Chimera::Serial::Parity parity )
  {
    switch ( static_cast<uint8_t>( parity ) )
    {
      case static_cast<uint8_t>( Parity::PAR_EVEN ):
        initStruct.Parity = USART_PARITY_EVEN;
        break;

      case static_cast<uint8_t>( Parity::PAR_ODD ):
        initStruct.Parity = USART_PARITY_ODD;
        break;

      case static_cast<uint8_t>( Parity::PAR_NONE ):
      default:
        initStruct.Parity = USART_PARITY_NONE;
        break;
    }

    /* All cases are handled, so it will always return true */
    return true;
  }

  bool USARTClass::setStopBits( USART_InitTypeDef &initStruct, const Chimera::Serial::StopBits stopBits )
  {
    bool result = true;

    switch ( static_cast<uint8_t>( stopBits ) )
    {
      case static_cast<uint8_t>( StopBits::SBITS_ONE ):
      default:
        initStruct.StopBits = USART_STOPBITS_1;
        break;

      case static_cast<uint8_t>( StopBits::SBITS_ONE_POINT_FIVE ):
        result = false;
        break;

      case static_cast<uint8_t>( StopBits::SBITS_TWO ):
        initStruct.StopBits = USART_STOPBITS_2;
        break;
    }

    return result;
  }

  void USARTClass::setBlockingMode( const Chimera::Hardware::SubPeripheral periph )
  {
    using namespace Chimera::Hardware;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      txMode = SubPeripheralMode::BLOCKING;    // Must be set before the other functions

      /* Make sure RX side isn't using interrupts before disabling */
      if ( rxMode == SubPeripheralMode::BLOCKING )
      {
        USART_DisableInterrupts();
      }

      USART_DMA_DeInit( periph );
    }
    else
    {
      rxMode = SubPeripheralMode::BLOCKING;    // Must be set before the other functions

      /* Make sure TX side isn't using interrupts before disabling */
      if ( txMode == SubPeripheralMode::BLOCKING )
      {
        USART_DisableInterrupts();
      }

      USART_DMA_DeInit( periph );
    }
  }

  void USARTClass::setInterruptMode( const Chimera::Hardware::SubPeripheral periph )
  {
    using namespace Chimera::Hardware;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      txMode = SubPeripheralMode::INTERRUPT;

      USART_EnableInterrupts();
      USART_DMA_DeInit( periph );
    }
    else
    {
      rxMode = SubPeripheralMode::INTERRUPT;

      USART_EnableInterrupts();
      USART_DMA_DeInit( periph );
    }
  }

  void USARTClass::setDMAMode( const Chimera::Hardware::SubPeripheral periph )
  {
    using namespace Chimera::Hardware;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      txMode = SubPeripheralMode::DMA;

      USART_EnableInterrupts();
      USART_DMA_Init( periph );
    }
    else
    {
      rxMode        = SubPeripheralMode::DMA;
      AUTO_ASYNC_RX = true;

      USART_EnableInterrupts();
      USART_DMA_Init( periph );

      /* Set the idle line interrupt for asynchronously getting the end of transmission */
      USART_EnableIT_IDLE( &usart_handle );

      /* Instruct the DMA hardware to start listening for transmissions */
      memset( rxBuffers.internal, 0, rxBuffers.internalSize );
      HAL_USART_Receive_DMA( &usart_handle, rxBuffers.internal, rxBuffers.internalSize );
    }
  }

  Chimera::Status_t USARTClass::readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_BUSY;

    if ( reserve( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    {
      /*------------------------------------------------
      It's possible to get into the condition where ORE is set before trying to receive some
      new data. In the current STM HAL library, all error interrupts for the blocking mode are
      disabled by default so the overrun has to be handled manually. This restores normal
      operation. A nearly exact condition of this bug is encountered here: https://goo.gl/bKi8Ps
      ------------------------------------------------*/
      USART_OverrunHandler();

#if defined( USING_FREERTOS )
      stm32Error = HAL_USART_Receive( &usart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ),
                                      pdMS_TO_TICKS( BLOCKING_TIMEOUT_MS ) );
#else
      stm32Error = HAL_USART_Receive( &usart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ),
                                      BLOCKING_TIMEOUT_MS );
#endif
      release();
    }

    return convertHALStatus( stm32Error );
  }

  Chimera::Status_t USARTClass::readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    /* clang-format off */
    if ( ( length <= rxBuffers.internalSize )
      && reserve( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    { /* clang-format on */
      /*------------------------------------------------
      Let the ISR handler know that we explicitely asked to receive some data.
      This will cause a redirect into the correct ISR handling control flow.
      ------------------------------------------------*/
      AUTO_ASYNC_RX = false;
      memset( rxBuffers.internal, 0, rxBuffers.internalSize );
      stm32Error = HAL_USART_Receive_IT( &usart_handle, rxBuffers.internal, static_cast<uint16_t>( length ) );

      if ( stm32Error == HAL_OK )
      {
        error = Chimera::Serial::Status::RX_IN_PROGRESS;
      }
      else
      {
        error = convertHALStatus( stm32Error );
      }

      release();
    }
    else
    {
      error = Chimera::CommonStatusCodes::MEMORY;
    }

    return error;
  }

  Chimera::Status_t USARTClass::readDMA( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    /* clang-format off */
    if ( ( length <= rxBuffers.internalSize )
      && reserve( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    { /* clang-format on */
      /*------------------------------------------------
      Let the ISR handler know that we explicitely asked to receive some data.
      This will cause a redirect into the correct handling channels.
      ------------------------------------------------*/
      AUTO_ASYNC_RX = false;

      /*------------------------------------------------
      Stop any background listening for data and restart with the new transfer
      ------------------------------------------------*/
      HAL_USART_DMAStop( &usart_handle );
      memset( rxBuffers.internal, 0, rxBuffers.internalSize );
      stm32Error = HAL_USART_Receive_DMA( &usart_handle, rxBuffers.internal, static_cast<uint16_t>( length ) );

      if ( stm32Error == HAL_OK )
      {
        error = Chimera::Serial::Status::RX_IN_PROGRESS;
      }
      else
      {
        error = convertHALStatus( stm32Error );
      }

      release();
    }
    else
    {
      error = Chimera::CommonStatusCodes::MEMORY;
    }

    return error;
  }

  Chimera::Status_t USARTClass::writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_BUSY;

    if ( reserve( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    {
#if defined( USING_FREERTOS )
      stm32Error = HAL_USART_Transmit( &usart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ),
                                       pdMS_TO_TICKS( BLOCKING_TIMEOUT_MS ) );
#else
      stm32Error = HAL_USART_Transmit( &usart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ),
                                       BLOCKING_TIMEOUT_MS );
#endif
      release();
    }
    return convertHALStatus( stm32Error );
  }

  Chimera::Status_t USARTClass::writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    /* clang-format off */
    if ( txBuffers.initialized()
      && PeripheralState.tx_buffering_enabled 
      && reserve( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    { /* clang-format on */
      if ( tx_complete )
      {
        /*------------------------------------------------
        Hardware is free. Send the data directly.
        ------------------------------------------------*/
        tx_complete = false;
        stm32Error  = HAL_USART_Transmit_IT( &usart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ) );
        error       = convertHALStatus( stm32Error );
      }
      else
      {
        /*------------------------------------------------
        Queue up everything to send later
        ------------------------------------------------*/
        error = Chimera::CommonStatusCodes::BUSY;
        txBuffers.push( buffer, length );
      }

      release();
    }
    else
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    return error;
  }

  Chimera::Status_t USARTClass::writeDMA( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    /* clang-format off */
      if ( txBuffers.initialized() 
        && PeripheralState.tx_buffering_enabled 
        && reserve( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
      { /* clang-format on */
      if ( tx_complete )
      {
        /*------------------------------------------------
        Hardware is free. Send the data directly.
        ------------------------------------------------*/
        tx_complete = false;
        stm32Error  = HAL_USART_Transmit_DMA( &usart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ) );
        error       = convertHALStatus( stm32Error );
      }
      else
      {
        /*------------------------------------------------
        Queue up everything to send later
        ------------------------------------------------*/
        error = Chimera::CommonStatusCodes::BUSY;
        txBuffers.push( buffer, length );
      }

      release();
    }
    else
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    return error;
  }

  void USARTClass::IRQHandler_TXDMA()
  {
    HAL_DMA_IRQHandler( usart_handle.hdmatx );
  }

  void USARTClass::IRQHandler_RXDMA()
  {
    HAL_DMA_IRQHandler( usart_handle.hdmarx );
  }

  Chimera::Status_t USARTClass::USART_Init()
  {
    using namespace Chimera::Hardware;

    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    USART_EnableClock();

    if ( HAL_USART_Init( &usart_handle ) != HAL_OK )
    {
      error                      = Chimera::CommonStatusCodes::FAIL;
      PeripheralState.configured = false;
    }

    if ( error == Chimera::CommonStatusCodes::OK )
    {
      setMode( Chimera::Hardware::SubPeripheral::TX, SubPeripheralMode::BLOCKING );
      setMode( Chimera::Hardware::SubPeripheral::RX, SubPeripheralMode::BLOCKING );

      PeripheralState.configured = true;
    }
    return error;
  }

  void USARTClass::USART_DeInit()
  {
    HAL_USART_DeInit( &usart_handle );
    PeripheralState.configured = false;
  }

  void USARTClass::USART_EnableClock()
  {
#if defined( SIM )
#else
    *getUsartClockReg( usart_handle.Instance ) |= ( usartClockMask( usart_handle.Instance ) );
#endif /* SIM */
  }

  void USARTClass::USART_DisableClock()
  {
#if defined( SIM )
#else
    *getUsartClockReg( usart_handle.Instance ) &= ~( usartClockMask( usart_handle.Instance ) );
#endif /* SIM */
  }

  void USARTClass::USART_DMA_EnableClock()
  {
    /* Global DMA Clock options. Only turn on capability is
    provided due to other peripherals possibly using DMA. */

#if defined( SIM )
#else
    if ( __DMA1_IS_CLK_DISABLED() )
    {
      __DMA1_CLK_ENABLE();
    }

    if ( __DMA2_IS_CLK_DISABLED() )
    {
      __DMA2_CLK_ENABLE();
    }
#endif /* SIM */
  }

  void USARTClass::USART_EnableInterrupts()
  {
    using namespace Chimera::Hardware;

    HAL_NVIC_DisableIRQ( ITSettings_HW.IRQn );
    HAL_NVIC_ClearPendingIRQ( ITSettings_HW.IRQn );
    HAL_NVIC_SetPriority( ITSettings_HW.IRQn, ITSettings_HW.preemptPriority, ITSettings_HW.subPriority );
    HAL_NVIC_EnableIRQ( ITSettings_HW.IRQn );

    /*------------------------------------------------
    Make sure we are able to asynchronously receive some data if it gets sent
    ------------------------------------------------*/
    AUTO_ASYNC_RX = true;
    if ( rxMode == SubPeripheralMode::INTERRUPT )
    {
      /*------------------------------------------------
      In interrupt mode, we have to handle the RX FIFO on a byte by byte
      basis otherwise an overrun error is generated.
      ------------------------------------------------*/
#if defined( SIM )
#else
      __HAL_USART_ENABLE_IT( &usart_handle, USART_IT_RXNE );
#endif /* SIM */
    }

    PeripheralState.interrupts_enabled = true;
  }

  void USARTClass::USART_DisableInterrupts()
  {
#if defined( SIM )
#else
    __HAL_USART_DISABLE_IT( &usart_handle, USART_IT_IDLE );
    __HAL_USART_DISABLE_IT( &usart_handle, USART_IT_RXNE );
#endif /* SIM */

    HAL_NVIC_DisableIRQ( ITSettings_HW.IRQn );
    HAL_NVIC_ClearPendingIRQ( ITSettings_HW.IRQn );

    PeripheralState.interrupts_enabled = false;
  }

  void USARTClass::USART_GPIO_Init()
  {
    PeripheralState.gpio_enabled = true;

    if ( !tx_pin || ( tx_pin->setMode( Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL, true ) != Chimera::CommonStatusCodes::OK ) )
    {
      PeripheralState.gpio_enabled = false;
    }

    if ( !rx_pin || ( rx_pin->setMode( Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL, true ) != Chimera::CommonStatusCodes::OK ) )
    {
      PeripheralState.gpio_enabled = false;
    }
  }

  void USARTClass::USART_GPIO_DeInit()
  {
    // TODO: Implement GPIO DeInit
  }

  void USARTClass::USART_DMA_Init( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      USART_DMA_EnableClock();
      hdma_usart_tx.Instance = const_cast<DMA_Stream_TypeDef *>( hwConfig[ usart_channel ]->dmaTX.Instance );

      /* Grab the default init settings and modify for the specific hardware */
      hdma_usart_tx.Init           = dflt_DMA_Init_TX;
      hdma_usart_tx.Init.Channel   = hwConfig[ usart_channel ]->dmaTX.channel;
      hdma_usart_tx.Init.Direction = hwConfig[ usart_channel ]->dmaTX.direction;

      /* Hard error if initialization fails. */
      assert( HAL_DMA_Init( &hdma_usart_tx ) == HAL_OK );
      __HAL_LINKDMA( &usart_handle, hdmatx, hdma_usart_tx );

      /*------------------------------------------------
      Attach the class specific implementation TX DMA handler
      ------------------------------------------------*/
      Thor::DMA::requestHandlers[ dmaTXReqSig ] = boost::bind( &USARTClass::IRQHandler_TXDMA, this );

      USART_DMA_EnableIT( periph );

      PeripheralState.dma_enabled_tx = true;
    }
    else
    {
      USART_DMA_EnableClock();
      hdma_usart_rx.Instance = const_cast<DMA_Stream_TypeDef *>( hwConfig[ usart_channel ]->dmaRX.Instance );

      /* Grab the default init settings and modify for the specific hardware */
      hdma_usart_rx.Init           = dflt_DMA_Init_RX;
      hdma_usart_rx.Init.Channel   = hwConfig[ usart_channel ]->dmaRX.channel;
      hdma_usart_rx.Init.Direction = hwConfig[ usart_channel ]->dmaRX.direction;

      /* Hard error if initialization fails. */
      assert( HAL_DMA_Init( &hdma_usart_rx ) == HAL_OK );
      __HAL_LINKDMA( &usart_handle, hdmarx, hdma_usart_rx );

      /*------------------------------------------------
      Attach the class specific implementation TX DMA handler
      ------------------------------------------------*/
      Thor::DMA::requestHandlers[ dmaRXReqSig ] = boost::bind( &USARTClass::IRQHandler_RXDMA, this );

      USART_DMA_EnableIT( periph );

      PeripheralState.dma_enabled_rx = true;
    }
  }

  void USARTClass::USART_DMA_DeInit( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      if ( !PeripheralState.dma_enabled_tx )
        return;

      HAL_DMA_Abort( usart_handle.hdmatx );
      HAL_DMA_DeInit( usart_handle.hdmatx );
      USART_DMA_DisableIT( periph );
      Thor::DMA::requestHandlers[ dmaTXReqSig ].clear();

      PeripheralState.dma_enabled_tx = false;
    }
    else
    {
      if ( !PeripheralState.dma_enabled_rx )
        return;

      HAL_DMA_Abort( usart_handle.hdmarx );
      HAL_DMA_DeInit( usart_handle.hdmarx );
      USART_DMA_DisableIT( periph );
      Thor::DMA::requestHandlers[ dmaRXReqSig ].clear();

      PeripheralState.dma_enabled_rx = false;
    }
  }

  void USARTClass::USART_DMA_EnableIT( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      HAL_NVIC_DisableIRQ( ITSettings_DMA_TX.IRQn );
      HAL_NVIC_ClearPendingIRQ( ITSettings_DMA_TX.IRQn );
      HAL_NVIC_SetPriority( ITSettings_DMA_TX.IRQn, ITSettings_DMA_TX.preemptPriority, ITSettings_DMA_TX.subPriority );
      HAL_NVIC_EnableIRQ( ITSettings_DMA_TX.IRQn );

      PeripheralState.dma_interrupts_enabled_tx = true;
    }
    else
    {
      HAL_NVIC_DisableIRQ( ITSettings_DMA_RX.IRQn );
      HAL_NVIC_ClearPendingIRQ( ITSettings_DMA_RX.IRQn );
      HAL_NVIC_SetPriority( ITSettings_DMA_RX.IRQn, ITSettings_DMA_RX.preemptPriority, ITSettings_DMA_RX.subPriority );
      HAL_NVIC_EnableIRQ( ITSettings_DMA_RX.IRQn );

      PeripheralState.dma_interrupts_enabled_rx = true;
    }
  }

  void USARTClass::USART_DMA_DisableIT( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      HAL_NVIC_ClearPendingIRQ( ITSettings_DMA_TX.IRQn );
      HAL_NVIC_DisableIRQ( ITSettings_DMA_TX.IRQn );

      PeripheralState.dma_interrupts_enabled_tx = false;
    }
    else
    {
      HAL_NVIC_ClearPendingIRQ( ITSettings_DMA_RX.IRQn );
      HAL_NVIC_DisableIRQ( ITSettings_DMA_RX.IRQn );

      PeripheralState.dma_interrupts_enabled_rx = false;
    }
  }

  void USARTClass::USART_OverrunHandler()
  {
#if defined( STM32F7 )
    __HAL_USART_CLEAR_IT( &usart_handle, USART_CLEAR_OREF );
    usart_handle.Instance->RDR;
#endif
  }

#if defined( STM32F7 )
  void USARTClass::IRQHandler()
  {
    using namespace Chimera::Hardware;
  }
#endif

#if defined( STM32F4 )
  void USARTClass::IRQHandler()
  {
    using namespace Chimera::Hardware;

    /*------------------------------------------------
    Handle Asynchronous RX in Interrupt and DMA Mode:
    The user did not ask for data, but we got some anyways (cause we were listening)
    ------------------------------------------------*/
    if ( AUTO_ASYNC_RX )
    {
      /*------------------------------------------------
      Reading these registers in the order of SR then DR ends up clearing all
      flags, so it's best to store the returned contents for further processing.
      ------------------------------------------------*/
      volatile uint32_t isrflags = READ_REG( usart_handle.Instance->SR );
      volatile uint32_t data_reg = READ_REG( usart_handle.Instance->DR );
      volatile uint32_t cr1      = READ_REG( usart_handle.Instance->CR1 );

      /*------------------------------------------------
      Prepare necessary flags for ISR flow control
      ------------------------------------------------*/
      bool RX_DATA_READY    = ( ( isrflags & USART_FLAG_RXNE ) == USART_FLAG_RXNE );
      bool RX_LINE_IDLE     = ( ( isrflags & USART_FLAG_IDLE ) == USART_FLAG_IDLE );
      bool RX_DATA_READY_IE = ( ( cr1 & USART_CR1_RXNEIE ) == USART_CR1_RXNEIE );
      bool RX_LINE_IDLE_IE  = ( ( cr1 & USART_CR1_IDLEIE ) == USART_CR1_IDLEIE );

      /*------------------------------------------------
      RX In Progress
      ------------------------------------------------*/
      if ( RX_DATA_READY && RX_DATA_READY_IE && usart_handle.State != HAL_USART_STATE_BUSY_TX )
      {
        uint32_t error = ( isrflags & ( uint32_t )( USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE ) );

        if ( !error )
        {
          /*------------------------------------------------
          Detected start of a new frame of unknown size. Enable
          the IDLE interrupt bit to detect the end of frame.
          ------------------------------------------------*/
          if ( asyncRXDataSize == 0 )
          {
            USART_EnableIT_IDLE( &usart_handle );
            memset( rxBuffers.internal, 0, rxBuffers.internalSize );
          }

          /*------------------------------------------------
          Buffer the received byte to our internal buffer
          ------------------------------------------------*/
          if ( rxMode == SubPeripheralMode::INTERRUPT && ( asyncRXDataSize < rxBuffers.internalSize ) )
          {
            rxBuffers.internal[ asyncRXDataSize ] = static_cast<uint8_t>( data_reg );
            asyncRXDataSize += 1u;
          }
          else if ( rxMode != SubPeripheralMode::DMA )
          {
            // The receive buffer was full!!!! We lost data! Somehow set an error.
            // Probably should set this as Chimera thing.
          }
        }
      }

      /*------------------------------------------------
      RX Complete
      ------------------------------------------------*/
      if ( RX_LINE_IDLE && RX_LINE_IDLE_IE )
      {
        if ( rxMode == SubPeripheralMode::INTERRUPT )
        {
          /*------------------------------------------------
          Turn off the idle line interrupt so we aren't interrupted again
          while trying to process all the information just received.
          ------------------------------------------------*/
          USART_DisableIT_IDLE( &usart_handle );

          /*------------------------------------------------
          Manually call this function because HAL_USART_IRQHandler won't get called
          and it normally handles this operation.
          ------------------------------------------------*/
          HAL_USART_RxCpltCallback( &usart_handle );
        }
        else /* DMA */
        {
          /*------------------------------------------------
          Turn off the idle line interrupt so we aren't interrupted again
          while trying to process all the information just received.
          ------------------------------------------------*/
          USART_DisableIT_IDLE( &usart_handle );

          /*------------------------------------------------
          Force DMA hard reset to trigger the DMA RX Complete handler
          ------------------------------------------------*/
          __HAL_DMA_DISABLE( usart_handle.hdmarx );
        }
      }
    }

    /*------------------------------------------------
    Handle when the standard IRQHandler is called:

    1. We are TX-ing in IT or DMA mode (getting to this func means we are in those modes)
    2. We are RX-ing an expected asynchronous transfer that the user started
    ------------------------------------------------*/
    if ( !AUTO_ASYNC_RX || usart_handle.State == HAL_USART_STATE_BUSY_TX )
    {
      HAL_USART_IRQHandler( &usart_handle );
    }
#endif /* STM32F4 */
  }    // namespace Thor::USART

}    // namespace Thor::USART

#if !defined( GMOCK_TEST )
void HAL_USART_TxCpltCallback( USART_HandleTypeDef *UsartHandle )
{
  const auto usart = getUSARTClassRef( UsartHandle->Instance );

  if ( usart )
  {
    usart->event_bits |= EVENT_BIT_TX_COMPLETE;

    /*------------------------------------------------
    Wake up and immediately switch to the user-land ISR
    handler thread. Should be a very high priority thread.
    ------------------------------------------------*/
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( isrPPWakeup[ usart->usart_channel ], &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
}

void HAL_USART_RxCpltCallback( USART_HandleTypeDef *UsartHandle )
{
  const auto usart = getUSARTClassRef( UsartHandle->Instance );

  if ( usart )
  {
    usart->event_bits |= EVENT_BIT_TX_COMPLETE;

    /*------------------------------------------------
    Wake up and immediately switch to the user-land ISR
    handler thread. Should be a very high priority thread.
    ------------------------------------------------*/
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( isrPPWakeup[ usart->usart_channel ], &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
}

void HAL_USART_TxHalfCpltCallback( USART_HandleTypeDef *UsartHandle )
{
}

void HAL_USART_RxHalfCpltCallback( USART_HandleTypeDef *UsartHandle )
{
}

void HAL_USART_ErrorCallback( USART_HandleTypeDef *UsartHandle )
{
}
#endif /* !GMOCK_TEST */

void USART1_IRQHandler( void )
{
  if ( usartObjects[ 1 ] )
  {
    usartObjects[ 1 ]->IRQHandler();
  }
}

void USART2_IRQHandler( void )
{
  if ( usartObjects[ 2 ] )
  {
    usartObjects[ 2 ]->IRQHandler();
  }
}

void USART3_IRQHandler( void )
{
  if ( usartObjects[ 3 ] )
  {
    usartObjects[ 3 ]->IRQHandler();
  }
}

void USART6_IRQHandler( void )
{
  if ( usartObjects[ 6 ] )
  {
    usartObjects[ 6 ]->IRQHandler();
  }
}

static USARTClass *const getUSARTClassRef( USART_TypeDef *const instance )
{
  /*------------------------------------------------
  Simply converts the pointer into the raw numerical address value, which be compared against
  the peripheral base address. USARTx is simply (USART_TypeDef*)USARTx_Base.
  ------------------------------------------------*/
  auto i = reinterpret_cast<std::uintptr_t>( instance );
  switch ( i )
  {
#if defined( USART1 )
    case USART1_BASE:
      return usartObjects[ 1 ];
      break;
#endif
#if defined( USART2 )
    case USART2_BASE:
      return usartObjects[ 2 ];
      break;
#endif
#if defined( USART3 )
    case USART3_BASE:
      return usartObjects[ 3 ];
      break;
#endif
#if defined( USART4 )
    case USART4_BASE:
      return usartObjects[ 4 ];
      break;
#endif
#if defined( USART5 )
    case USART5_BASE:
      return usartObjects[ 5 ];
      break;
#endif
#if defined( USART6 )
    case USART6_BASE:
      return usartObjects[ 6 ];
      break;
#endif
#if defined( USART7 )
    case USART7_BASE:
      return usartObjects[ 7 ];
      break;
#endif
#if defined( USART8 )
    case USART8_BASE:
      return usartObjects[ 8 ];
      break;
#endif

    default:
      return usartObjects[ 0 ];
      break;
  };
};

static volatile uint32_t *getUsartClockReg( USART_TypeDef *const instance )
{
  auto i = reinterpret_cast<std::uintptr_t>( instance );
  switch ( i )
  {
#if defined( STM32F446xx ) || defined( STM32F767xx )
#if defined( USART1 )
    case USART1_BASE:
      return &( RCC->APB2ENR );
      break;
#endif
#if defined( USART2 )
    case USART2_BASE:
      return &( RCC->APB1ENR );
      break;
#endif
#if defined( USART3 )
    case USART3_BASE:
      return &( RCC->APB1ENR );
      break;
#endif
#if defined( USART6 )
    case USART6_BASE:
      return &( RCC->APB2ENR );
      break;
#endif
#endif /* !STM32F446xx  !STM32F767xx */

    default:
      return nullptr;
      break;
  };
}

static uint32_t usartClockMask( USART_TypeDef *const instance )
{
  auto i = reinterpret_cast<std::uintptr_t>( instance );
  switch ( i )
  {
#if defined( STM32F446xx ) || defined( STM32F767xx )
#if defined( USART1 )
    case USART1_BASE:
      return RCC_APB2ENR_USART1EN;
      break;
#endif
#if defined( USART2 )
    case USART2_BASE:
      return RCC_APB1ENR_USART2EN;
      break;
#endif
#if defined( USART3 )
    case USART3_BASE:
      return RCC_APB1ENR_USART3EN;
      break;
#endif
#if defined( USART6 )
    case USART6_BASE:
      return RCC_APB2ENR_USART6EN;
      break;
#endif
#endif /* !STM32F446xx  !STM32F767xx */

    default:
      return 0u;
      break;
  };
};

static constexpr bool isChannelSupported( const uint8_t channel )
{
  return ( ( channel == 1 ) || ( channel == 2 ) || ( channel == 3 ) || ( channel == 6 ) );
}

static void USART1ISRPostProcessor( void *argument )
{
  static constexpr uint8_t channel = 1u;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    if ( xSemaphoreTake( isrPPWakeup[ channel ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto usart = usartObjects[ channel ]; usart )
      {
        usart->postISRProcessing();
      }
    }
  }
}

static void USART2ISRPostProcessor( void *argument )
{
  static constexpr uint8_t channel = 2u;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    if ( xSemaphoreTake( isrPPWakeup[ channel ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto usart = usartObjects[ channel ]; usart )
      {
        usart->postISRProcessing();
      }
    }
  }
}

static void USART3ISRPostProcessor( void *argument )
{
  static constexpr uint8_t channel = 3u;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    if ( xSemaphoreTake( isrPPWakeup[ channel ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto usart = usartObjects[ channel ]; usart )
      {
        usart->postISRProcessing();
      }
    }
  }
}

static void USART6ISRPostProcessor( void *argument )
{
  static constexpr uint8_t channel = 6u;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    if ( xSemaphoreTake( isrPPWakeup[ channel ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto usart = usartObjects[ channel ]; usart )
      {
        usart->postISRProcessing();
      }
    }
  }
}