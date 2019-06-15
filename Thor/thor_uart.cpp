/********************************************************************************
 * File Name:
 *   thor_uart.cpp
 *
 * Description:
 *   Implements the UART interface for Thor
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Boost Includes */
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>

/* Chimera Includes */
#include <Chimera/watchdog.hpp>

/* Thor Includes */
#include <Thor/core.hpp>
#include <Thor/defaults/serial_defaults.hpp>
#include <Thor/definitions/serial_definitions.hpp>
#include <Thor/dma.hpp>
#include <Thor/headers.hpp>
#include <Thor/uart.hpp>

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
#include "mock_stm32_hal_uart.hpp"
#endif

using namespace Thor;
using namespace Thor::Serial;
using namespace Thor::UART;
using namespace Thor::GPIO;
using namespace Thor::Interrupt;
using namespace Chimera::Serial;

/*------------------------------------------------
Local Functions Declarations
------------------------------------------------*/
static constexpr bool isChannelSupported( const uint8_t channel );
static void UART4ISRPostProcessor( void *argument );
static void UART5ISRPostProcessor( void *argument );
static void UART7ISRPostProcessor( void *argument );
static void UART8ISRPostProcessor( void *argument );

static uint32_t uartClockMask( USART_TypeDef *const instance );
static UARTClass *const getUARTClassRef( USART_TypeDef *const instance );

/*------------------------------------------------
Local Data
------------------------------------------------*/
static std::array<UARTClass *, MAX_SERIAL_CHANNELS + 1> uartObjects;

/* Post Processor Thread Handles */
static std::array<TaskHandle_t, MAX_SERIAL_CHANNELS + 1> isrPPHandle;

/* Post Processor Thread Wakeup Signals */
static std::array<SemaphoreHandle_t, MAX_SERIAL_CHANNELS + 1> isrPPWakeup;

/* Post Processor Thread Function Pointers */ /* clang-format off */
static std::array<Chimera::Function::void_func_void_ptr, MAX_SERIAL_CHANNELS + 1> isrPPThread = { 
  nullptr, nullptr, nullptr, nullptr, 
  UART4ISRPostProcessor, 
  UART5ISRPostProcessor, nullptr,
  UART7ISRPostProcessor, 
  UART8ISRPostProcessor 
}; /* clang-format on */


namespace Thor::UART
{
  inline void UART_ClearIT_IDLE( UART_HandleTypeDef *const UartHandle )
  {
#if defined( STM32F7 )
    __HAL_UART_CLEAR_IT( UartHandle, UART_CLEAR_IDLEF );
#endif
  }

  inline void UART_EnableIT_IDLE( UART_HandleTypeDef *const UartHandle )
  {
    UART_ClearIT_IDLE( UartHandle );

#if defined( SIM )
#else
    __HAL_UART_ENABLE_IT( UartHandle, UART_IT_IDLE );
#endif /* SIM */
  }

  inline void UART_DisableIT_IDLE( UART_HandleTypeDef *const UartHandle )
  {
    UART_ClearIT_IDLE( UartHandle );

#if defined( SIM )
#else
    __HAL_UART_DISABLE_IT( UartHandle, UART_IT_IDLE );
#endif /* SIM */
  }

  UARTClass::UARTClass()
  {
    /*------------------------------------------------
    Register the mode change function pointers
    ------------------------------------------------*/
    modeChangeFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::BLOCKING ) ] = &UARTClass::setBlockingMode;
    modeChangeFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::INTERRUPT ) ] =
        &UARTClass::setInterruptMode;
    modeChangeFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::DMA ) ] = &UARTClass::setDMAMode;

    /*------------------------------------------------
    Register the read function pointers
    ------------------------------------------------*/
    readFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::BLOCKING ) ]  = &UARTClass::readBlocking;
    readFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::INTERRUPT ) ] = &UARTClass::readInterrupt;
    readFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::DMA ) ]       = &UARTClass::readDMA;

    /*------------------------------------------------
    Register the write function pointers
    ------------------------------------------------*/
    writeFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::BLOCKING ) ]  = &UARTClass::writeBlocking;
    writeFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::INTERRUPT ) ] = &UARTClass::writeInterrupt;
    writeFuncPtrs[ static_cast<uint8_t>( Chimera::Hardware::SubPeripheralMode::DMA ) ]       = &UARTClass::writeDMA;

    AUTO_ASYNC_RX = false;

    dmaRXReqSig = Thor::DMA::Source::NONE;
    dmaTXReqSig = Thor::DMA::Source::NONE;

    rxCompleteWakeup = nullptr;
    txCompleteWakeup = nullptr;

    awaitEventRXComplete = xSemaphoreCreateBinary();
    awaitEventTXComplete = xSemaphoreCreateBinary();


    txMode = Chimera::Hardware::SubPeripheralMode::UNKNOWN_MODE;
    rxMode = Chimera::Hardware::SubPeripheralMode::UNKNOWN_MODE;

#if defined( GMOCK_TEST )
    if ( !STM32HAL_Mock::uartMockObj )
    {
      STM32HAL_Mock::uartMockObj = std::make_shared<STM32HAL_Mock::UARTNiceMock>();
    }
#endif /* GMOCK_TEST */
  }

  UARTClass::~UARTClass()
  {
    uartObjects[ uart_channel ] = nullptr;
  }

  Chimera::Status_t UARTClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
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
      uartObjects[ channel ] = this;
      uart_channel           = channel;
      uart_handle.Init       = dflt_UART_Init;
      uart_handle.Instance   = const_cast<USART_TypeDef *>( hwConfig[ uart_channel ]->instance );

#if defined( STM32F7 )
      uart_handle.AdvancedInit = Thor::Serial::dflt_UART_AdvInit;
#endif

      /*------------------------------------------------
      Copy over the interrupt settings information
      ------------------------------------------------*/
      ITSettings_HW     = hwConfig[ uart_channel ]->IT_HW;
      ITSettings_DMA_TX = hwConfig[ uart_channel ]->dmaIT_TX;
      ITSettings_DMA_RX = hwConfig[ uart_channel ]->dmaIT_RX;

      dmaTXReqSig = Thor::Serial::DMATXRequestSignal[ uart_channel ];
      dmaRXReqSig = Thor::Serial::DMARXRequestSignal[ uart_channel ];

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

      UART_GPIO_Init();

      PeripheralState.hardware_assigned = true;
    }

    return error;
  }

  Chimera::Status_t UARTClass::begin( const Chimera::Hardware::SubPeripheralMode txMode,
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
    if ( ( error == Chimera::CommonStatusCodes::OK ) && isrPPThread[ uart_channel ] )
    {
      isrPPWakeup[ uart_channel ] = xSemaphoreCreateBinary();
      isrPPHandle[ uart_channel ] = nullptr;

      Chimera::Threading::addThread( isrPPThread[ uart_channel ], "", 500, NULL, 5, &isrPPHandle[ uart_channel ] );
    }

    return error;
  }

  Chimera::Status_t UARTClass::end()
  {
    UART_DeInit();
    UART_GPIO_DeInit();
    UART_DisableInterrupts();
    UART_DMA_DeInit( Chimera::Hardware::SubPeripheral::TX );
    UART_DMA_DeInit( Chimera::Hardware::SubPeripheral::RX );

    txMode = Chimera::Hardware::SubPeripheralMode::UNKNOWN_MODE;
    rxMode = Chimera::Hardware::SubPeripheralMode::UNKNOWN_MODE;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t UARTClass::configure( const uint32_t baud, const Chimera::Serial::CharWid width,
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

      UART_InitTypeDef init = uart_handle.Init;

      init.BaudRate = baud;
      result |= setWordLength( init, width );
      result |= setParity( init, parity );
      result |= setStopBits( init, stop );
      result |= setFlowControl( init, flow );

      if ( result )
      {
        error = UART_Init();
      }
      else
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
    }

    return error;
  }

  Chimera::Status_t UARTClass::setBaud( const uint32_t baud )
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
      UART_InitTypeDef init = uart_handle.Init;
      init.BaudRate         = baud;
      uart_handle.Init      = init;

      /*------------------------------------------------
      Clear the hardware config and re-initialize with new settings
      ------------------------------------------------*/
      UART_DeInit();
      UART_Init();
    }

    return error;
  }

  Chimera::Status_t UARTClass::setMode( const Chimera::Hardware::SubPeripheral periph,
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

  Chimera::Status_t UARTClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
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

  Chimera::Status_t UARTClass::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
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

  Chimera::Status_t UARTClass::flush( const Chimera::Hardware::SubPeripheral periph )
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

  void UARTClass::postISRProcessing()
  {
    if ( eventBits & Chimera::Event::Flags::BIT_WRITE_COMPLETE )
    {
      tx_complete = true;
      eventBits &= ~Chimera::Event::Flags::BIT_WRITE_COMPLETE;

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
        Wake up any user threads
        ------------------------------------------------*/
        if ( txCompleteWakeup )
        {
          xSemaphoreGive( *txCompleteWakeup );
        }

        /*------------------------------------------------
        Wake up whatever thread called await()
        ------------------------------------------------*/
        xSemaphoreGive( awaitEventTXComplete );
      }
    }
    else if ( eventBits & Chimera::Event::Flags::BIT_READ_COMPLETE )
    {
      if ( rxMode == Chimera::Hardware::SubPeripheralMode::INTERRUPT )
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
          uint16_t actuallyRead = uart_handle.RxXferSize - uart_handle.RxXferCount;
          for ( uint16_t x = 0; x < actuallyRead; x++ )
          {
            rxBuffers.external->push_back( rxBuffers.internal[ x ] );
          }
        }

        /*------------------------------------------------
        Go back to async RX listening
        ------------------------------------------------*/
        __HAL_UART_ENABLE_IT( &uart_handle, UART_IT_RXNE );
      }
      else if ( rxMode == Chimera::Hardware::SubPeripheralMode::DMA )
      {
        /*------------------------------------------------
        Calculate how many bytes were received by looking at remaining RX buffer space.
                    num_received = bufferMaxSize - bufferSizeRemaining
        ------------------------------------------------*/
        auto num_received = static_cast<uint32_t>( uart_handle.RxXferSize - uart_handle.hdmarx->Instance->NDTR );

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
        Thor::UART::UART_EnableIT_IDLE( &uart_handle );
        HAL_UART_Receive_DMA( &uart_handle, rxBuffers.internal, rxBuffers.internalSize );
      }

      /*------------------------------------------------
      Exit the ISR processing ensuring that we can still receive asynchronous data
      ------------------------------------------------*/
      rx_complete   = true;
      AUTO_ASYNC_RX = true;

      /*------------------------------------------------
      Let user threads know the transfer completed
      ------------------------------------------------*/
      if ( rxCompleteWakeup )
      {
        xSemaphoreGive( *rxCompleteWakeup );
      }

      /*------------------------------------------------
      Wake up whatever thread called await()
      ------------------------------------------------*/
      xSemaphoreGive( awaitEventRXComplete );
    }
  }

  Chimera::Status_t UARTClass::readAsync( uint8_t *const buffer, const size_t len )
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

  Chimera::Status_t UARTClass::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
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

  Chimera::Status_t UARTClass::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
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

  Chimera::Status_t UARTClass::attachNotifier( const Chimera::Event::Trigger event, SemaphoreHandle_t *const semphr )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !semphr )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else if ( event == Chimera::Event::Trigger::READ_COMPLETE )
    {
      rxCompleteWakeup = semphr;
    }
    else if ( event == Chimera::Event::Trigger::WRITE_COMPLETE )
    {
      txCompleteWakeup = semphr;
    }
    else
    {
      error = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    return error;
  }

  Chimera::Status_t UARTClass::detachNotifier( const Chimera::Event::Trigger event, SemaphoreHandle_t *const semphr )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( event == Chimera::Event::Trigger::READ_COMPLETE )
    {
      rxCompleteWakeup = nullptr;
    }
    else if ( event == Chimera::Event::Trigger::WRITE_COMPLETE )
    {
      txCompleteWakeup = nullptr;
    }
    else
    {
      error = Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    return error;
  }

  bool UARTClass::available( size_t *const bytes )
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

  void UARTClass::await( const Chimera::Event::Trigger event )
  {
    // TODO: Refactor into arrays keyed off of the event
    if ( ( event == Chimera::Event::Trigger::READ_COMPLETE ) && ( rxMode != Chimera::Hardware::SubPeripheralMode::BLOCKING ) )
    {
      xSemaphoreTake( awaitEventRXComplete, portMAX_DELAY );
    }

    if ( ( event == Chimera::Event::Trigger::WRITE_COMPLETE ) && ( txMode != Chimera::Hardware::SubPeripheralMode::BLOCKING ) )
    {
      xSemaphoreTake( awaitEventTXComplete, portMAX_DELAY );
    }
  }

  void UARTClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier )
  {
    /*------------------------------------------------
    Currently not supported
    ------------------------------------------------*/
    Chimera::Watchdog::invokeTimeout();
  }


  bool UARTClass::setWordLength( UART_InitTypeDef &initStruct, const Chimera::Serial::CharWid width )
  {
    switch ( static_cast<uint8_t>( width ) )
    {
      case static_cast<uint8_t>( CharWid::CW_8BIT ):
      default:
        initStruct.WordLength = UART_WORDLENGTH_8B;
        break;
    }

    /* All cases are handled, so it will always return true */
    return true;
  }

  bool UARTClass::setParity( UART_InitTypeDef &initStruct, const Chimera::Serial::Parity parity )
  {
    switch ( static_cast<uint8_t>( parity ) )
    {
      case static_cast<uint8_t>( Parity::PAR_EVEN ):
        initStruct.Parity = UART_PARITY_EVEN;
        break;

      case static_cast<uint8_t>( Parity::PAR_ODD ):
        initStruct.Parity = UART_PARITY_ODD;
        break;

      case static_cast<uint8_t>( Parity::PAR_NONE ):
      default:
        initStruct.Parity = UART_PARITY_NONE;
        break;
    }

    /* All cases are handled, so it will always return true */
    return true;
  }

  bool UARTClass::setStopBits( UART_InitTypeDef &initStruct, const Chimera::Serial::StopBits stopBits )
  {
    bool result = true;

    switch ( static_cast<uint8_t>( stopBits ) )
    {
      case static_cast<uint8_t>( StopBits::SBITS_ONE ):
      default:
        initStruct.StopBits = UART_STOPBITS_1;
        break;

      case static_cast<uint8_t>( StopBits::SBITS_ONE_POINT_FIVE ):
        result = false;
        break;

      case static_cast<uint8_t>( StopBits::SBITS_TWO ):
        initStruct.StopBits = UART_STOPBITS_2;
        break;
    }

    return result;
  }

  bool UARTClass::setFlowControl( UART_InitTypeDef &initStruct, const Chimera::Serial::FlowControl flow )
  {
    switch ( static_cast<uint8_t>( flow ) )
    {
      case static_cast<uint8_t>( FlowControl::FCTRL_HW ):
        initStruct.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
        break;

      case static_cast<uint8_t>( FlowControl::FCTRL_NONE ):
      case static_cast<uint8_t>( FlowControl::FCTRL_SW ):
      default:
        initStruct.HwFlowCtl = UART_HWCONTROL_NONE;
        break;
    }

    /* All cases are handled, so it will always return true */
    return true;
  }

  void UARTClass::setBlockingMode( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      txMode = Chimera::Hardware::SubPeripheralMode::BLOCKING;    // Must be set before the other functions

      /* Make sure RX side isn't using interrupts before disabling */
      if ( rxMode == Chimera::Hardware::SubPeripheralMode::BLOCKING )
      {
        UART_DisableInterrupts();
      }

      UART_DMA_DeInit( periph );
    }
    else
    {
      rxMode = Chimera::Hardware::SubPeripheralMode::BLOCKING;    // Must be set before the other functions

      /* Make sure TX side isn't using interrupts before disabling */
      if ( txMode == Chimera::Hardware::SubPeripheralMode::BLOCKING )
      {
        UART_DisableInterrupts();
      }

      UART_DMA_DeInit( periph );
    }
  }

  void UARTClass::setInterruptMode( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      txMode = Chimera::Hardware::SubPeripheralMode::INTERRUPT;

      UART_EnableInterrupts();
      UART_DMA_DeInit( periph );
    }
    else
    {
      rxMode = Chimera::Hardware::SubPeripheralMode::INTERRUPT;

      UART_EnableInterrupts();
      UART_DMA_DeInit( periph );
    }
  }

  void UARTClass::setDMAMode( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      txMode = Chimera::Hardware::SubPeripheralMode::DMA;

      UART_EnableInterrupts();
      UART_DMA_Init( periph );
    }
    else
    {
      rxMode        = Chimera::Hardware::SubPeripheralMode::DMA;
      AUTO_ASYNC_RX = true;

      UART_EnableInterrupts();
      UART_DMA_Init( periph );

      /* Set the idle line interrupt for asynchronously getting the end of transmission */
      UART_EnableIT_IDLE( &uart_handle );

      /* Instruct the DMA hardware to start listening for transmissions */
      memset( rxBuffers.internal, 0, rxBuffers.internalSize );
      HAL_UART_Receive_DMA( &uart_handle, rxBuffers.internal, rxBuffers.internalSize );
    }
  }

  Chimera::Status_t UARTClass::readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_BUSY;

    if ( lock( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    {
      /*------------------------------------------------
      It's possible to get into the condition where ORE is set before trying to receive some
      new data. In the current STM HAL library, all error interrupts for the blocking mode are
      disabled by default so the overrun has to be handled manually. This restores normal
      operation. A nearly exact condition of this bug is encountered here: https://goo.gl/bKi8Ps
      ------------------------------------------------*/
      UART_OverrunHandler();

#if defined( USING_FREERTOS )
      stm32Error = HAL_UART_Receive( &uart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ),
                                     pdMS_TO_TICKS( BLOCKING_TIMEOUT_MS ) );
#else
      stm32Error = HAL_UART_Receive( &uart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ),
                                     BLOCKING_TIMEOUT_MS );
#endif
      unlock();
    }

    return convertHALStatus( stm32Error );
  }

  Chimera::Status_t UARTClass::readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    /* clang-format off */
    if ( ( length <= rxBuffers.internalSize )
      && lock( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    { /* clang-format on */
      /*------------------------------------------------
      Let the ISR handler know that we explicitely asked to receive some data.
      This will cause a redirect into the correct ISR handling control flow.
      ------------------------------------------------*/
      AUTO_ASYNC_RX = false;
      memset( rxBuffers.internal, 0, rxBuffers.internalSize );
      stm32Error = HAL_UART_Receive_IT( &uart_handle, rxBuffers.internal, static_cast<uint16_t>( length ) );

      if ( stm32Error == HAL_OK )
      {
        error = Chimera::Serial::Status::RX_IN_PROGRESS;
      }
      else
      {
        error = convertHALStatus( stm32Error );
      }

      unlock();
    }
    else
    {
      error = Chimera::CommonStatusCodes::MEMORY;
    }

    return error;
  }

  Chimera::Status_t UARTClass::readDMA( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    /* clang-format off */
    if ( ( length <= rxBuffers.internalSize )
      && lock( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    { /* clang-format on */
      /*------------------------------------------------
      Let the ISR handler know that we explicitely asked to receive some data.
      This will cause a redirect into the correct handling channels.
      ------------------------------------------------*/
      AUTO_ASYNC_RX = false;

      /*------------------------------------------------
      Stop any background listening for data and restart with the new transfer
      ------------------------------------------------*/
      HAL_UART_DMAStop( &uart_handle );
      memset( rxBuffers.internal, 0, rxBuffers.internalSize );
      stm32Error = HAL_UART_Receive_DMA( &uart_handle, rxBuffers.internal, static_cast<uint16_t>( length ) );

      if ( stm32Error == HAL_OK )
      {
        error = Chimera::Serial::Status::RX_IN_PROGRESS;
      }
      else
      {
        error = convertHALStatus( stm32Error );
      }

      unlock();
    }
    else
    {
      error = Chimera::CommonStatusCodes::MEMORY;
    }

    return error;
  }

  Chimera::Status_t UARTClass::writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_BUSY;

    if ( lock( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    {
#if defined( USING_FREERTOS )
      stm32Error = HAL_UART_Transmit( &uart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ),
                                      pdMS_TO_TICKS( BLOCKING_TIMEOUT_MS ) );
#else
      stm32Error = HAL_UART_Transmit( &uart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ),
                                      BLOCKING_TIMEOUT_MS );
#endif
      unlock();
    }

    return convertHALStatus( stm32Error );
  }

  Chimera::Status_t UARTClass::writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    /* clang-format off */
    if ( txBuffers.initialized()
      && PeripheralState.tx_buffering_enabled 
      && lock( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
    { /* clang-format on */
      if ( tx_complete )
      {
        /*------------------------------------------------
        Hardware is free. Send the data directly.
        ------------------------------------------------*/
        tx_complete = false;
        stm32Error  = HAL_UART_Transmit_IT( &uart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ) );
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

      unlock();
    }
    else
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    return error;
  }

  Chimera::Status_t UARTClass::writeDMA( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    HAL_StatusTypeDef stm32Error = HAL_OK;
    Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

    /* clang-format off */
      if ( txBuffers.initialized()
        && PeripheralState.tx_buffering_enabled 
        && lock( Chimera::Threading::TIMEOUT_DONT_WAIT ) == Chimera::CommonStatusCodes::OK )
      { /* clang-format on */
      if ( tx_complete )
      {
        /*------------------------------------------------
        Hardware is free. Send the data directly.
        ------------------------------------------------*/
        tx_complete = false;
        stm32Error  = HAL_UART_Transmit_DMA( &uart_handle, const_cast<uint8_t *>( buffer ), static_cast<uint16_t>( length ) );
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

      unlock();
    }
    else
    {
      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    return error;
  }

  void UARTClass::IRQHandler_TXDMA()
  {
    HAL_DMA_IRQHandler( uart_handle.hdmatx );
  }

  void UARTClass::IRQHandler_RXDMA()
  {
    HAL_DMA_IRQHandler( uart_handle.hdmarx );
  }

  Chimera::Status_t UARTClass::UART_Init()
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    UART_EnableClock();

    if ( HAL_UART_Init( &uart_handle ) != HAL_OK )
    {
      error = Chimera::CommonStatusCodes::FAIL;
    }

    setMode( Chimera::Hardware::SubPeripheral::TX, Chimera::Hardware::SubPeripheralMode::BLOCKING );
    setMode( Chimera::Hardware::SubPeripheral::RX, Chimera::Hardware::SubPeripheralMode::BLOCKING );

    PeripheralState.configured = true;
    return error;
  }

  void UARTClass::UART_DeInit()
  {
    HAL_UART_DeInit( &uart_handle );
    PeripheralState.configured = false;
  }

  void UARTClass::UART_EnableClock()
  {
    using namespace Thor::Serial;

#if defined( SIM )
#else
#if defined( TARGET_STM32F7 ) || defined( TARGET_STM32F4 )
    RCC->APB1ENR |= ( uartClockMask( uart_handle.Instance ) );
#endif
#endif /* SIM */
  }

  void UARTClass::UART_DisableClock()
  {
    using namespace Thor::Serial;

#if defined( SIM )
#else
#if defined( TARGET_STM32F7 ) || defined( TARGET_STM32F4 )
    RCC->APB1ENR &= ~( uartClockMask( uart_handle.Instance ) );
#endif
#endif /* SIM */
  }

  void UARTClass::UART_DMA_EnableClock()
  {
    /* Global DMA Clock options. Only turn on capability is
    provided due to other peripherals possibly using DMA. */
#if defined( SIM )
#else
    if ( __HAL_RCC_DMA1_IS_CLK_DISABLED() )
    {
      __HAL_RCC_DMA1_CLK_ENABLE();
    }

    if ( __HAL_RCC_DMA2_IS_CLK_DISABLED() )
    {
      __HAL_RCC_DMA2_CLK_ENABLE();
    }
#endif /* SIM */
  }

  void UARTClass::UART_EnableInterrupts()
  {
    HAL_NVIC_DisableIRQ( ITSettings_HW.IRQn );
    HAL_NVIC_ClearPendingIRQ( ITSettings_HW.IRQn );
    HAL_NVIC_SetPriority( ITSettings_HW.IRQn, ITSettings_HW.preemptPriority, ITSettings_HW.subPriority );
    HAL_NVIC_EnableIRQ( ITSettings_HW.IRQn );

    /*------------------------------------------------
    Make sure we are able to asynchronously receive some data if it gets sent
    ------------------------------------------------*/
    AUTO_ASYNC_RX = true;
    if ( rxMode == Chimera::Hardware::SubPeripheralMode::INTERRUPT )
    {
      /*------------------------------------------------
      In interrupt mode, we have to handle the RX FIFO on a byte by byte
      basis otherwise an overrun error is generated.
      ------------------------------------------------*/
#if defined( SIM )
#else
      __HAL_UART_ENABLE_IT( &uart_handle, UART_IT_RXNE );
#endif /* SIM */
    }

    PeripheralState.interrupts_enabled = true;
  }

  void UARTClass::UART_DisableInterrupts()
  {
#if defined( SIM )
#else
    __HAL_UART_DISABLE_IT( &uart_handle, UART_IT_IDLE );
    __HAL_UART_DISABLE_IT( &uart_handle, UART_IT_RXNE );
#endif /* SIM */

    HAL_NVIC_DisableIRQ( ITSettings_HW.IRQn );
    HAL_NVIC_ClearPendingIRQ( ITSettings_HW.IRQn );

    PeripheralState.interrupts_enabled = false;
  }

  void UARTClass::UART_GPIO_Init()
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

  void UARTClass::UART_GPIO_DeInit()
  {
    // TODO: Implement GPIO DeInit
  }

  void UARTClass::UART_DMA_Init( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      UART_DMA_EnableClock();
      hdma_uart_tx.Instance = const_cast<DMA_Stream_TypeDef *>( hwConfig[ uart_channel ]->dmaTX.Instance );

      /* Grab the default init settings and modify for the specific hardware */
      hdma_uart_tx.Init           = dflt_DMA_Init_TX;
      hdma_uart_tx.Init.Channel   = hwConfig[ uart_channel ]->dmaTX.channel;
      hdma_uart_tx.Init.Direction = hwConfig[ uart_channel ]->dmaTX.direction;

      /* Hard error if initialization fails. */
      assert( HAL_DMA_Init( &hdma_uart_tx ) == HAL_OK );
      __HAL_LINKDMA( &uart_handle, hdmatx, hdma_uart_tx );

      /*------------------------------------------------
      Attach the class specific implementation TX DMA handler
      ------------------------------------------------*/
      Thor::DMA::requestHandlers[ dmaTXReqSig ] = boost::bind( &UARTClass::IRQHandler_TXDMA, this );

      UART_DMA_EnableIT( periph );

      PeripheralState.dma_enabled_tx = true;
    }
    else
    {
      UART_DMA_EnableClock();
      hdma_uart_rx.Instance = const_cast<DMA_Stream_TypeDef *>( hwConfig[ uart_channel ]->dmaRX.Instance );

      /* Grab the default init settings and modify for the specific hardware */
      hdma_uart_rx.Init           = dflt_DMA_Init_RX;
      hdma_uart_rx.Init.Channel   = hwConfig[ uart_channel ]->dmaRX.channel;
      hdma_uart_rx.Init.Direction = hwConfig[ uart_channel ]->dmaRX.direction;

      /* Hard error if initialization fails. */
      assert( HAL_DMA_Init( &hdma_uart_rx ) == HAL_OK );
      __HAL_LINKDMA( &uart_handle, hdmarx, hdma_uart_rx );

      /*------------------------------------------------
      Attach the class specific implementation TX DMA handler
      ------------------------------------------------*/
      Thor::DMA::requestHandlers[ dmaRXReqSig ] = boost::bind( &UARTClass::IRQHandler_RXDMA, this );

      UART_DMA_EnableIT( periph );

      PeripheralState.dma_enabled_rx = true;
    }
  }

  void UARTClass::UART_DMA_DeInit( const Chimera::Hardware::SubPeripheral periph )
  {
    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      if ( !PeripheralState.dma_enabled_tx )
        return;

      HAL_DMA_Abort( uart_handle.hdmatx );
      HAL_DMA_DeInit( uart_handle.hdmatx );
      UART_DMA_DisableIT( periph );
      Thor::DMA::requestHandlers[ dmaTXReqSig ].clear();

      PeripheralState.dma_enabled_tx = false;
    }
    else
    {
      if ( !PeripheralState.dma_enabled_rx )
        return;

      HAL_DMA_Abort( uart_handle.hdmarx );
      HAL_DMA_DeInit( uart_handle.hdmarx );
      UART_DMA_DisableIT( periph );
      Thor::DMA::requestHandlers[ dmaRXReqSig ].clear();

      PeripheralState.dma_enabled_rx = false;
    }
  }

  void UARTClass::UART_DMA_EnableIT( const Chimera::Hardware::SubPeripheral periph )
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

  void UARTClass::UART_DMA_DisableIT( const Chimera::Hardware::SubPeripheral periph )
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

  void UARTClass::UART_OverrunHandler()
  {
#if defined( STM32F7 )
    __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_OREF );
    uart_handle.Instance->RDR;
#endif
  }

#if defined( STM32F4 )
  void UARTClass::IRQHandler()
  {
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
      volatile uint32_t isrflags = READ_REG( uart_handle.Instance->SR );
      volatile uint32_t data_reg = READ_REG( uart_handle.Instance->DR );
      volatile uint32_t cr1      = READ_REG( uart_handle.Instance->CR1 );

      /*------------------------------------------------
      Prepare necessary flags for ISR flow control
      ------------------------------------------------*/
      bool RX_DATA_READY    = ( ( isrflags & UART_FLAG_RXNE ) == UART_FLAG_RXNE );
      bool RX_LINE_IDLE     = ( ( isrflags & UART_FLAG_IDLE ) == UART_FLAG_IDLE );
      bool RX_DATA_READY_IE = ( ( cr1 & USART_CR1_RXNEIE ) == USART_CR1_RXNEIE );
      bool RX_LINE_IDLE_IE  = ( ( cr1 & USART_CR1_IDLEIE ) == USART_CR1_IDLEIE );

      /*------------------------------------------------
      RX In Progress
      ------------------------------------------------*/
      if ( RX_DATA_READY && RX_DATA_READY_IE && uart_handle.gState != HAL_UART_STATE_BUSY_TX )
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
            UART_EnableIT_IDLE( &uart_handle );
            memset( rxBuffers.internal, 0, rxBuffers.internalSize );
          }

          /*------------------------------------------------
          Buffer the received byte to our internal buffer
          ------------------------------------------------*/
          if ( rxMode == Chimera::Hardware::SubPeripheralMode::INTERRUPT && ( asyncRXDataSize < rxBuffers.internalSize ) )
          {
            rxBuffers.internal[ asyncRXDataSize ] = static_cast<uint8_t>( data_reg );
            asyncRXDataSize += 1u;
          }
          else if ( rxMode != Chimera::Hardware::SubPeripheralMode::DMA )
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
        if ( rxMode == Chimera::Hardware::SubPeripheralMode::INTERRUPT )
        {
          /*------------------------------------------------
          Turn off the idle line interrupt so we aren't interrupted again
          while trying to process all the information just received.
          ------------------------------------------------*/
          UART_DisableIT_IDLE( &uart_handle );

          /*------------------------------------------------
          Manually call this function because HAL_UART_IRQHandler won't get called
          and it normally handles this operation.
          ------------------------------------------------*/
          HAL_UART_RxCpltCallback( &uart_handle );
        }
        else /* DMA */
        {
          /*------------------------------------------------
          Turn off the idle line interrupt so we aren't interrupted again
          while trying to process all the information just received.
          ------------------------------------------------*/
          UART_DisableIT_IDLE( &uart_handle );

          /*------------------------------------------------
          Force DMA hard reset to trigger the DMA RX Complete handler
          ------------------------------------------------*/
          __HAL_DMA_DISABLE( uart_handle.hdmarx );
        }
      }
    }

    /*------------------------------------------------
    Handle when the standard IRQHandler is called:

    1. We are TX-ing in IT or DMA mode (getting to this func means we are in those modes)
    2. We are RX-ing an expected asynchronous transfer that the user started
    ------------------------------------------------*/
    if ( !AUTO_ASYNC_RX || uart_handle.gState == HAL_UART_STATE_BUSY_TX )
    {
      HAL_UART_IRQHandler( &uart_handle );
    }
  }
#endif /* STM32F4 */

#if defined( STM32F7 )
  void UARTClass::IRQHandler()
  {
//    bool RX_DATA_READY   = __HAL_UART_GET_FLAG( &uart_handle, UART_FLAG_RXNE );
//    bool RX_LINE_IDLE    = __HAL_UART_GET_FLAG( &uart_handle, UART_FLAG_IDLE );
//    bool RX_LINE_IDLE_EN = __HAL_UART_GET_IT_SOURCE( &uart_handle, UART_IT_IDLE );
//
//    /*------------------------------------
//     * Handle Asynchronous RX (Interrupt and DMA Mode)
//     *------------------------------------*/
//    /* RX In Progress */
//    if ( RX_ASYNC && RX_DATA_READY && uart_handle.gState != HAL_UART_STATE_BUSY_TX )
//    {
//      uint32_t isrflags   = READ_REG( uart_handle.Instance->ISR );
//      uint32_t errorflags = ( isrflags & ( uint32_t )( USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE ) );
//
//      if ( errorflags == RESET )
//      {
//        /* Detected new RX of unknown size */
//        if ( asyncRXDataSize == 0 )
//        {
//          memset( RX_Queue[ RXQueueIdx ], 0, UART_QUEUE_BUFFER_SIZE );
//
//          /* Enable UART_IT_IDLE to detect transmission end.
//           * Sometimes IDLEF is set before interrupt enable, which will immediately trigger this ISR,
//           * and cause reception of only 1 character. Clear first to ensure accurate idle line trigger. */
//          __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_IDLEF );
//          __HAL_UART_ENABLE_IT( &uart_handle, UART_IT_IDLE );
//        }
//
//        /* Buffer the new data */
//        if ( rxMode == Chimera::Hardware::SubPeripheralMode::INTERRUPT && ( asyncRXDataSize < UART_QUEUE_BUFFER_SIZE ) )
//        {
//          RX_Queue[ RXQueueIdx ][ asyncRXDataSize ] = ( uint8_t )( uart_handle.Instance->RDR & ( uint8_t )0xFF );
//          asyncRXDataSize += 1u;
//        }
//        else
//        {
//          /* Forced to read data without being able to store it.
//           * Really need to put some kind of error thing here...*/
//          uart_handle.Instance->RDR;
//        }
//      }
//      else
//      {
//        // Do something more useful later
//        __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_PEF );
//        __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_FEF );
//        __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_NEF );
//        __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_OREF );
//      }
//    }
//
//    /* RX Complete */
//    if ( RX_ASYNC && RX_LINE_IDLE_EN && RX_LINE_IDLE )
//    {
//      if ( rxMode == Chimera::Hardware::SubPeripheralMode::INTERRUPT )
//      {
//        UART_DisableIT_IDLE( &uart_handle );
//
//        /* Copy data received to the internal buffer */
//        RX_tempPacket.data_ptr = RX_Queue[ RXQueueIdx ];
//        RX_tempPacket.length   = asyncRXDataSize;
//        RXPacketBuffer.push_back( RX_tempPacket );
//
//        /* Clean up the class variables to prepare for a new reception */
//        rx_complete     = true;
//        asyncRXDataSize = 0;
//        totalUnreadPackets++;
//        _rxIncrQueueIdx();
//
//        /* Finally, call this here because the normal HAL_UART_IRQHandler does not get called
//         * due to the asynchronous nature of operation. */
//        HAL_UART_RxCpltCallback( &uart_handle );
//      }
//      else if ( rxMode == Chimera::Hardware::SubPeripheralMode::DMA )
//      {
//        auto num_received = ( size_t )( uart_handle.RxXferSize - uart_handle.hdmarx->Instance->NDTR );
//
//        if ( num_received != 0 )
//        {
//          UART_DisableIT_IDLE( &uart_handle );
//          rx_complete = true;
//          totalUnreadPackets++;
//
//          /* Force DMA hard reset to trigger the DMA RX Complete handler */
//          __HAL_DMA_DISABLE( uart_handle.hdmarx );
//        }
//        else
//        {
//          /* ISR was randomly triggered. Clear any errant flags that might be enabled.*/
//          UART_ClearIT_IDLE( &uart_handle );
//        }
//      }
//    }
//
//    /*------------------------------------
//     * Handle Synchronous RX or Asynchronous TX (Interrupt Mode)
//     *------------------------------------*/
//    if ( !RX_ASYNC || uart_handle.gState == HAL_UART_STATE_BUSY_TX )
//    {
//      HAL_UART_IRQHandler( &uart_handle );
//    }
  }
#endif /* STM32F7 */
}    // namespace Thor::UART

#if !defined( GMOCK_TEST )
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *UartHandle )
{
  const auto uart = getUARTClassRef( UartHandle->Instance );

  if ( uart )
  {
    uart->eventBits |= Chimera::Event::Flags::BIT_WRITE_COMPLETE;

    /*------------------------------------------------
    Wake up and immediately switch to the user-land ISR
    handler thread. Should be a very high priority thread.
    ------------------------------------------------*/
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( isrPPWakeup[ uart->uart_channel ], &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *UartHandle )
{
  const auto uart = getUARTClassRef( UartHandle->Instance );

  if ( uart )
  {
    uart->eventBits |= Chimera::Event::Flags::BIT_READ_COMPLETE;

    /*------------------------------------------------
    Wake up and immediately switch to the user-land ISR
    handler thread. Should be a very high priority thread.
    ------------------------------------------------*/
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( isrPPWakeup[ uart->uart_channel ], &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
}

void HAL_UART_TxHalfCpltCallback( UART_HandleTypeDef *UartHandle )
{
}

void HAL_UART_RxHalfCpltCallback( UART_HandleTypeDef *UartHandle )
{
}

void HAL_UART_ErrorCallback( UART_HandleTypeDef *UartHandle )
{
}

#endif /* !GMOCK_TEST */

void UART4_IRQHandler( void )
{
  if ( uartObjects[ 4 ] )
  {
    uartObjects[ 4 ]->IRQHandler();
  }
}

void UART5_IRQHandler( void )
{
  if ( uartObjects[ 5 ] )
  {
    uartObjects[ 5 ]->IRQHandler();
  }
}

void UART7_IRQHandler( void )
{
  if ( uartObjects[ 7 ] )
  {
    uartObjects[ 7 ]->IRQHandler();
  }
}

void UART8_IRQHandler( void )
{
  if ( uartObjects[ 8 ] )
  {
    uartObjects[ 8 ]->IRQHandler();
  }
}

static uint32_t uartClockMask( USART_TypeDef *const instance )
{
  /* Simply converts the pointer into the raw numerical address value, which be compared against
  the peripheral base address. UARTx is simply (USART_TypeDef*)UARTx_Base. */
  auto i = reinterpret_cast<std::uintptr_t>( instance );
  switch ( i )
  {
#if defined( STM32F446xx ) || defined( STM32F767xx )
#if defined( UART4 )
    case UART4_BASE:
      return RCC_APB1ENR_UART4EN;
      break;
#endif
#if defined( UART5 )
    case UART5_BASE:
      return RCC_APB1ENR_UART5EN;
      break;
#endif
#if defined( UART7 )
    case UART7_BASE:
      return RCC_APB1ENR_UART7EN;
      break;
#endif
#if defined( UART8 )
    case UART8_BASE:
      return RCC_APB1ENR_UART8EN;
      break;
#endif
#endif /* !STM32F446xx  !STM32F767xx */

    default:
      return 0u;
      break;
  };
};

static UARTClass *const getUARTClassRef( USART_TypeDef *const instance )
{
  /*------------------------------------------------
  Simply converts the pointer into the raw numerical address value, which be compared against
  the peripheral base address. UARTx is simply (USART_TypeDef*)UARTx_Base.
  ------------------------------------------------*/
  auto i = reinterpret_cast<std::uintptr_t>( instance );
  switch ( i )
  {
#if defined( UART1 )
    case UART1_BASE:
      return uartObjects[ 1 ];
      break;
#endif
#if defined( UART2 )
    case UART2_BASE:
      return uartObjects[ 2 ];
      break;
#endif
#if defined( UART3 )
    case UART3_BASE:
      return uartObjects[ 3 ];
      break;
#endif
#if defined( UART4 )
    case UART4_BASE:
      return uartObjects[ 4 ];
      break;
#endif
#if defined( UART5 )
    case UART5_BASE:
      return uartObjects[ 5 ];
      break;
#endif
#if defined( UART6 )
    case UART6_BASE:
      return uartObjects[ 6 ];
      break;
#endif
#if defined( UART7 )
    case UART7_BASE:
      return uartObjects[ 7 ];
      break;
#endif
#if defined( UART8 )
    case UART8_BASE:
      return uartObjects[ 8 ];
      break;
#endif

    default:
      return uartObjects[ 0 ];
      break;
  };
};

static constexpr bool isChannelSupported( const uint8_t channel )
{
  return ( ( channel == 4 ) || ( channel == 5 ) || ( channel == 7 ) || ( channel == 8 ) );
}

static void UART4ISRPostProcessor( void *argument )
{
  static constexpr uint8_t channel = 4u;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    if ( xSemaphoreTake( isrPPWakeup[ channel ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto uart = uartObjects[ channel ]; uart )
      {
        uart->postISRProcessing();
      }
    }
  }
}

static void UART5ISRPostProcessor( void *argument )
{
  static constexpr uint8_t channel = 5u;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    if ( xSemaphoreTake( isrPPWakeup[ channel ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto uart = uartObjects[ channel ]; uart )
      {
        uart->postISRProcessing();
      }
    }
  }
}

static void UART7ISRPostProcessor( void *argument )
{
  static constexpr uint8_t channel = 7u;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    if ( xSemaphoreTake( isrPPWakeup[ channel ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto uart = uartObjects[ channel ]; uart )
      {
        uart->postISRProcessing();
      }
    }
  }
}

static void UART8ISRPostProcessor( void *argument )
{
  static constexpr uint8_t channel = 8u;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    if ( xSemaphoreTake( isrPPWakeup[ channel ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto uart = uartObjects[ channel ]; uart )
      {
        uart->postISRProcessing();
      }
    }
  }
}