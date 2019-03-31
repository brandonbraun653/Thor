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

/* Thor Includes */
#include <Thor/definitions.hpp>
#include <Thor/defaults.hpp>
#include <Thor/dma.hpp>
#include <Thor/usart.hpp>
#include <Thor/exceptions.hpp>

#if defined( USING_FREERTOS )
#include <Thor/exti.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
}
#endif

static std::array<SemaphoreHandle_t, Thor::Serial::MAX_SERIAL_CHANNELS + 1> usartSemphrs;
TaskTrigger usartTaskTrigger;
#endif /* USING_FREERTOS */

using namespace Thor;
using namespace Thor::Serial;
using namespace Thor::USART;
using namespace Thor::GPIO;
using namespace Thor::Interrupt;
using namespace Thor::Defaults::Serial;
using namespace Chimera::Serial;

static std::array<USARTClass_sPtr, MAX_SERIAL_CHANNELS + 1> usartObjects;

static const USARTClass_sPtr &getUSARTClassRef( USART_TypeDef *const instance )
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

namespace Thor
{
  namespace USART
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
      __HAL_USART_ENABLE_IT( UsartHandle, USART_IT_IDLE );
    }

    inline void USART_DisableIT_IDLE( USART_HandleTypeDef *const UsartHandle )
    {
      USART_ClearIT_IDLE( UsartHandle );
      __HAL_USART_DISABLE_IT( UsartHandle, USART_IT_IDLE );
    }

    USARTClass::USARTClass()
    {
      /*------------------------------------------------
      Register the mode change function pointers
      ------------------------------------------------*/
      modeChangeFuncPtrs[ static_cast<uint8_t>( Modes::BLOCKING ) ]  = &USARTClass::setBlockingMode;
      modeChangeFuncPtrs[ static_cast<uint8_t>( Modes::INTERRUPT ) ] = &USARTClass::setInterruptMode;
      modeChangeFuncPtrs[ static_cast<uint8_t>( Modes::DMA ) ]       = &USARTClass::setDMAMode;

      /*------------------------------------------------
      Register the read function pointers
      ------------------------------------------------*/
      readFuncPtrs[ static_cast<uint8_t>( Modes::BLOCKING ) ]  = &USARTClass::readBlocking;
      readFuncPtrs[ static_cast<uint8_t>( Modes::INTERRUPT ) ] = &USARTClass::readInterrupt;
      readFuncPtrs[ static_cast<uint8_t>( Modes::DMA ) ]       = &USARTClass::readDMA;

      /*------------------------------------------------
      Register the write function pointers
      ------------------------------------------------*/
      writeFuncPtrs[ static_cast<uint8_t>( Modes::BLOCKING ) ]  = &USARTClass::writeBlocking;
      writeFuncPtrs[ static_cast<uint8_t>( Modes::INTERRUPT ) ] = &USARTClass::writeInterrupt;
      writeFuncPtrs[ static_cast<uint8_t>( Modes::DMA ) ]       = &USARTClass::writeDMA;

      AUTO_ASYNC_RX = false;

      dmaRXReqSig = Thor::DMA::Source::NONE;
      dmaTXReqSig = Thor::DMA::Source::NONE;
    }

    USARTClass::~USARTClass()
    {
      delete[] rxInternalBuffer;
      delete[] txInternalBuffer;
    }

    USARTClass_sPtr USARTClass::create( const uint8_t channel, const size_t bufferSize )
    {
      USARTClass_sPtr newClass( new USARTClass() );

      if ( channel <= ( Serial::MAX_SERIAL_CHANNELS + 1 ) )
      {
        usartObjects[ channel ] = newClass;
        newClass->assignRXBuffer( new uint8_t[ bufferSize ](), bufferSize );
        newClass->assignTXBuffer( new uint8_t[ bufferSize ](), bufferSize );
      }
      else
      {
        newClass = nullptr;
      }

      return newClass;
    }

    Chimera::Status_t USARTClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
    {
      asyncRXDataSize = 0;

      /*------------------------------------------------
      Assign the default handle settings
      ------------------------------------------------*/
      usart_channel         = channel;
      usart_handle.Init     = Defaults::Serial::dflt_USART_Init;
      usart_handle.Instance = const_cast<USART_TypeDef *>( hwConfig[ usart_channel ]->instance );

#if defined( STM32F7 )
      usart_handle.AdvancedInit = Defaults::Serial::dflt_USART_AdvInit;
#endif

      /*------------------------------------------------
      Copy over the interrupt settings information
      ------------------------------------------------*/
      ITSettings_HW     = hwConfig[ usart_channel ]->IT_HW;
      ITSettings_DMA_TX = hwConfig[ usart_channel ]->dmaIT_TX;
      ITSettings_DMA_RX = hwConfig[ usart_channel ]->dmaIT_RX;

      dmaTXReqSig = Thor::DMA::serialTXReq[ usart_channel ];
      dmaRXReqSig = Thor::DMA::serialRXReq[ usart_channel ];

      /*------------------------------------------------
      Initialize the GPIO pins
      ------------------------------------------------*/
      tx_pin = std::make_shared<Thor::GPIO::GPIOClass>();
      tx_pin->initAdvanced( Thor::GPIO::convertPort( pins.tx.port ), Thor::GPIO::convertPinNum( pins.tx.pin ),
                            PinSpeed::ULTRA_SPD, pins.tx.alternate );

      rx_pin = std::make_shared<Thor::GPIO::GPIOClass>();
      rx_pin->initAdvanced( Thor::GPIO::convertPort( pins.rx.port ), Thor::GPIO::convertPinNum( pins.rx.pin ),
                            PinSpeed::ULTRA_SPD, pins.rx.alternate );

      USART_GPIO_Init();

#if defined( USING_FREERTOS )
      usartSemphrs[ usart_channel ] = xSemaphoreCreateCounting( USART_QUEUE_SIZE, USART_QUEUE_SIZE );
#endif

      hardware_assigned = true;
      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t USARTClass::begin( const Modes txMode, const Modes rxMode )
    {
      setMode( SubPeripheral::TX, txMode );
      setMode( SubPeripheral::RX, rxMode );

      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t USARTClass::end()
    {
      USART_DeInit();
      USART_GPIO_DeInit();
      USART_DisableInterrupts();
      USART_DMA_DeInit( SubPeripheral::TX );
      USART_DMA_DeInit( SubPeripheral::RX );

      txMode = Modes::MODE_UNDEFINED;
      rxMode = Modes::MODE_UNDEFINED;

      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t USARTClass::configure( const uint32_t baud, const Chimera::Serial::CharWid width,
                                             const Chimera::Serial::Parity parity, const Chimera::Serial::StopBits stop,
                                             const Chimera::Serial::FlowControl flow )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( !hardware_assigned )
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

      if ( !hardware_assigned )
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

    Chimera::Status_t USARTClass::setMode( const SubPeripheral periph, const Modes mode )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;
      auto iter               = static_cast<uint8_t>( mode );

      if ( !hardware_assigned )
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }
      else if ( iter >= modeChangeFuncPtrs.size() )
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
      else
      {
        ( this->*( modeChangeFuncPtrs[ iter ] ) )( periph );
      }

      return error;
    }

    Chimera::Status_t USARTClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;
      auto iter               = static_cast<uint8_t>( txMode );

      if ( !PeripheralState.gpio_enabled || !PeripheralState.enabled )
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }
      else if ( iter >= modeChangeFuncPtrs.size() )
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

      if ( !PeripheralState.gpio_enabled || !PeripheralState.enabled )
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

    Chimera::Status_t USARTClass::flush( const Chimera::Serial::SubPeripheral periph )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( periph == SubPeripheral::TX )
      {
        if ( txInternalBuffer )
        {
          memset( txInternalBuffer, 0, txInternalBufferSize );
        }

        if ( txUserBuffer )
        {
          txUserBuffer->clear();
        }
      }
      else if ( periph == SubPeripheral::RX )
      {
        if ( rxInternalBuffer )
        {
          memset( rxInternalBuffer, 0, rxInternalBufferSize );
        }

        if ( rxUserBuffer )
        {
          rxUserBuffer->clear();
        }
      }
      else
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }

      return error;
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

        while ( !rxUserBuffer->empty() && ( bytesRead < len ) )
        {
          buffer[ bytesRead ] = rxUserBuffer->front();
          rxUserBuffer->pop_front();
          bytesRead++;
        }

        if ( bytesRead != len )
        {
          error = Chimera::CommonStatusCodes::EMPTY;
        }
      }

      return error;
    }

    Chimera::Status_t USARTClass::enableBuffering( const Chimera::Serial::SubPeripheral periph,
                                                   boost::circular_buffer<uint8_t> *const buffer )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( periph == SubPeripheral::TX )
      {
        txUserBuffer                         = buffer;
        PeripheralState.tx_buffering_enabled = true;
      }
      else if ( periph == SubPeripheral::RX )
      {
        rxUserBuffer                         = buffer;
        PeripheralState.rx_buffering_enabled = true;
      }
      else
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }

      return error;
    }

    Chimera::Status_t USARTClass::disableBuffering( const Chimera::Serial::SubPeripheral periph )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( periph == SubPeripheral::TX )
      {
        PeripheralState.tx_buffering_enabled = false;
      }
      else if ( periph == SubPeripheral::RX )
      {
        PeripheralState.tx_buffering_enabled = false;
      }
      else
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }

      return error;
    }

#if defined( USING_FREERTOS )
    Chimera::Status_t USARTClass::attachEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t USARTClass::removeEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }
#endif

    bool USARTClass::available( size_t *const bytes )
    {
      bool retval = false;

      if ( PeripheralState.rx_buffering_enabled && !rxUserBuffer->empty() )
      {
        retval = true;
        if ( bytes )
        {
          *bytes = rxUserBuffer->size();
        }
      }

      return retval;
    }

    void USARTClass::assignRXBuffer( uint8_t *const buffer, const size_t size )
    {
      rxInternalBuffer     = buffer;
      rxInternalBufferSize = size;
    }

    void USARTClass::assignTXBuffer( uint8_t *const buffer, const size_t size )
    {
      txInternalBuffer     = buffer;
      txInternalBufferSize = size;
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

    void USARTClass::setBlockingMode( const Chimera::Serial::SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
      {
        txMode = Modes::BLOCKING;    // Must be set before the other functions

        /* Make sure RX side isn't using interrupts before disabling */
        if ( rxMode == Modes::BLOCKING )
        {
          USART_DisableInterrupts();
        }

        USART_DMA_DeInit( periph );
      }
      else
      {
        rxMode = Modes::BLOCKING;    // Must be set before the other functions

        /* Make sure TX side isn't using interrupts before disabling */
        if ( txMode == Modes::BLOCKING )
        {
          USART_DisableInterrupts();
        }

        USART_DMA_DeInit( periph );
      }
    }

    void USARTClass::setInterruptMode( const Chimera::Serial::SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
      {
        txMode = Modes::INTERRUPT;

        USART_EnableInterrupts();
        USART_DMA_DeInit( periph );
      }
      else
      {
        rxMode = Modes::INTERRUPT;

        USART_EnableInterrupts();
        USART_DMA_DeInit( periph );
      }
    }

    void USARTClass::setDMAMode( const Chimera::Serial::SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
      {
        txMode = Modes::DMA;

        USART_EnableInterrupts();
        USART_DMA_Init( periph );
      }
      else
      {
        rxMode        = Modes::DMA;
        AUTO_ASYNC_RX = true;

        USART_EnableInterrupts();
        USART_DMA_Init( periph );

        /* Set the idle line interrupt for asynchronously getting the end of transmission */
        USART_EnableIT_IDLE( &usart_handle );

        /* Instruct the DMA hardware to start listening for transmissions */
        memset( rxInternalBuffer, 0, rxInternalBufferSize );
        HAL_USART_Receive_DMA( &usart_handle, rxInternalBuffer, rxInternalBufferSize );
      }
    }

    Chimera::Status_t USARTClass::readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
    {
      HAL_StatusTypeDef stm32Error = HAL_OK;

      /*------------------------------------------------
      It's possible to get into the condition where ORE is set before trying to receive some
      new data. In the current STM HAL library, all error interrupts for the blocking mode are
      disabled by default so the overrun has to be handled manually. This restores normal
      operation. A nearly exact condition of this bug is encountered here: https://goo.gl/bKi8Ps
      ------------------------------------------------*/
      USART_OverrunHandler();

#if defined( USING_FREERTOS )
      stm32Error =
          HAL_USART_Receive( &usart_handle, const_cast<uint8_t *>( buffer ), length, pdMS_TO_TICKS( BLOCKING_TIMEOUT_MS ) );
#else
      stm32Error = HAL_USART_Receive( &usart_handle, const_cast<uint8_t *>( buffer ), length, BLOCKING_TIMEOUT_MS );
#endif
      return convertHALStatus( stm32Error );
    }

    Chimera::Status_t USARTClass::readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
    {
      HAL_StatusTypeDef stm32Error = HAL_OK;
      Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

      if ( length <= rxInternalBufferSize )
      {
        /*------------------------------------------------
        Let the ISR handler know that we explicitely asked to receive some data.
        This will cause a redirect into the correct ISR handling control flow.
        ------------------------------------------------*/
        AUTO_ASYNC_RX = false;
        memset( rxInternalBuffer, 0, rxInternalBufferSize );
        stm32Error = HAL_USART_Receive_IT( &usart_handle, rxInternalBuffer, length );

        if ( stm32Error == HAL_OK )
        {
          error = Chimera::Serial::Status::RX_IN_PROGRESS;
        }
        else
        {
          error = convertHALStatus( stm32Error );
        }
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

      if ( length <= rxInternalBufferSize )
      {
        /*------------------------------------------------
        Let the ISR handler know that we explicitely asked to receive some data.
        This will cause a redirect into the correct handling channels.
        ------------------------------------------------*/
        AUTO_ASYNC_RX = false;

        /*------------------------------------------------
        Stop any background listening for data and restart with the new transfer
        ------------------------------------------------*/
        HAL_USART_DMAStop( &usart_handle );
        memset( rxInternalBuffer, 0, rxInternalBufferSize );
        stm32Error = HAL_USART_Receive_DMA( &usart_handle, rxInternalBuffer, length );

        if ( stm32Error == HAL_OK )
        {
          error = Chimera::Serial::Status::RX_IN_PROGRESS;
        }
        else
        {
          error = convertHALStatus( stm32Error );
        }
      }
      else
      {
        error = Chimera::CommonStatusCodes::MEMORY;
      }

      return error;
    }

    Chimera::Status_t USARTClass::writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
    {
      HAL_StatusTypeDef stm32Error = HAL_OK;

#if defined( USING_FREERTOS )
      stm32Error =
          HAL_USART_Transmit( &usart_handle, const_cast<uint8_t *>( buffer ), length, pdMS_TO_TICKS( BLOCKING_TIMEOUT_MS ) );
#else
      stm32Error = HAL_USART_Transmit( &usart_handle, const_cast<uint8_t *>( buffer ), length, BLOCKING_TIMEOUT_MS );
#endif
      return convertHALStatus( stm32Error );
    }

    Chimera::Status_t USARTClass::writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
    {
      HAL_StatusTypeDef stm32Error = HAL_OK;
      Chimera::Status_t error      = Chimera::CommonStatusCodes::OK;

      if ( txUserBuffer && PeripheralState.tx_buffering_enabled )
      {
        if ( tx_complete )
        {
          /*------------------------------------------------
          Hardware is free. Send the data directly.
          ------------------------------------------------*/
          tx_complete = false;
          stm32Error  = HAL_USART_Transmit_IT( &usart_handle, const_cast<uint8_t *>( buffer ), length );
          error       = convertHALStatus( stm32Error );
        }
        else
        {
          /*------------------------------------------------
          Queue up everything to send later
          ------------------------------------------------*/
          error = Chimera::CommonStatusCodes::BUSY;
          for ( uint32_t x = 0; x < length; x++ )
          {
            txUserBuffer->push_back( *( buffer + x ) );
          }
        }
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

      if ( txUserBuffer && PeripheralState.tx_buffering_enabled )
      {
        if ( tx_complete )
        {
          /*------------------------------------------------
          Hardware is free. Send the data directly.
          ------------------------------------------------*/
          tx_complete = false;
          stm32Error  = HAL_USART_Transmit_DMA( &usart_handle, const_cast<uint8_t *>( buffer ), length );
          error       = convertHALStatus( stm32Error );
        }
        else
        {
          /*------------------------------------------------
          Queue up everything to send later
          ------------------------------------------------*/
          error = Chimera::CommonStatusCodes::BUSY;
          for ( uint32_t x = 0; x < length; x++ )
          {
            txUserBuffer->push_back( *( buffer + x ) );
          }
        }
      }
      else
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }

      return error;
    }

    void USARTClass::IRQHandler()
    {
#if defined( STM32F7 )

#endif

#if defined( STM32F4 )
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
              memset( rxInternalBuffer, 0, rxInternalBufferSize );
            }

            /*------------------------------------------------
            Buffer the received byte to our internal buffer
            ------------------------------------------------*/
            if ( rxMode == Modes::INTERRUPT && ( asyncRXDataSize < rxInternalBufferSize ) )
            {
              rxInternalBuffer[ asyncRXDataSize ] = static_cast<uint8_t>( data_reg );
              asyncRXDataSize += 1u;
            }
            else if ( rxMode != Modes::DMA )
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
          if ( rxMode == Modes::INTERRUPT )
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
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      USART_EnableClock();

      if ( HAL_USART_Init( &usart_handle ) != HAL_OK )
      {
        error = Chimera::CommonStatusCodes::FAIL;
      }

      setMode( SubPeripheral::TX, Modes::BLOCKING );
      setMode( SubPeripheral::RX, Modes::BLOCKING );

      PeripheralState.enabled = true;
      return error;
    }

    void USARTClass::USART_DeInit()
    {
      HAL_USART_DeInit( &usart_handle );
      PeripheralState.enabled = false;
    }

    void USARTClass::USART_EnableClock()
    {
      *getUsartClockReg( usart_handle.Instance ) |= ( usartClockMask( usart_handle.Instance ) );
    }

    void USARTClass::USART_DisableClock()
    {
      *getUsartClockReg( usart_handle.Instance ) &= ~( usartClockMask( usart_handle.Instance ) );
    }

    void USARTClass::USART_DMA_EnableClock()
    {
      /* Global DMA Clock options. Only turn on capability is
      provided due to other peripherals possibly using DMA. */
      if ( __DMA1_IS_CLK_DISABLED() )
      {
        __DMA1_CLK_ENABLE();
      }

      if ( __DMA2_IS_CLK_DISABLED() )
      {
        __DMA2_CLK_ENABLE();
      }
    }

    void USARTClass::USART_EnableInterrupts()
    {
      HAL_NVIC_DisableIRQ( ITSettings_HW.IRQn );
      HAL_NVIC_ClearPendingIRQ( ITSettings_HW.IRQn );
      HAL_NVIC_SetPriority( ITSettings_HW.IRQn, ITSettings_HW.preemptPriority, ITSettings_HW.subPriority );
      HAL_NVIC_EnableIRQ( ITSettings_HW.IRQn );

      /*------------------------------------------------
Make sure we are able to asynchronously receive some data if it gets sent
------------------------------------------------*/
      AUTO_ASYNC_RX = true;
      if ( rxMode == Modes::INTERRUPT )
      {
        /*------------------------------------------------
        In interrupt mode, we have to handle the RX FIFO on a byte by byte
        basis otherwise an overrun error is generated.
        ------------------------------------------------*/
        __HAL_USART_ENABLE_IT( &usart_handle, USART_IT_RXNE );
      }

      PeripheralState.interrupts_enabled = true;
    }

    void USARTClass::USART_DisableInterrupts()
    {
      __HAL_USART_DISABLE_IT( &usart_handle, USART_IT_IDLE );
      __HAL_USART_DISABLE_IT( &usart_handle, USART_IT_RXNE );

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

    void USARTClass::USART_DMA_Init( const SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
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

    void USARTClass::USART_DMA_DeInit( const SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
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

    void USARTClass::USART_DMA_EnableIT( const SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
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

    void USARTClass::USART_DMA_DisableIT( const SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
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
  }    // namespace USART
}    // namespace Thor

#if !defined( GMOCK_TEST )
void HAL_USART_TxCpltCallback( USART_HandleTypeDef *UsartHandle )
{
  const USARTClass_sPtr &usart = getUSARTClassRef( UsartHandle->Instance );

  if ( usart && usart->PeripheralState.enabled )
  {
    usart->tx_complete = true;

    /*------------------------------------------------
    Transmit more data if we have any
    ------------------------------------------------*/
    if ( !usart->txUserBuffer->empty() )
    {
      size_t bytesToWrite = std::min( usart->txUserBuffer->size(), usart->txInternalBufferSize );
      memset( usart->txInternalBuffer, 0, usart->txInternalBufferSize );

      for ( uint32_t x = 0; x < bytesToWrite; x++ )
      {
        usart->txInternalBuffer[ x ] = usart->txUserBuffer->front();
        usart->txUserBuffer->pop_front();
      }

      usart->write( usart->txInternalBuffer, bytesToWrite );
    }
  }
}

void HAL_USART_RxCpltCallback( USART_HandleTypeDef *UsartHandle )
{
  const USARTClass_sPtr &usart = getUSARTClassRef( UsartHandle->Instance );

  if ( usart )
  {
    if ( usart->rxMode == Modes::INTERRUPT )
    {
      if ( usart->AUTO_ASYNC_RX )
      {
        /*------------------------------------------------
        We unexpectedly got some data, so dump it into the user's buffer for them
        to read later. Don't need to check if the buffer is valid as:

        1. ISR routine. We want AFAP.
        2. The buffer's existence is a requirement for this ISR to even execute.
        ------------------------------------------------*/
        for ( size_t x = 0; x < usart->asyncRXDataSize; x++ )
        {
          usart->rxUserBuffer->push_back( usart->rxInternalBuffer[ x ] );
        }
        usart->asyncRXDataSize = 0;
      }
      else
      {
        /*------------------------------------------------
        The user explicitly requested for data to be read
        ------------------------------------------------*/
        uint16_t actuallyRead = UsartHandle->RxXferSize - UsartHandle->RxXferCount;
        for ( uint16_t x = 0; x < actuallyRead; x++ )
        {
          usart->rxUserBuffer->push_back( usart->rxInternalBuffer[ x ] );
        }
      }

      /*------------------------------------------------
      Go back to async RX listening
      ------------------------------------------------*/
      __HAL_USART_ENABLE_IT( UsartHandle, USART_IT_RXNE );
    }
    else if ( usart->rxMode == Modes::DMA )
    {
      /*------------------------------------------------
      Calculate how many bytes were received by looking at remaining RX buffer space.
                  num_received = bufferMaxSize - bufferSizeRemaining
      ------------------------------------------------*/
      auto num_received = static_cast<uint32_t>( UsartHandle->RxXferSize - UsartHandle->hdmarx->Instance->NDTR );

      /*------------------------------------------------
      Copy out the data
      ------------------------------------------------*/
      for ( size_t x = 0; x < num_received; x++ )
      {
        usart->rxUserBuffer->push_back( usart->rxInternalBuffer[ x ] );
      }

      /*------------------------------------------------
      Go back to async RX listening
      ------------------------------------------------*/
      memset( usart->rxInternalBuffer, 0, usart->rxInternalBufferSize );
      Thor::USART::USART_EnableIT_IDLE( UsartHandle );
      HAL_USART_Receive_DMA( UsartHandle, usart->rxInternalBuffer, usart->rxInternalBufferSize );
    }

    /*------------------------------------------------
    Exit the ISR ensuring that we can still receive asynchronous data
    ------------------------------------------------*/
    usart->rx_complete   = true;
    usart->AUTO_ASYNC_RX = true;
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
