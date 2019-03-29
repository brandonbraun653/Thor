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

/* Thor Includes */
#include <Thor/uart.hpp>
#include <Thor/exceptions.hpp>
#include <Thor/interrupt.hpp>

using namespace Thor;
using namespace Thor::Serial;
using namespace Thor::UART;
using namespace Thor::Interrupt;
using namespace Thor::Defaults::Serial;


#if defined( USING_FREERTOS )
static SemaphoreHandle_t uartSemphrs[ MAX_SERIAL_CHANNELS + 1 ];
TaskTrigger uartTaskTrigger;
#endif

static Thor::UART::UARTClass_sPtr uartObjects[ MAX_SERIAL_CHANNELS + 1 ];

static const UARTClass_sPtr &getUARTClassRef( USART_TypeDef *instance )
{
  /* Simply converts the pointer into the raw numerical address value, which be compared against
     the peripheral base address. UARTx is simply (USART_TypeDef*)UARTx_Base. */
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

static uint32_t uartClockMask( USART_TypeDef *instance )
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

namespace Thor
{
  namespace UART
  {
    using namespace Thor::GPIO;
    using namespace Chimera::Serial;

    inline void UART_ClearIT_IDLE( UART_HandleTypeDef *UartHandle )
    {
#if defined( STM32F7 )
      __HAL_UART_CLEAR_IT( UartHandle, UART_CLEAR_IDLEF );
#endif
    }

    inline void UART_EnableIT_IDLE( UART_HandleTypeDef *UartHandle )
    {
      UART_ClearIT_IDLE( UartHandle );
      __HAL_UART_ENABLE_IT( UartHandle, UART_IT_IDLE );
    }

    inline void UART_DisableIT_IDLE( UART_HandleTypeDef *UartHandle )
    {
      UART_ClearIT_IDLE( UartHandle );
      __HAL_UART_DISABLE_IT( UartHandle, UART_IT_IDLE );
    }

    UARTClass::~UARTClass()
    {
      delete[] rxInternalBuffer;
      delete[] txInternalBuffer;
    }

    UARTClass_sPtr UARTClass::create( const uint8_t channel, const size_t bufferSize )
    {
      UARTClass_sPtr newClass( new UARTClass() );

      if ( channel <= ( MAX_SERIAL_CHANNELS + 1 ) )
      {
        uartObjects[ channel ] = newClass;
        newClass->assignRXBuffer( new uint8_t[ bufferSize ](), bufferSize );
        newClass->assignTXBuffer( new uint8_t[ bufferSize ](), bufferSize );
      }
      else
      {
        newClass = nullptr;
      }

      return newClass;
    }

    Chimera::Status_t UARTClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
    {
      asyncRXDataSize = 0;

      /*------------------------------------------------
      Assign the default handle settings
      ------------------------------------------------*/
      uart_channel         = channel;
      uart_handle.Init     = Defaults::Serial::dflt_UART_Init;
      uart_handle.Instance = const_cast<USART_TypeDef *>( hwConfig[ uart_channel ]->instance );

#if defined( STM32F7 )
      uart_handle.AdvancedInit = Defaults::Serial::dflt_UART_AdvInit;
#endif

      /*------------------------------------------------
      Copy over the interrupt settings information
      ------------------------------------------------*/
      ITSettings_HW     = hwConfig[ uart_channel ]->IT_HW;
      ITSettings_DMA_TX = hwConfig[ uart_channel ]->dmaIT_TX;
      ITSettings_DMA_RX = hwConfig[ uart_channel ]->dmaIT_RX;

      /*------------------------------------------------
      Initialize the GPIO pins
      ------------------------------------------------*/
      tx_pin = std::make_shared<Thor::GPIO::GPIOClass>();
      tx_pin->initAdvanced( Thor::GPIO::convertPort( pins.tx.port ), Thor::GPIO::convertPinNum( pins.tx.pin ),
                            PinSpeed::ULTRA_SPD, pins.tx.alternate );

      rx_pin = std::make_shared<Thor::GPIO::GPIOClass>();
      rx_pin->initAdvanced( Thor::GPIO::convertPort( pins.rx.port ), Thor::GPIO::convertPinNum( pins.rx.pin ),
                            PinSpeed::ULTRA_SPD, pins.rx.alternate );

      UART_GPIO_Init();

#if defined( USING_FREERTOS )
      uartSemphrs[ uart_channel ] = xSemaphoreCreateCounting( UART_QUEUE_SIZE, UART_QUEUE_SIZE );
#endif

      hardware_assigned = true;
      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t UARTClass::begin( const Modes txMode, const Modes rxMode )
    {
      setMode( SubPeripheral::TX, txMode );
      setMode( SubPeripheral::RX, rxMode );

      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t UARTClass::end()
    {
      UART_DeInit();
      UART_GPIO_DeInit();
      UART_DisableInterrupts();
      UART_DMA_DeInit( SubPeripheral::TX );
      UART_DMA_DeInit( SubPeripheral::RX );

      txMode = Modes::MODE_UNDEFINED;
      rxMode = Modes::MODE_UNDEFINED;

      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t UARTClass::configure( const uint32_t baud, const Chimera::Serial::CharWid width,
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

        UART_InitTypeDef init = uart_handle.Init;

        init.BaudRate = baud;
        result |= setWordLength( init, width );
        result |= setParity( init, parity );
        result |= setStopBits( init, stop );
        result |= setFlowControl( init, flow );

        if( result )
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

      if ( !hardware_assigned )
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

    Chimera::Status_t UARTClass::setMode( const SubPeripheral periph, const Modes mode )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( !hardware_assigned )
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }
      else
      {
        if ( periph == SubPeripheral::TX )
        {
          switch ( mode )
          {
            case Modes::BLOCKING:
              txMode = mode;    // Must be set before the other functions

              /* Make sure RX side isn't using interrupts before disabling */
              if ( rxMode == Modes::BLOCKING )
              {
                UART_DisableInterrupts();
              }

              UART_DMA_DeInit( periph );
              break;

            case Modes::INTERRUPT:
              txMode = mode;    // Must be set before the other functions

              UART_EnableInterrupts();
              UART_DMA_DeInit( periph );
              break;

            case Modes::DMA:
              txMode = mode;    // Must be set before the other functions

              UART_EnableInterrupts();
              UART_DMA_Init( periph );
              break;

            default:
              txMode = Modes::MODE_UNDEFINED;
              error  = Chimera::CommonStatusCodes::FAIL;
              break;
          }
        }
        else if ( periph == SubPeripheral::RX )
        {
          switch ( mode )
          {
            case Modes::BLOCKING:
              rxMode = mode;    // Must be set before the other functions

              /* Make sure TX side isn't using interrupts before disabling */
              if ( txMode == Modes::BLOCKING )
              {
                UART_DisableInterrupts();
              }

              UART_DMA_DeInit( periph );

              break;

            case Modes::INTERRUPT:
              rxMode = mode;    // Must be set before the other functions

              UART_EnableInterrupts();
              UART_DMA_DeInit( periph );
              break;

            case Modes::DMA:
              rxMode = mode;    // Must be set before the other functions

              UART_EnableInterrupts();
              UART_DMA_Init( periph );

              /* Set the idle line interrupt for asynchronously getting the end of transmission */
              UART_EnableIT_IDLE( &uart_handle );

              /* Instruct the DMA hardware to start listening for transmissions */
              HAL_UART_Receive_DMA(&uart_handle, rxInternalBuffer, rxInternalBufferSize );
              break;

            default:
              rxMode = Modes::MODE_UNDEFINED;
              error  = Chimera::CommonStatusCodes::FAIL;
              break;
          }
        }
        else
        {
          error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
        }
      }

      return error;
    }

    Chimera::Status_t UARTClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( !PeripheralState.gpio_enabled || !PeripheralState.enabled )
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }
      else
      {
        HAL_StatusTypeDef stm32Error = HAL_OK;

        /*------------------------------------------------
        Shame on me and my cow for const casting. What am I to do?
        Differing interface specs are no good...
        ------------------------------------------------*/
        switch ( txMode )
        {
          case Modes::BLOCKING:
#if defined( USING_FREERTOS )
            stm32Error = HAL_UART_Transmit( &uart_handle, const_cast<uint8_t *>( buffer ), length,
                                            pdMS_TO_TICKS( BLOCKING_TIMEOUT_MS ) );
#else
            stm32Error = HAL_UART_Transmit( &uart_handle, const_cast<uint8_t *>( buffer ), length, BLOCKING_TIMEOUT_MS );
#endif
            error = convertHALStatus( stm32Error );
            break;

          case Modes::INTERRUPT:
            if ( txUserBuffer && PeripheralState.tx_buffering_enabled )
            {
              if ( tx_complete )
              {
                /*------------------------------------------------
                Hardware is free. Send the data directly.
                ------------------------------------------------*/
                tx_complete = false;
                stm32Error  = HAL_UART_Transmit_IT( &uart_handle, const_cast<uint8_t *>( buffer ), length );
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

            break;

          case Modes::DMA:
            if ( txUserBuffer && PeripheralState.tx_buffering_enabled )
            {
              if ( tx_complete )
              {
                /*------------------------------------------------
                Hardware is free. Send the data directly.
                ------------------------------------------------*/
                tx_complete = false;
                stm32Error  = HAL_UART_Transmit_DMA( &uart_handle, const_cast<uint8_t *>( buffer ), length );
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
            break;

          default:
            error = Chimera::CommonStatusCodes::UNKNOWN_ERROR;
            break;
        }
      }

      return error;
    }

    Chimera::Status_t UARTClass::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( !PeripheralState.gpio_enabled || !PeripheralState.enabled )
      {
        error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
      }
      else
      {
        HAL_StatusTypeDef stm32Error = HAL_OK;

        switch ( rxMode )
        {
          case Modes::BLOCKING:
            /* It's possible to get into the condition where ORE is set before trying to receive some
             * new data. In the current STM HAL library, all error interrupts for the blocking mode are
             * disabled by default so the overrun has to be handled manually. This restores normal
             * operation. A nearly exact condition of this bug is encountered here: https://goo.gl/bKi8Ps
             **/
            UART_OverrunHandler();

#if defined( USING_FREERTOS )
            stm32Error =
                HAL_UART_Receive( &uart_handle, const_cast<uint8_t *>( buffer ), length, pdMS_TO_TICKS( BLOCKING_TIMEOUT_MS ) );
#else
            stm32Error = HAL_UART_Receive( &uart_handle, const_cast<uint8_t *>( buffer ), length, BLOCKING_TIMEOUT_MS );
#endif
            error = convertHALStatus( stm32Error );
            break;

          case Modes::INTERRUPT:
            /*------------------------------------------------
            Let the ISR handler know that we explicitely asked to receive some data.
            This will cause a redirect into the correct handling channels.
            ------------------------------------------------*/
            RX_ASYNC_EXPLICIT = false;

            stm32Error = HAL_UART_Receive_IT( &uart_handle, const_cast<uint8_t *>( buffer ), length );

            if ( stm32Error == HAL_OK )
            {
              error = Chimera::Serial::Status::RX_IN_PROGRESS;
            }
            else
            {
              error = convertHALStatus( stm32Error );
            }
            break;

          case Modes::DMA:
            /*------------------------------------------------
            Let the ISR handler know that we explicitely asked to receive some data.
            This will cause a redirect into the correct handling channels.
            ------------------------------------------------*/
            RX_ASYNC_EXPLICIT = false;

            /*------------------------------------------------
            Stop any background listening for data and restart with the new array
            ------------------------------------------------*/
            HAL_UART_DMAStop( &uart_handle );
            stm32Error = HAL_UART_Receive_DMA( &uart_handle, const_cast<uint8_t *>( buffer ), length );

            if ( stm32Error == HAL_OK )
            {
              error = Chimera::Serial::Status::RX_IN_PROGRESS;
            }
            else
            {
              error = convertHALStatus( stm32Error );
            }
            break;

          default:
            break;
        }
      }

      return error;
    }

    Chimera::Status_t UARTClass::flush( const Chimera::Serial::SubPeripheral periph )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if( periph == SubPeripheral::TX )
      {
        if( txInternalBuffer )
        {
          memset( txInternalBuffer, 0, txInternalBufferSize );
        }

        if( txUserBuffer )
        {
          txUserBuffer->clear();
        }
      }
      else if( periph == SubPeripheral::RX )
      {
        if( rxInternalBuffer )
        {
          memset( rxInternalBuffer, 0, rxInternalBufferSize );
        }

        if( rxUserBuffer )
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

    Chimera::Status_t UARTClass::readAsync( uint8_t *const buffer, const size_t len )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if( !buffer )
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
      else
      {
        size_t bytesRead = 0;

        while( !rxUserBuffer->empty() && ( bytesRead < len ) )
        {
          buffer[ bytesRead ] = rxUserBuffer->front();
          rxUserBuffer->pop_front();
          bytesRead++;
        }

        if( bytesRead != len )
        {
          error = Chimera::CommonStatusCodes::EMPTY;
        }
      }

      return error;
    }

#if defined( USING_FREERTOS )
    Chimera::Status_t UARTClass::attachEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    Chimera::Status_t UARTClass::removeEventNotifier( const Chimera::Serial::Event event, SemaphoreHandle_t *const semphr )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }
#endif

    Chimera::Status_t UARTClass::enableBuffering( const Chimera::Serial::SubPeripheral periph,
                                         boost::circular_buffer<uint8_t> *const buffer )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      if ( periph == SubPeripheral::TX )
      {
        txUserBuffer                          = buffer;
        PeripheralState.tx_buffering_enabled = true;
      }
      else if ( periph == SubPeripheral::RX )
      {
        rxUserBuffer                          = buffer;
        PeripheralState.rx_buffering_enabled = true;
      }
      else
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }

      return error;
    }

    Chimera::Status_t UARTClass::disableBuffering( const Chimera::Serial::SubPeripheral periph )
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

    void UARTClass::assignRXBuffer( uint8_t *const buffer, const size_t size )
    {
      rxInternalBuffer = buffer;
      rxInternalBufferSize = size;
    }

    void UARTClass::assignTXBuffer( uint8_t *const buffer, const size_t size )
    {
      txInternalBuffer = buffer;
      txInternalBufferSize = size;
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
      switch( static_cast<uint8_t>( parity ) )
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

      switch( static_cast<uint8_t>( stopBits ) )
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
      bool result = true;

      switch( static_cast<uint8_t>( flow ))
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

    void UARTClass::IRQHandler()
    {
      /** NOTE: The way some of these interrupts are handled may seem weird, but it's to protect against
       *	instances where the ISR is randomly triggered and there really shouldn't be any processing going on.
       *	While a few sources of these errant events have been tracked down, not all are found. Thus, it is
       *	easier to structure the code in such a way that it only runs exactly when certain conditions are met.
       **/

#if defined( STM32F7 )
      bool RX_DATA_READY   = __HAL_UART_GET_FLAG( &uart_handle, UART_FLAG_RXNE );
      bool RX_LINE_IDLE    = __HAL_UART_GET_FLAG( &uart_handle, UART_FLAG_IDLE );
      bool RX_LINE_IDLE_EN = __HAL_UART_GET_IT_SOURCE( &uart_handle, UART_IT_IDLE );

      /*------------------------------------
       * Handle Asynchronous RX (Interrupt and DMA Mode)
       *------------------------------------*/
      /* RX In Progress */
      if ( RX_ASYNC && RX_DATA_READY && uart_handle.gState != HAL_UART_STATE_BUSY_TX )
      {
        uint32_t isrflags   = READ_REG( uart_handle.Instance->ISR );
        uint32_t errorflags = ( isrflags & ( uint32_t )( USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE ) );

        if ( errorflags == RESET )
        {
          /* Detected new RX of unknown size */
          if ( asyncRXDataSize == 0 )
          {
            memset( RX_Queue[ RXQueueIdx ], 0, UART_QUEUE_BUFFER_SIZE );

            /* Enable UART_IT_IDLE to detect transmission end.
             * Sometimes IDLEF is set before interrupt enable, which will immediately trigger this ISR,
             * and cause reception of only 1 character. Clear first to ensure accurate idle line trigger. */
            __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_IDLEF );
            __HAL_UART_ENABLE_IT( &uart_handle, UART_IT_IDLE );
          }

          /* Buffer the new data */
          if ( rxMode == Modes::INTERRUPT && ( asyncRXDataSize < UART_QUEUE_BUFFER_SIZE ) )
          {
            RX_Queue[ RXQueueIdx ][ asyncRXDataSize ] = ( uint8_t )( uart_handle.Instance->RDR & ( uint8_t )0xFF );
            asyncRXDataSize += 1u;
          }
          else
          {
            /* Forced to read data without being able to store it.
             * Really need to put some kind of error thing here...*/
            uart_handle.Instance->RDR;
          }
        }
        else
        {
          // Do something more useful later
          __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_PEF );
          __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_FEF );
          __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_NEF );
          __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_OREF );
        }
      }

      /* RX Complete */
      if ( RX_ASYNC && RX_LINE_IDLE_EN && RX_LINE_IDLE )
      {
        if ( rxMode == Modes::INTERRUPT )
        {
          UART_DisableIT_IDLE( &uart_handle );

          /* Copy data received to the internal buffer */
          RX_tempPacket.data_ptr = RX_Queue[ RXQueueIdx ];
          RX_tempPacket.length   = asyncRXDataSize;
          RXPacketBuffer.push_back( RX_tempPacket );

          /* Clean up the class variables to prepare for a new reception */
          rx_complete     = true;
          asyncRXDataSize = 0;
          totalUnreadPackets++;
          _rxIncrQueueIdx();

          /* Finally, call this here because the normal HAL_UART_IRQHandler does not get called
           * due to the asynchronous nature of operation. */
          HAL_UART_RxCpltCallback( &uart_handle );
        }
        else if ( rxMode == Modes::DMA )
        {
          auto num_received = ( size_t )( uart_handle.RxXferSize - uart_handle.hdmarx->Instance->NDTR );

          if ( num_received != 0 )
          {
            UART_DisableIT_IDLE( &uart_handle );
            rx_complete = true;
            totalUnreadPackets++;

            /* Force DMA hard reset to trigger the DMA RX Complete handler */
            __HAL_DMA_DISABLE( uart_handle.hdmarx );
          }
          else
          {
            /* ISR was randomly triggered. Clear any errant flags that might be enabled.*/
            UART_ClearIT_IDLE( &uart_handle );
          }
        }
      }

      /*------------------------------------
       * Handle Synchronous RX or Asynchronous TX (Interrupt Mode)
       *------------------------------------*/
      if ( !RX_ASYNC || uart_handle.gState == HAL_UART_STATE_BUSY_TX )
      {
        HAL_UART_IRQHandler( &uart_handle );
      }
#endif /* STM32F7 */

#if defined( STM32F4 )
      /** Reading these two in the order of SR then DR ends up clearing all flags, so it's best to store the returned
       *	contents for further processing. See uart clearing procedure in device datasheet in the Registers section.
       **/
      volatile uint32_t isrflags = READ_REG( uart_handle.Instance->SR );
      volatile uint32_t data_reg = READ_REG( uart_handle.Instance->DR );

      /*------------------------------------------------
      Handle Asynchronous RX In Interrupt and DMA Mode:
      The user did not ask for data, but we got some anyways (cause we were listening)
      ------------------------------------------------*/
      if ( RX_ASYNC_EXPLICIT )
      {
        volatile uint32_t cr1 = READ_REG( uart_handle.Instance->CR1 );

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
            else if( rxMode != Modes::DMA )
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
          size_t num_received = static_cast<size_t>( uart_handle.RxXferSize - uart_handle.hdmarx->Instance->NDTR );

          if ( rxMode == Modes::INTERRUPT )
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
          else if ( ( rxMode == Modes::DMA ) && num_received )
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
      Handle Synchronous RX or Asynchronous TX (Interrupt Mode Only):
      The user explicitly asked for us to read some data. Allow the normal handlers to run.
      ------------------------------------------------*/
      if ( !RX_ASYNC_EXPLICIT || uart_handle.gState == HAL_UART_STATE_BUSY_TX )
      {
        HAL_UART_IRQHandler( &uart_handle );
      }
#endif /* STM32F4 */
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

      setMode( SubPeripheral::TX, Modes::BLOCKING );
      setMode( SubPeripheral::RX, Modes::BLOCKING );

      PeripheralState.enabled = true;
      return error;
    }

    void UARTClass::UART_DeInit()
    {
      HAL_UART_DeInit( &uart_handle );
      PeripheralState.enabled = false;
    }

    void UARTClass::UART_EnableClock()
    {
      using namespace Thor::Serial;

#if defined( TARGET_STM32F7 ) || defined( TARGET_STM32F4 )
      RCC->APB1ENR |= ( uartClockMask( uart_handle.Instance ) );
#endif
    }

    void UARTClass::UART_DisableClock()
    {
      using namespace Thor::Serial;

#if defined( TARGET_STM32F7 ) || defined( TARGET_STM32F4 )
      RCC->APB1ENR &= ~( uartClockMask( uart_handle.Instance ) );
#endif
    }

    void UARTClass::UART_DMA_EnableClock()
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

    void UARTClass::UART_EnableInterrupts()
    {
      HAL_NVIC_DisableIRQ( ITSettings_HW.IRQn );
      HAL_NVIC_ClearPendingIRQ( ITSettings_HW.IRQn );
      HAL_NVIC_SetPriority( ITSettings_HW.IRQn, ITSettings_HW.preemptPriority, ITSettings_HW.subPriority );
      HAL_NVIC_EnableIRQ( ITSettings_HW.IRQn );

      /* Specific interrupts to enable */
      if ( rxMode == Modes::INTERRUPT )
      {
        RX_ASYNC_EXPLICIT = true;
        __HAL_UART_ENABLE_IT( &uart_handle, UART_IT_RXNE );    // RX Data Register not Empty
      }

      PeripheralState.interrupts_enabled = true;
    }

    void UARTClass::UART_DisableInterrupts()
    {
      __HAL_UART_DISABLE_IT( &uart_handle, UART_IT_IDLE );
      __HAL_UART_DISABLE_IT( &uart_handle, UART_IT_RXNE );

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

    void UARTClass::UART_DMA_Init( const SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
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

        serialDMAManager.attachCallback_TXDMA( uart_channel, boost::bind( &UARTClass::IRQHandler_TXDMA, this ) );

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

        serialDMAManager.attachCallback_RXDMA( uart_channel, boost::bind( &UARTClass::IRQHandler_RXDMA, this ) );

        UART_DMA_EnableIT( periph );

        PeripheralState.dma_enabled_rx = true;
      }
    }

    void UARTClass::UART_DMA_DeInit( const SubPeripheral periph )
    {
      if ( periph == SubPeripheral::TX )
      {
        if ( !PeripheralState.dma_enabled_tx )
          return;

        HAL_DMA_Abort( uart_handle.hdmatx );
        HAL_DMA_DeInit( uart_handle.hdmatx );
        UART_DMA_DisableIT( periph );
        serialDMAManager.removeCallback_TXDMA( uart_channel );

        PeripheralState.dma_enabled_tx = false;
      }
      else
      {
        if ( !PeripheralState.dma_enabled_rx )
          return;

        HAL_DMA_Abort( uart_handle.hdmarx );
        HAL_DMA_DeInit( uart_handle.hdmarx );
        UART_DMA_DisableIT( periph );
        serialDMAManager.removeCallback_RXDMA( uart_channel );

        PeripheralState.dma_enabled_rx = false;
      }
    }

    void UARTClass::UART_DMA_EnableIT( const SubPeripheral periph )
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

    void UARTClass::UART_DMA_DisableIT( const SubPeripheral periph )
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

    void UARTClass::UART_OverrunHandler()
    {
#if defined( STM32F7 )
      __HAL_UART_CLEAR_IT( &uart_handle, UART_CLEAR_OREF );
      uart_handle.Instance->RDR;
#endif
    }

  }    // namespace UART
}    // namespace Thor

#if !defined( GMOCK_TEST )
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *UartHandle )
{
  const UARTClass_sPtr& uart = getUARTClassRef(UartHandle->Instance);

  if ( uart && uart->PeripheralState.enabled )
  {
    uart->tx_complete = true;

    /*------------------------------------------------
    Transmit more data if we have any
    ------------------------------------------------*/
    if (!uart->txUserBuffer->empty())
    {
      size_t bytesToWrite = std::min( uart->txUserBuffer->size(), uart->txInternalBufferSize );
      memset( uart->txInternalBuffer, 0, uart->txInternalBufferSize );

      for( uint32_t x=0; x<bytesToWrite; x++ )
      {
        uart->txInternalBuffer[x] = uart->txUserBuffer->front();
        uart->txUserBuffer->pop_front();
      }

      uart->write( uart->txInternalBuffer, bytesToWrite );
    }
  }
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *UartHandle )
{
  const UARTClass_sPtr& uart = getUARTClassRef(UartHandle->Instance);

  if ( uart && uart->rxMode == Modes::DMA )
  {
    uart->rx_complete = true;

    /*------------------------------------------------
    Calculate how many bytes were received by looking at remaining RX buffer space.
                num_received = bufferMaxSize - bufferSizeRemaining
    ------------------------------------------------*/
    size_t num_received = static_cast<size_t>( UartHandle->RxXferSize - UartHandle->hdmarx->Instance->NDTR );

    /*------------------------------------------------
    Copy out the data depending on which type of transfer was used
    ------------------------------------------------*/
    if( uart->RX_ASYNC_EXPLICIT )
    {
      for( size_t x=0; x<num_received; x++ )
      {
        uart->rxUserBuffer->push_back( UartHandle->pRxBuffPtr[x] );
      }
    }
    else
    {
      for( size_t x=0; x<num_received; x++ )
      {
        uart->rxUserBuffer->push_back( uart->rxInternalBuffer[x] );
      }
    }

    /*------------------------------------------------
    Start the listening process again for a new packet
    ------------------------------------------------*/
    Thor::UART::UART_EnableIT_IDLE(UartHandle);
    HAL_UART_Receive_DMA( UartHandle, uart->rxInternalBuffer, uart->rxInternalBufferSize );
  }
  else if( uart && !uart->rx_complete )
  {
    uart->rx_complete = true;

    if ( uart->rxMode == Modes::INTERRUPT)
    {
      uart->RX_ASYNC_EXPLICIT = false;
    }
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

void UART1_IRQHandler( void )
{
  if ( uartObjects[ 1 ] )
  {
    uartObjects[ 1 ]->IRQHandler();
  }
}

void UART2_IRQHandler( void )
{
  if ( uartObjects[ 2 ] )
  {
    uartObjects[ 2 ]->IRQHandler();
  }
}

void UART3_IRQHandler( void )
{
  if ( uartObjects[ 3 ] )
  {
    uartObjects[ 3 ]->IRQHandler();
  }
}

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

void UART6_IRQHandler( void )
{
  if ( uartObjects[ 6 ] )
  {
    uartObjects[ 6 ]->IRQHandler();
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
