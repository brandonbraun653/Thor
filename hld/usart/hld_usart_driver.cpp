/********************************************************************************
 *  File Name:
 *    thor_custom_usart.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor USART interface.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstdint>
#include <memory>

/* Boost Includes */
#include <boost/circular_buffer.hpp>

/* Aurora Includes */
#include <Aurora/constants>

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/event>
#include <Chimera/interrupt>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/event>
#include <Thor/usart>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_detail.hpp>
#include <Thor/lld/interface/usart/usart_prv_data.hpp>

#if defined( THOR_HLD_USART )

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::USART;
namespace LLD = ::Thor::LLD::USART;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = LLD::NUM_USART_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static size_t s_driver_initialized;                        /**< Tracks the module level initialization state */
static HLD::Driver hld_driver[ NUM_DRIVERS ];              /**< Driver objects */

/*-------------------------------------------------------------------------------
Private Function Declarations
-------------------------------------------------------------------------------*/
static void USARTxISRUserThread( void *arg )
{
  using namespace Chimera::Thread;

  while ( 1 )
  {
    /*-------------------------------------------------
    Wait for something to wake this thread. If the msg
    isn't correct, go back to waiting.
    -------------------------------------------------*/
    if( !this_thread::pendTaskMsg( ITCMsg::TSK_MSG_ISR_HANDLER ) )
    {
      continue;
    }

    /*-------------------------------------------------
    Handle every ISR. Don't know which triggered this.
    -------------------------------------------------*/
    for( size_t index = 0; index < NUM_DRIVERS; index++ )
    {
      hld_driver[ index ].postISRProcessing();
    }
  }
}


namespace Thor::USART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    using namespace Chimera::Thread;

    /*------------------------------------------------
    Prevent multiple initializations (need reset first)
    ------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*------------------------------------------------
    Register the ISR post processor thread
    ------------------------------------------------*/
    Task userThread;
    TaskConfig cfg;

    cfg.arg        = nullptr;
    cfg.function   = USARTxISRUserThread;
    cfg.priority   = Priority::MAXIMUM;
    cfg.stackWords = STACK_BYTES( 512 );
    cfg.type       = TaskInitType::DYNAMIC;
    cfg.name       = "PP_USARTx";

    userThread.create( cfg );
    LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_USART, userThread.start() );

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    ::LLD::initialize();

    /*-------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  Chimera::Status_t reset()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  bool isChannelUSART( const Chimera::Serial::Channel channel )
  {
    return ::LLD::isSupported( channel );
  }


  Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    if ( auto idx = ::LLD::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return &hld_driver[ idx ];
    }
    else
    {
      RT_HARD_ASSERT( false );
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() :
      enabled( false ), channel( Chimera::Serial::Channel::NOT_SUPPORTED ), resourceIndex( 0 ), awaitRXComplete( 1 ),
      awaitTXComplete( 1 ), rxLock( 1 ), txLock( 1 )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::assignHW( const Chimera::Serial::Channel channel, const Chimera::Serial::IOPins &pins )
  {
    /*------------------------------------------------
    Make sure the channel is actually supported
    ------------------------------------------------*/
    this->channel = channel;
    resourceIndex = ::LLD::getResourceIndex( channel );

    /*------------------------------------------------
    Create the hardware drivers
    ------------------------------------------------*/
    txPin = Chimera::GPIO::getDriver( pins.tx.port, pins.tx.pin );
    rxPin = Chimera::GPIO::getDriver( pins.rx.port, pins.rx.pin );

    /*------------------------------------------------
    Initialize/Configure hardware drivers
    ------------------------------------------------*/
    txPin->init( pins.tx );
    rxPin->init( pins.rx );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::begin( const Chimera::Hardware::PeripheralMode txMode,
                                   const Chimera::Hardware::PeripheralMode rxMode )
  {
    /*------------------------------------------------
    Ensure the internal event signals are reset
    ------------------------------------------------*/
    awaitRXComplete.try_acquire();
    awaitTXComplete.try_acquire();

    /*------------------------------------------------
    Initialize to the desired TX/RX modes
    ------------------------------------------------*/
    setMode( Chimera::Hardware::SubPeripheral::RX, rxMode );
    setMode( Chimera::Hardware::SubPeripheral::TX, txMode );

    enabled = true;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::end()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::configure( const Chimera::Serial::Config &config )
  {
    Thor::LLD::Serial::Config cfg = ::LLD::getDriver( channel )->getConfiguration();

    /*------------------------------------------------
    Convert between the generalize Chimera options into
    MCU specific register configuration settings. If these lookup
    tables fail, you might have something wrong with the mappings.
    ------------------------------------------------*/
    cfg.BaudRate   = static_cast<uint32_t>( config.baud );
    cfg.Mode       = ::LLD::Configuration::Modes::TX_RX;
    cfg.Parity     = ::LLD::ConfigMap::Parity[ static_cast<size_t>( config.parity ) ];
    cfg.StopBits   = ::LLD::ConfigMap::StopBits[ static_cast<size_t>( config.stopBits ) ];
    cfg.WordLength = ::LLD::ConfigMap::CharWidth[ static_cast<size_t>( config.width ) ];

    return ::LLD::getDriver( channel )->init( cfg );
  }


  Chimera::Status_t Driver::setBaud( const uint32_t baud )
  {
    auto currentConfig = ::LLD::getDriver( channel )->getConfiguration();
    currentConfig.BaudRate = baud;

    return ::LLD::getDriver( channel )->init( currentConfig );
  }


  Chimera::Status_t Driver::setMode( const Chimera::Hardware::SubPeripheral periph,
                                         const Chimera::Hardware::PeripheralMode mode )
  {
    using namespace Chimera::Hardware;

    if ( ( periph == SubPeripheral::RX ) || ( periph == SubPeripheral::TXRX ) )
    {
      rxMode = mode;
    }

    if ( ( periph == SubPeripheral::TX ) || ( periph == SubPeripheral::TXRX ) )
    {
      txMode = mode;
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::write( const void *const buffer, const size_t length  )
  {
    auto error = Chimera::Status::OK;

    switch( txMode )
    {
      case Chimera::Hardware::PeripheralMode::BLOCKING:
        error = writeBlocking( buffer, length );
        break;

      case Chimera::Hardware::PeripheralMode::INTERRUPT:
        error = writeInterrupt( buffer, length );
        break;

      case Chimera::Hardware::PeripheralMode::DMA:
        error = writeDMA( buffer, length );
        break;

      default:
        return Chimera::Status::FAIL;
        break;
    };

    return error;
  }


  Chimera::Status_t Driver::read( void *const buffer, const size_t length )
  {
    auto error = Chimera::Status::OK;

    switch( rxMode )
    {
      case Chimera::Hardware::PeripheralMode::BLOCKING:
        error = readBlocking( buffer, length );
        break;

      case Chimera::Hardware::PeripheralMode::INTERRUPT:
        error = readInterrupt( buffer, length );
        break;

      case Chimera::Hardware::PeripheralMode::DMA:
        error = readDMA( buffer, length );
        break;

      default:
        return Chimera::Status::FAIL;
        break;
    };

    return error;
  }


  Chimera::Status_t Driver::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    Chimera::Status_t error = Chimera::Status::OK;

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
      error = Chimera::Status::INVAL_FUNC_PARAM;
    }

    return error;
  }


  void Driver::postISRProcessing()
  {
    using namespace Chimera::Peripheral;
    using namespace Chimera::Serial;
    using namespace Thor::LLD::USART;

    if( !enabled )
    {
      return;
    }

    const auto flags = ::LLD::getDriver( channel )->getFlags();

    /*-------------------------------------------------------------------------------
    Handle TX Complete
    -------------------------------------------------------------------------------*/
    if ( flags & Runtime::Flag::TX_COMPLETE )
    {
      /*------------------------------------------------
      Clear out the flags which got us in here
      ------------------------------------------------*/
      ::LLD::getDriver( channel )->clearFlags( Runtime::Flag::TX_COMPLETE );
      auto cb  = txBuffers.circularBuffer();
      auto lb  = txBuffers.linearBuffer();

      /*------------------------------------------------
      Process Transmit Buffers
      ------------------------------------------------*/
      if ( !cb->empty() )
      {
        size_t bytesToTransmit = 0;
        txBuffers.transferInto( cb->size(), bytesToTransmit );
        write( lb, bytesToTransmit );
      }

      /*------------------------------------------------
      Notify those waiting on the event occurrance
      ------------------------------------------------*/
      awaitTXComplete.release();
      txLock.release();

      /*-------------------------------------------------
      Handle any user-space callback
      -------------------------------------------------*/
      auto callbacks = LLD::INT::getISRHandler( Type::PERIPH_USART, SIG_TX_COMPLETE );
      if( callbacks && callbacks->userCallback )
      {
        callbacks->userCallback( callbacks );
      }
    }

    /*-------------------------------------------------------------------------------
    Handle RX Complete or RX Idle
    -------------------------------------------------------------------------------*/
    if ( ( flags & ::LLD::Runtime::Flag::RX_COMPLETE ) || ( flags & Runtime::Flag::RX_LINE_IDLE_ABORT ) )
    {
      /*------------------------------------------------
      Clear out the flags which got us in here
      ------------------------------------------------*/
      ::LLD::getDriver( channel )->clearFlags( Runtime::Flag::RX_COMPLETE );
      ::LLD::getDriver( channel )->clearFlags( Runtime::Flag::RX_LINE_IDLE_ABORT );

      /*------------------------------------------------
      Process Receive Buffers
      ------------------------------------------------*/
      auto tcb      = ::LLD::getDriver( channel )->getTCB_RX();
      size_t tmp    = 0;

      rxBuffers.transferOutOf( tcb.remaining, tmp );

      /*------------------------------------------------
      Notify those waiting on the event occurrance
      ------------------------------------------------*/
      awaitRXComplete.release();
      rxLock.release();

      /*-------------------------------------------------
      Handle any user-space callback
      -------------------------------------------------*/
      auto callbacks = LLD::INT::getISRHandler( Type::PERIPH_USART, SIG_RX_COMPLETE );
      if( callbacks && callbacks->userCallback )
      {
        callbacks->userCallback( callbacks );
      }
    }
  }


  Chimera::Status_t Driver::toggleAsyncListening( const bool state )
  {
    if ( state )
    {
      return ::LLD::getDriver( channel )->receiveIT( rxBuffers.linearBuffer(), rxBuffers.linearSize() );
    }
    else
    {
      ::LLD::getDriver( channel )->killReceive();
      return Chimera::Status::OK;
    }
  }


  Chimera::Status_t Driver::readAsync( uint8_t *const buffer, const size_t len )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( !buffer )
    {
      error = Chimera::Status::INVAL_FUNC_PARAM;
    }
    else
    {
      size_t bytesRead = 0;

      if ( auto tmp = rxBuffers.circularBuffer(); tmp )
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
        error = Chimera::Status::EMPTY;
      }
    }

    return error;
  }


  Chimera::Status_t Driver::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                 boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                                 const size_t hwBufferSize )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      txBuffers.assign( userBuffer, hwBuffer, hwBufferSize );
      txBuffers.flush();
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      rxBuffers.assign( userBuffer, hwBuffer, hwBufferSize );
      rxBuffers.flush();
    }
    else
    {
      error = Chimera::Status::INVAL_FUNC_PARAM;
    }

    return error;
  }


  Chimera::Status_t Driver::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    Chimera::Status_t error = Chimera::Status::OK;

//    if ( periph == Chimera::Hardware::SubPeripheral::TX )
//    {
//      PeripheralState.tx_buffering_enabled = false;
//    }
//    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
//    {
//      PeripheralState.tx_buffering_enabled = false;
//    }
//    else
//    {
//      error = Chimera::Status::INVAL_FUNC_PARAM;
//    }

    return error;
  }


  bool Driver::available( size_t *const bytes )
  {
    bool retval = false;

    if ( auto tmp = rxBuffers.circularBuffer(); tmp && !tmp->empty() )
    {
      retval = true;

      if ( bytes )
      {
        *bytes = tmp->size();
      }
    }

    return retval;
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    using namespace Chimera::Event;

    if ( ( event != Trigger::TRIGGER_READ_COMPLETE ) && ( event != Trigger::TRIGGER_WRITE_COMPLETE ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    if ( ( event == Trigger::TRIGGER_WRITE_COMPLETE ) && !awaitTXComplete.try_acquire_for( timeout ) )
    {
      return Chimera::Status::TIMEOUT;
    }
    else if ( ( event == Trigger::TRIGGER_READ_COMPLETE ) && !awaitRXComplete.try_acquire_for( timeout ) )
    {
      return Chimera::Status::TIMEOUT;
    }

    awaitRXComplete.release();
    awaitTXComplete.release();

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Thread::BinarySemaphore &notifier,
                                       const size_t timeout )
  {
    auto result = await( event, timeout );

    if ( result == Chimera::Status::OK )
    {
      notifier.release();
    }

    return result;
  }


  Chimera::Status_t Driver::readBlocking( void *const buffer, const size_t length )
  {
    return ::LLD::getDriver( channel )->receive( buffer, length );
  }


  Chimera::Status_t Driver::readInterrupt( void *const buffer, const size_t length  )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( !rxBuffers.initialized() )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    if ( length <= rxBuffers.linearSize() )
    {
      memset( rxBuffers.linearBuffer(), 0, rxBuffers.linearSize() );
      error = ::LLD::getDriver( channel )->receiveIT( rxBuffers.linearBuffer(), length );

      if ( error == Chimera::Status::OK )
      {
        error = Chimera::Serial::Status::RX_IN_PROGRESS;
      }
    }
    else
    {
      error = Chimera::Status::MEMORY;
    }

    return error;
  }


  Chimera::Status_t Driver::readDMA( void *const buffer, const size_t length )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( !rxBuffers.initialized() )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    if ( length <= rxBuffers.linearSize() )
    {
      memset( rxBuffers.linearBuffer(), 0, rxBuffers.linearSize() );
      error = ::LLD::getDriver( channel )->receiveDMA( rxBuffers.linearBuffer(), length );

      if ( error == Chimera::Status::OK )
      {
        error = Chimera::Serial::Status::RX_IN_PROGRESS;
      }
    }
    else
    {
      error = Chimera::Status::MEMORY;
    }

    return error;
  }


  Chimera::Status_t Driver::writeBlocking( const void *const buffer, const size_t length )
  {
    return ::LLD::getDriver( channel )->transmit( buffer, length );
  }


  Chimera::Status_t Driver::writeInterrupt( const void *const buffer, const size_t length )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( !txBuffers.initialized() )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Hardware is free. Send the data directly. Otherwise
    queue everything up to send later.
    ------------------------------------------------*/
    if ( txLock.try_acquire() )
    {
      awaitTXComplete.try_acquire();
      error = ::LLD::getDriver( channel )->transmitIT( buffer, length );
    }
    else
    {
      size_t pushed = 0;
      error = Chimera::Status::BUSY;
      txBuffers.push( reinterpret_cast<const uint8_t *const>( buffer ), length, pushed );
    }

    return error;
  }


  Chimera::Status_t Driver::writeDMA( const void *const buffer, const size_t length )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( !txBuffers.initialized() )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Hardware is free. Send the data directly. Otherwise
    queue everything up to send later.
    ------------------------------------------------*/
    if ( txLock.try_acquire() )
    {
      awaitTXComplete.try_acquire();
      error = ::LLD::getDriver( channel )->transmitDMA( buffer, length );
    }
    else
    {
      size_t pushed = 0;
      error = Chimera::Status::BUSY;
      txBuffers.push( reinterpret_cast<const uint8_t *const>( buffer ), length, pushed );
    }

    return error;
  }

}    // namespace Thor::USART

#endif /* THOR_HLD_USART */
