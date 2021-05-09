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

/* Aurora Includes */
#include <Aurora/constants>
#include <Aurora/utility>

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
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/usart>


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
static size_t s_driver_initialized;           /**< Tracks the module level initialization state */
static HLD::Driver hld_driver[ NUM_DRIVERS ]; /**< Driver objects */

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
    if ( !this_thread::pendTaskMsg( ITCMsg::TSK_MSG_ISR_HANDLER ) )
    {
      continue;
    }

    /*-------------------------------------------------
    Handle every ISR. Don't know which triggered this.
    -------------------------------------------------*/
    for ( size_t index = 0; index < NUM_DRIVERS; index++ )
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


  bool isChannelUSART( const Chimera::Serial::Channel mChannel )
  {
    return ::LLD::isSupported( mChannel );
  }


  Driver_rPtr getDriver( const Chimera::Serial::Channel mChannel )
  {
    if ( auto idx = ::LLD::getResourceIndex( mChannel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
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
      mEnabled( false ), mChannel( Chimera::Serial::Channel::NOT_SUPPORTED ), mResourceIndex( 0 ), mAwaitRXComplete( 1 ),
      mAwaitTXComplete( 1 ), mRxLock( 1 ), mTxLock( 1 )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::assignHW( const Chimera::Serial::Channel channel, const Chimera::Serial::IOPins &pins )
  {
    /*------------------------------------------------
    Make sure the mChannel is actually supported
    ------------------------------------------------*/
    mChannel       = channel;
    mResourceIndex = ::LLD::getResourceIndex( mChannel );

    if ( mResourceIndex == Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*------------------------------------------------
    Initialize/Configure hardware drivers
    ------------------------------------------------*/
    auto result = Chimera::Status::OK;

    result |= Chimera::GPIO::getDriver( pins.tx.port, pins.tx.pin )->init( pins.tx );
    result |= Chimera::GPIO::getDriver( pins.rx.port, pins.rx.pin )->init( pins.rx );

    return result;
  }


  Chimera::Status_t Driver::begin( const Chimera::Hardware::PeripheralMode txMode,
                                   const Chimera::Hardware::PeripheralMode rxMode )
  {
    /*------------------------------------------------
    Ensure the internal event signals are reset
    ------------------------------------------------*/
    mAwaitRXComplete.try_acquire();
    mAwaitTXComplete.try_acquire();

    /*------------------------------------------------
    Initialize to the desired TX/RX modes
    ------------------------------------------------*/
    setMode( Chimera::Hardware::SubPeripheral::RX, rxMode );
    setMode( Chimera::Hardware::SubPeripheral::TX, txMode );

    mEnabled = true;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::end()
  {
    return ::LLD::getDriver( mChannel )->deinit();
  }


  Chimera::Status_t Driver::configure( const Chimera::Serial::Config &config )
  {
    Thor::LLD::Serial::Config cfg = ::LLD::getDriver( mChannel )->getConfiguration();

    /*------------------------------------------------
    Convert between the generalize Chimera options into
    MCU specific register configuration settings. If these lookup
    tables fail, you might have something wrong with the mappings.
    ------------------------------------------------*/
    cfg.BaudRate   = static_cast<uint32_t>( config.baud );
    cfg.Mode       = ::LLD::Configuration::Modes::TX_RX;
    cfg.Parity     = ::LLD::ConfigMap::Parity[ EnumValue( config.parity ) ];
    cfg.StopBits   = ::LLD::ConfigMap::StopBits[ EnumValue( config.stopBits ) ];
    cfg.WordLength = ::LLD::ConfigMap::CharWidth[ EnumValue( config.width ) ];

    return ::LLD::getDriver( mChannel )->init( cfg );
  }


  Chimera::Status_t Driver::setBaud( const uint32_t baud )
  {
    auto currentConfig     = ::LLD::getDriver( mChannel )->getConfiguration();
    currentConfig.BaudRate = baud;

    return ::LLD::getDriver( mChannel )->init( currentConfig );
  }


  Chimera::Status_t Driver::setMode( const Chimera::Hardware::SubPeripheral periph,
                                     const Chimera::Hardware::PeripheralMode mode )
  {
    using namespace Chimera::Hardware;

    if ( ( periph == SubPeripheral::RX ) || ( periph == SubPeripheral::TXRX ) )
    {
      mRxMode = mode;
    }

    if ( ( periph == SubPeripheral::TX ) || ( periph == SubPeripheral::TXRX ) )
    {
      mTxMode = mode;
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::write( const void *const buffer, const size_t length )
  {
    auto error = Chimera::Status::OK;

    switch ( mTxMode )
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

    switch ( mRxMode )
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
      error = mTxBuffers.flush();
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      error = mRxBuffers.flush();
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

    /*-------------------------------------------------
    Entrancy protection
    -------------------------------------------------*/
    if ( !mEnabled )
    {
      return;
    }

    /*-------------------------------------------------
    Read why the LLD decided to wake us up
    -------------------------------------------------*/
    const auto flags = ::LLD::getDriver( mChannel )->getFlags();

    /*-------------------------------------------------------------------------------
    Handle TX Complete
    -------------------------------------------------------------------------------*/
    if ( flags & Runtime::Flag::TX_COMPLETE )
    {
      /*------------------------------------------------
      Clear out the flags which got us in here
      ------------------------------------------------*/
      ::LLD::getDriver( mChannel )->clearFlags( Runtime::Flag::TX_COMPLETE );
      auto cb = mTxBuffers.circularBuffer();
      auto lb = mTxBuffers.linearBuffer();

      /*------------------------------------------------
      Process Transmit Buffers:
      Pull waiting data from the circular buffer into
      the linear buffer for the HW to transmit from.
      ------------------------------------------------*/
      if ( !cb->empty() )
      {
        size_t bytesToTransmit = 0;
        mTxBuffers.transferInto( cb->size(), bytesToTransmit );
        write( lb, bytesToTransmit );
      }

      /*------------------------------------------------
      Notify those waiting on the TX occurrance
      ------------------------------------------------*/
      mAwaitTXComplete.release();
      mTxLock.release();

      /*-------------------------------------------------
      Handle any user-space callback that was registered
      with the Interrupt module for this peripheral.
      -------------------------------------------------*/
      auto callbacks = LLD::INT::getISRHandler( Type::PERIPH_USART, SIG_TX_COMPLETE );
      if ( callbacks && callbacks->userCallback )
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
      ::LLD::getDriver( mChannel )->clearFlags( Runtime::Flag::RX_COMPLETE );
      ::LLD::getDriver( mChannel )->clearFlags( Runtime::Flag::RX_LINE_IDLE_ABORT );

      /*------------------------------------------------
      Process Receive Buffers:
      Get the transfer control block for the last RX and
      move the data from the linear HW buffer into the
      circular buffer for the user to read.
      ------------------------------------------------*/
      auto tcb   = ::LLD::getDriver( mChannel )->getTCB_RX();
      size_t tmp = 0;

      mRxBuffers.transferOutOf( tcb.remaining, tmp );

      /*------------------------------------------------
      Notify those waiting on the RX occurrance
      ------------------------------------------------*/
      mAwaitRXComplete.release();
      mRxLock.release();

      /*-------------------------------------------------
      Handle any user-space callback that was registered
      with the Interrupt module for this peripheral.
      -------------------------------------------------*/
      auto callbacks = LLD::INT::getISRHandler( Type::PERIPH_USART, SIG_RX_COMPLETE );
      if ( callbacks && callbacks->userCallback )
      {
        callbacks->userCallback( callbacks );
      }
    }
  }


  Chimera::Status_t Driver::toggleAsyncListening( const bool state )
  {
    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( mRxMode == Chimera::Hardware::PeripheralMode::BLOCKING )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Start listening if requested, else kill ongoing
    -------------------------------------------------*/
    if ( state )
    {
      return this->read( mRxBuffers.linearBuffer(), mRxBuffers.linearSize() );
    }
    else
    {
      ::LLD::getDriver( mChannel )->killReceive();
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

      if ( auto tmp = mRxBuffers.circularBuffer(); tmp )
      {
        while ( !tmp->empty() && ( bytesRead < len ) )
        {
          buffer[ bytesRead ] = tmp->front();
          tmp->pop();
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
                                             Chimera::Serial::CircularBuffer &userBuffer, uint8_t *const hwBuffer,
                                             const size_t hwBufferSize )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      mTxBuffers.assign( userBuffer, hwBuffer, hwBufferSize );
      mTxBuffers.flush();
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      mRxBuffers.assign( userBuffer, hwBuffer, hwBufferSize );
      mRxBuffers.flush();
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
    //      PeripheralState.tx_buffering_mEnabled = false;
    //    }
    //    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    //    {
    //      PeripheralState.tx_buffering_mEnabled = false;
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

    if ( auto tmp = mRxBuffers.circularBuffer(); tmp && !tmp->empty() )
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

    if ( ( event == Trigger::TRIGGER_WRITE_COMPLETE ) && !mAwaitTXComplete.try_acquire_for( timeout ) )
    {
      return Chimera::Status::TIMEOUT;
    }
    else if ( ( event == Trigger::TRIGGER_READ_COMPLETE ) && !mAwaitRXComplete.try_acquire_for( timeout ) )
    {
      return Chimera::Status::TIMEOUT;
    }

    mAwaitRXComplete.release();
    mAwaitTXComplete.release();

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
    return ::LLD::getDriver( mChannel )->receive( buffer, length );
  }


  Chimera::Status_t Driver::readInterrupt( void *const buffer, const size_t length )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( !mRxBuffers.initialized() )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    if ( length <= mRxBuffers.linearSize() )
    {
      memset( mRxBuffers.linearBuffer(), 0, mRxBuffers.linearSize() );
      error = ::LLD::getDriver( mChannel )->receiveIT( mRxBuffers.linearBuffer(), length );

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

    if ( !mRxBuffers.initialized() )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    if ( length <= mRxBuffers.linearSize() )
    {
      memset( mRxBuffers.linearBuffer(), 0, mRxBuffers.linearSize() );
      error = ::LLD::getDriver( mChannel )->receiveDMA( mRxBuffers.linearBuffer(), length );

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
    return ::LLD::getDriver( mChannel )->transmit( buffer, length );
  }


  Chimera::Status_t Driver::writeInterrupt( const void *const buffer, const size_t length )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( !mTxBuffers.initialized() )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Hardware is free. Send the data directly. Otherwise
    queue everything up to send later.
    ------------------------------------------------*/
    if ( mTxLock.try_acquire() )
    {
      mAwaitTXComplete.try_acquire();
      error = ::LLD::getDriver( mChannel )->transmitIT( buffer, length );
    }
    else
    {
      size_t pushed = 0;
      error         = Chimera::Status::BUSY;
      mTxBuffers.push( reinterpret_cast<const uint8_t *const>( buffer ), length, pushed );
    }

    return error;
  }


  Chimera::Status_t Driver::writeDMA( const void *const buffer, const size_t length )
  {
    Chimera::Status_t error = Chimera::Status::OK;

    if ( !mTxBuffers.initialized() )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Hardware is free. Send the data directly. Otherwise
    queue everything up to send later.
    ------------------------------------------------*/
    if ( mTxLock.try_acquire() )
    {
      mAwaitTXComplete.try_acquire();
      error = ::LLD::getDriver( mChannel )->transmitDMA( buffer, length );
    }
    else
    {
      size_t pushed = 0;
      error         = Chimera::Status::BUSY;
      mTxBuffers.push( reinterpret_cast<const uint8_t *const>( buffer ), length, pushed );
    }

    return error;
  }

}    // namespace Thor::USART

#endif /* THOR_HLD_USART */
