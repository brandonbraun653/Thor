/********************************************************************************
 *  File Name:
 *    thor_custom_usart.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor USART interface.
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Aurora/utility>
#include <Chimera/assert>
#include <Chimera/buffer>
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/interrupt>
#include <Chimera/serial>
#include <Chimera/thread>
#include <Chimera/usart>
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/event>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/usart>
#include <array>
#include <cstdint>
#include <memory>


#if defined( THOR_USART )
namespace Chimera::USART
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::USART;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_USART_PERIPHS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ThorImpl
  {
    LLD::Driver_rPtr            lldriver;
    Chimera::USART::Driver_rPtr hldriver;

    /*-------------------------------------------------
    Misc state variables
    -------------------------------------------------*/
    bool                     mEnabled;       /**< Has the peripheral been enabled */
    Chimera::Serial::Channel mChannel;       /**< Hardware channel associated with this driver */
    size_t                   mResourceIndex; /**< Lookup table index for USART resources */

    /*-------------------------------------------------
    Internal locks for protecting the data buffers
    -------------------------------------------------*/
    Chimera::Thread::RecursiveMutex  mRxLock;
    Chimera::Thread::BinarySemaphore mTxLock;

    /*-------------------------------------------------
    Buffers for queueing data and interacting with HW
    -------------------------------------------------*/
    Chimera::Buffer::PeripheralBuffer mTxBuffers;
    Chimera::Buffer::PeripheralBuffer mRxBuffers;

    /*-------------------------------------------------
    Selects the HW mode each data line operates in
    -------------------------------------------------*/
    Chimera::Hardware::PeripheralMode mTxMode;
    Chimera::Hardware::PeripheralMode mRxMode;

    ThorImpl() :
        mEnabled( false ), mChannel( Chimera::Serial::Channel::NOT_SUPPORTED ), mResourceIndex( 0 ), mTxLock( 1 )
    {
    }

    Chimera::Status_t readBlocking( void *const buffer, const size_t length )
    {
      return lldriver->receive( buffer, length );
    }


    Chimera::Status_t readInterrupt( void *const buffer, const size_t length )
    {
      Chimera::Status_t error = Chimera::Status::OK;

      if ( !mRxBuffers.initialized() )
      {
        return Chimera::Status::NOT_INITIALIZED;
      }

      if ( length <= mRxBuffers.linearSize() )
      {
        memset( mRxBuffers.linearBuffer(), 0, mRxBuffers.linearSize() );
        error = lldriver->receiveIT( mRxBuffers.linearBuffer(), length );

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


    Chimera::Status_t readDMA( void *const buffer, const size_t length )
    {
      Chimera::Status_t error = Chimera::Status::OK;

      if ( !mRxBuffers.initialized() )
      {
        return Chimera::Status::NOT_INITIALIZED;
      }

      if ( length <= mRxBuffers.linearSize() )
      {
        memset( mRxBuffers.linearBuffer(), 0, mRxBuffers.linearSize() );
        error = lldriver->receiveDMA( mRxBuffers.linearBuffer(), length );

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


    Chimera::Status_t writeBlocking( const void *const buffer, const size_t length )
    {
      return lldriver->transmit( buffer, length );
    }


    Chimera::Status_t writeInterrupt( const void *const buffer, const size_t length )
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
        error = lldriver->transmitIT( buffer, length );
      }
      else
      {
        size_t pushed = 0;
        error         = Chimera::Status::BUSY;
        mTxBuffers.push( reinterpret_cast<const uint8_t *const>( buffer ), length, pushed );
      }

      return error;
    }


    Chimera::Status_t writeDMA( const void *const buffer, const size_t length )
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
        error = lldriver->transmitDMA( buffer, length );
      }
      else
      {
        size_t pushed = 0;
        error         = Chimera::Status::BUSY;
        mTxBuffers.push( reinterpret_cast<const uint8_t *const>( buffer ), length, pushed );
      }

      return error;
    }
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static size_t s_driver_initialized; /**< Tracks the module level initialization state */
  static DeviceManager<Driver, Chimera::Serial::Channel, NUM_DRIVERS>   s_raw_drivers;
  static DeviceManager<ThorImpl, Chimera::Serial::Channel, NUM_DRIVERS> s_impl_drivers;

  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
  static void USARTxISRUserThread( void *arg )
  {
    using namespace Chimera::Thread;

    Chimera::Serial::Channel instance_list[ EnumValue( Chimera::Serial::Channel::NUM_OPTIONS ) ];

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
      const size_t count = s_raw_drivers.registeredInstances( instance_list, ARRAY_COUNT( instance_list ) );
      for ( size_t idx = 0; idx < count; idx++ )
      {
        auto impl = s_raw_drivers.get( instance_list[ idx ] );
        impl->postISRProcessing();
      }
    }
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::assignHW( const Chimera::Serial::Channel channel, const Chimera::Serial::IOPins &pins )
  {
    /*-------------------------------------------------------------------------
    Ensure the channel is supported
    -------------------------------------------------------------------------*/
    auto index = LLD::getResourceIndex( channel );
    if ( index == Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Load up the drivers
    -------------------------------------------------------------------------*/
    auto impl = s_impl_drivers.getOrCreate( channel );
    RT_DBG_ASSERT( impl );

    impl->lldriver       = LLD::getDriver( channel );
    impl->hldriver       = this;
    impl->mChannel       = channel;
    impl->mResourceIndex = index;

    mImpl = reinterpret_cast<void *>( impl );

    /*-------------------------------------------------------------------------
    Initialize the GPIO pins
    -------------------------------------------------------------------------*/
    auto result = Chimera::Status::OK;

    result |= Chimera::GPIO::getDriver( pins.tx.port, pins.tx.pin )->init( pins.tx );
    result |= Chimera::GPIO::getDriver( pins.rx.port, pins.rx.pin )->init( pins.rx );

    return result;
  }


  Chimera::Status_t Driver::begin( const Chimera::Hardware::PeripheralMode txMode,
                                   const Chimera::Hardware::PeripheralMode rxMode )
  {
    using namespace Chimera::Hardware;
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    /*-------------------------------------------------------------------------
    Initialize AsyncIO for user notification of events
    -------------------------------------------------------------------------*/
    this->initAIO();

    /*-------------------------------------------------------------------------
    Initialize to the desired TX/RX modes
    -------------------------------------------------------------------------*/
    setMode( SubPeripheral::RX, rxMode );
    setMode( SubPeripheral::TX, txMode );
    impl->mEnabled = true;

    /*-------------------------------------------------------------------------
    Start listening for data
    -------------------------------------------------------------------------*/
    if ( ( rxMode == PeripheralMode::DMA ) || ( rxMode == PeripheralMode::INTERRUPT ) )
    {
      return this->toggleAsyncListening( true );
    }
    else
    {
      return Chimera::Status::OK;
    }
  }


  Chimera::Status_t Driver::end()
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    return impl->lldriver->deinit();
  }


  Chimera::Status_t Driver::configure( const Chimera::Serial::Config &config )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    Thor::LLD::Serial::Config cfg = impl->lldriver->getConfiguration();

    /*------------------------------------------------
    Convert between the generalize Chimera options into
    MCU specific register configuration settings. If these lookup
    tables fail, you might have something wrong with the mappings.
    ------------------------------------------------*/
    cfg.BaudRate   = static_cast<uint32_t>( config.baud );
    cfg.Mode       = LLD::Configuration::Modes::TX_RX;
    cfg.Parity     = LLD::ConfigMap::Parity[ EnumValue( config.parity ) ];
    cfg.StopBits   = LLD::ConfigMap::StopBits[ EnumValue( config.stopBits ) ];
    cfg.WordLength = LLD::ConfigMap::CharWidth[ EnumValue( config.width ) ];

    return impl->lldriver->init( cfg );
  }


  Chimera::Status_t Driver::setBaud( const uint32_t baud )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    auto currentConfig     = impl->lldriver->getConfiguration();
    currentConfig.BaudRate = baud;

    return impl->lldriver->init( currentConfig );
  }


  Chimera::Status_t Driver::setMode( const Chimera::Hardware::SubPeripheral  periph,
                                     const Chimera::Hardware::PeripheralMode mode )
  {
    using namespace Chimera::Hardware;
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    if ( ( periph == SubPeripheral::RX ) || ( periph == SubPeripheral::TXRX ) )
    {
      impl->mRxMode = mode;
    }

    if ( ( periph == SubPeripheral::TX ) || ( periph == SubPeripheral::TXRX ) )
    {
      impl->mTxMode = mode;
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::write( const void *const buffer, const size_t length )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl  = reinterpret_cast<ThorImpl *>( mImpl );
    auto error = Chimera::Status::OK;

    switch ( impl->mTxMode )
    {
      case Chimera::Hardware::PeripheralMode::BLOCKING:
        error = impl->writeBlocking( buffer, length );
        break;

      case Chimera::Hardware::PeripheralMode::INTERRUPT:
        error = impl->writeInterrupt( buffer, length );
        break;

      case Chimera::Hardware::PeripheralMode::DMA:
        error = impl->writeDMA( buffer, length );
        break;

      default:
        return Chimera::Status::FAIL;
        break;
    };

    return error;
  }


  Chimera::Status_t Driver::read( void *const buffer, const size_t length )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl  = reinterpret_cast<ThorImpl *>( mImpl );
    auto error = Chimera::Status::OK;

    switch ( impl->mRxMode )
    {
      case Chimera::Hardware::PeripheralMode::BLOCKING:
        error = impl->readBlocking( buffer, length );
        break;

      case Chimera::Hardware::PeripheralMode::INTERRUPT:
        error = impl->readInterrupt( buffer, length );
        break;

      case Chimera::Hardware::PeripheralMode::DMA:
        error = impl->readDMA( buffer, length );
        break;

      default:
        return Chimera::Status::FAIL;
        break;
    };

    return error;
  }


  Chimera::Status_t Driver::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl  = reinterpret_cast<ThorImpl *>( mImpl );
    auto error = Chimera::Status::OK;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      impl->mTxLock.acquire();
      error = impl->mTxBuffers.flush();
      impl->mTxLock.release();
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      impl->mRxLock.lock();
      error = impl->mRxBuffers.flush();
      impl->mRxLock.unlock();
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
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    if ( !impl->mEnabled )
    {
      return;
    }

    /*-------------------------------------------------
    Read why the LLD decided to wake us up
    -------------------------------------------------*/
    const auto flags = impl->lldriver->getFlags();

    /*-------------------------------------------------------------------------------
    Handle TX Complete
    -------------------------------------------------------------------------------*/
    if ( flags & Runtime::Flag::TX_COMPLETE )
    {
      /*------------------------------------------------
      Clear out the flags which got us in here
      ------------------------------------------------*/
      impl->lldriver->clearFlags( Runtime::Flag::TX_COMPLETE );
      auto cb = impl->mTxBuffers.circularBuffer();
      auto lb = impl->mTxBuffers.linearBuffer();

      /*------------------------------------------------
      Process Transmit Buffers:
      Pull waiting data from the circular buffer into
      the linear buffer for the HW to transmit from.
      ------------------------------------------------*/
      if ( !cb->empty() )
      {
        size_t bytesToTransmit = 0;
        impl->mTxBuffers.transferInto( cb->size(), bytesToTransmit );
        write( lb, bytesToTransmit );
      }

      /*------------------------------------------------
      Notify those waiting on the TX occurrance
      ------------------------------------------------*/
      impl->mTxLock.release();
      impl->hldriver->signalAIO( Chimera::Event::Trigger::TRIGGER_WRITE_COMPLETE );

      /*-------------------------------------------------
      Handle any user-space callback that was registered
      with the Interrupt module for this peripheral.
      -------------------------------------------------*/
      auto callbacks = Thor::LLD::INT::getISRHandler( Type::PERIPH_USART, SIG_TX_COMPLETE );
      if ( callbacks && callbacks->userCallback )
      {
        callbacks->userCallback( callbacks );
      }
    }

    /*-------------------------------------------------------------------------------
    Handle RX Complete or RX Idle
    -------------------------------------------------------------------------------*/
    if ( ( flags & LLD::Runtime::Flag::RX_COMPLETE ) || ( flags & Runtime::Flag::RX_LINE_IDLE_ABORT ) )
    {
      /*------------------------------------------------
      Clear out the flags which got us in here
      ------------------------------------------------*/
      impl->lldriver->clearFlags( Runtime::Flag::RX_COMPLETE );
      impl->lldriver->clearFlags( Runtime::Flag::RX_LINE_IDLE_ABORT );

      /*------------------------------------------------
      Process Receive Buffers:
      Get the transfer control block for the last RX and
      move the data from the linear HW buffer into the
      circular buffer for the user to read.
      ------------------------------------------------*/
      auto   tcb            = impl->lldriver->getTCB_RX();
      size_t bytes_received = tcb.expected - tcb.remaining;
      size_t tmp            = 0;

      impl->mRxLock.lock();
      impl->mRxBuffers.transferOutOf( bytes_received, tmp );
      impl->mRxLock.unlock();

      /*------------------------------------------------
      Notify those waiting on an RX occurrence
      ------------------------------------------------*/
      impl->hldriver->signalAIO( Chimera::Event::Trigger::TRIGGER_READ_COMPLETE );

      /*-------------------------------------------------
      Handle any user-space callback that was registered
      with the Interrupt module for this peripheral.
      -------------------------------------------------*/
      auto callbacks = Thor::LLD::INT::getISRHandler( Type::PERIPH_USART, SIG_RX_COMPLETE );
      if ( callbacks && callbacks->userCallback )
      {
        callbacks->userCallback( callbacks );
      }

      /*-------------------------------------------------
      Finally, start listening once more for more data
      -------------------------------------------------*/
      this->toggleAsyncListening( true );
    }
  }


  Chimera::Status_t Driver::toggleAsyncListening( const bool state )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    /*-------------------------------------------------
    Input protection
    -------------------------------------------------*/
    if ( impl->mRxMode == Chimera::Hardware::PeripheralMode::BLOCKING )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Start listening if requested, else kill ongoing
    -------------------------------------------------*/
    if ( state )
    {
      return this->read( impl->mRxBuffers.linearBuffer(), impl->mRxBuffers.linearSize() );
    }
    else
    {
      impl->lldriver->killReceive();
      return Chimera::Status::OK;
    }
  }


  Chimera::Status_t Driver::readAsync( uint8_t *const buffer, const size_t len )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    /*-------------------------------------------------------------------------
    Input protection
    -------------------------------------------------------------------------*/
    if ( !buffer )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Read out data accumulated asynchronously
    -------------------------------------------------------------------------*/
    Chimera::Status_t error     = Chimera::Status::OK;
    size_t            bytesRead = 0;

    impl->mRxLock.lock();
    {
      if ( auto tmp = impl->mRxBuffers.circularBuffer(); tmp )
      {
        while ( !tmp->empty() && ( bytesRead < len ) )
        {
          buffer[ bytesRead ] = tmp->front();
          tmp->pop();
          bytesRead++;
        }
      }
    }
    impl->mRxLock.unlock();


    if ( bytesRead != len )
    {
      error = Chimera::Status::EMPTY;
    }

    return error;
  }


  Chimera::Status_t Driver::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                             Chimera::Serial::CircularBuffer &userBuffer, uint8_t *const hwBuffer,
                                             const size_t hwBufferSize )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl  = reinterpret_cast<ThorImpl *>( mImpl );
    auto error = Chimera::Status::OK;

    if ( periph == Chimera::Hardware::SubPeripheral::TX )
    {
      impl->mTxBuffers.assign( userBuffer, hwBuffer, hwBufferSize );
      impl->mTxBuffers.flush();
    }
    else if ( periph == Chimera::Hardware::SubPeripheral::RX )
    {
      impl->mRxBuffers.assign( userBuffer, hwBuffer, hwBufferSize );
      impl->mRxBuffers.flush();
    }
    else
    {
      error = Chimera::Status::INVAL_FUNC_PARAM;
    }

    return error;
  }


  Chimera::Status_t Driver::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    RT_DBG_ASSERT( mImpl );
    // auto impl  = reinterpret_cast<ThorImpl *>( mImpl );
    auto error = Chimera::Status::OK;

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
    RT_DBG_ASSERT( mImpl );
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    bool retval = false;

    /*-------------------------------------------------------------------------
    Check the RX user (circular) buffer if anything available
    -------------------------------------------------------------------------*/
    if ( auto tmp = impl->mRxBuffers.circularBuffer(); tmp )
    {
      impl->mRxLock.lock();
      retval = !tmp->empty();

      if ( bytes )
      {
        *bytes = static_cast<size_t>( tmp->size() );
      }
      impl->mRxLock.unlock();
    }

    return retval;
  }

}    // namespace Chimera::USART


namespace Chimera::USART::Backend
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initialize()
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
    Task       userThread;
    TaskConfig cfg;

    cfg.arg        = nullptr;
    cfg.function   = USARTxISRUserThread;
    cfg.priority   = Priority::MAXIMUM;
    cfg.stackWords = STACK_BYTES( 512 );
    cfg.type       = TaskInitType::DYNAMIC;
    cfg.name       = "PP_USARTx";

    userThread.create( cfg );
    Thor::LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_USART, userThread.start() );

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    LLD::initialize();

    /*-------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  static Chimera::Status_t reset()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }

  static Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    if ( auto idx = LLD::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return s_raw_drivers.getOrCreate( channel );
    }
    else
    {
      return nullptr;
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::USART::Backend::DriverConfig &registry )
  {
    registry.isSupported    = true;
    registry.getDriver      = ::Chimera::USART::Backend::getDriver;
    registry.initialize     = ::Chimera::USART::Backend::initialize;
    registry.reset          = ::Chimera::USART::Backend::reset;
    registry.isChannelUSART = LLD::isSupported;
    return Chimera::Status::OK;
  }
}    // namespace Chimera::USART::Backend
#endif /* THOR_USART */
