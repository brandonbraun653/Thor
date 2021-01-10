/********************************************************************************
 *  File Name:
 *    thor_custom_usart.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor USART interface.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
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
#include <Chimera/event>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/event>
#include <Thor/usart>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_detail.hpp>
#include <Thor/lld/interface/usart/usart_prv_data.hpp>

#if defined( THOR_HLD_USART )

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::USART;
namespace LLD = ::Thor::LLD::USART;

using ThreadHandle = Chimera::Threading::detail::native_thread_handle_type;
using BinarySemphr = Chimera::Threading::BinarySemaphore;
using ThreadFunctn = Chimera::Function::void_func_void_ptr;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = LLD::NUM_USART_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static size_t s_driver_initialized;                        /**< Tracks the module level initialization state */
static HLD::Driver hld_driver[ NUM_DRIVERS ];              /**< Driver objects */
static ThreadHandle s_user_isr_handle[ NUM_DRIVERS ];      /**< Handle to the ISR post processing thread */
static BinarySemphr s_user_isr_signal[ NUM_DRIVERS ];      /**< Lock for each ISR post processing thread */
static ThreadFunctn s_user_isr_thread_func[ NUM_DRIVERS ]; /**< RTOS aware function to execute at end of ISR */

/*-------------------------------------------------------------------------------
Private Function Declarations
-------------------------------------------------------------------------------*/
#if defined( STM32_USART1_PERIPH_AVAILABLE )
static void USART1ISRPostProcessorThread( void *argument );
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
static void USART2ISRPostProcessorThread( void *argument );
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
static void USART3ISRPostProcessorThread( void *argument );
#endif


namespace Thor::USART
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*------------------------------------------------
    Prevent multiple initializations (need reset first)
    ------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    ::LLD::initialize();

    /*-------------------------------------------------
    Initialize ISR post-processing routines
    -------------------------------------------------*/
#if defined( STM32_USART1_PERIPH_AVAILABLE )
    s_user_isr_thread_func[ ::LLD::USART1_RESOURCE_INDEX ] = USART1ISRPostProcessorThread;
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
    s_user_isr_thread_func[ ::LLD::USART2_RESOURCE_INDEX ] = USART2ISRPostProcessorThread;
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
    s_user_isr_thread_func[ ::LLD::USART3_RESOURCE_INDEX ] = USART3ISRPostProcessorThread;
#endif

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
      return nullptr;
    }
  }


  Driver_sPtr getDriverShared( const Chimera::Serial::Channel channel )
  {
    if ( auto idx = ::LLD::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return ::HLD::Driver_sPtr( &hld_driver[ idx ] );
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() :
      channel( Chimera::Serial::Channel::NOT_SUPPORTED ), resourceIndex( 0 ), listenerIDCount( 0 ), awaitRXComplete( 1 ),
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

    /*------------------------------------------------
    Configure miscellaneous class members
    ------------------------------------------------*/
    listenerIDCount = 0u;
    eventListeners.clear();

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

    /*------------------------------------------------
    Register the ISR post processor thread
    ------------------------------------------------*/
    std::array<char, 10> tmp;

    if ( s_user_isr_thread_func[ resourceIndex ] )
    {
      s_user_isr_handle[ resourceIndex ] = {};

      ::LLD::getDriver( channel )->attachISRWakeup( &s_user_isr_signal[ resourceIndex ] );

      tmp.fill( 0 );
      snprintf( tmp.data(), tmp.size(), "PP_USART%d", resourceIndex );
      std::string_view threadName = tmp.data();

      Chimera::Threading::Thread thread;
      thread.initialize( s_user_isr_thread_func[ resourceIndex ], nullptr, Chimera::Threading::Priority::MAXIMUM,
                         STACK_BYTES( 250 ), threadName );
      thread.start();
      s_user_isr_handle[ resourceIndex ] = thread.native_handle();
    }

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
    using namespace Thor::LLD::USART;

    const auto flags = ::LLD::getDriver( channel )->getFlags();
    //auto event       = Chimera::Event::TRIGGER_INVALID;

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
      Process Event Listeners. Semaphores unlocked in preparation
      for the listeners to possibly try and write more data.
      ------------------------------------------------*/
      awaitTXComplete.release();
      txLock.release();

      processListeners( Chimera::Event::Trigger::TRIGGER_WRITE_COMPLETE );
    }

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
      size_t rxSize = tcb.expected - tcb.remaining;
      size_t tmp    = 0;

      rxBuffers.transferOutOf( tcb.remaining, tmp );

      /*------------------------------------------------
      Process Event Listeners. Semaphores unlocked in preparation
      for the listeners to possibly try and read more data.
      ------------------------------------------------*/
      awaitRXComplete.release();
      rxLock.release();

      processListeners( Chimera::Event::Trigger::TRIGGER_READ_COMPLETE );
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


  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                                       const size_t timeout )
  {
    auto result = await( event, timeout );

    if ( result == Chimera::Status::OK )
    {
      notifier.release();
    }

    return result;
  }


  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                                  size_t &registrationID )
  {
    registrationID = ++listenerIDCount;
    listener.id    = registrationID;

    eventListeners.push_back( listener );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    for ( auto iterator = eventListeners.begin(); iterator != eventListeners.end(); iterator++ )
    {
      if ( iterator->id == registrationID )
      {
        eventListeners.erase( iterator );
        return Chimera::Status::OK;
        break;
      }
    }

    return Chimera::Status::NOT_FOUND;
  }


  void Driver::processListeners( const Chimera::Event::Trigger event )
  {
    Chimera::Event::notifyListenerList( event, eventListeners, 0 );
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


#if defined( STM32_USART1_PERIPH_AVAILABLE )
static void USART1ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::USART1_RESOURCE_INDEX;

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif  /* STM32_USART1_PERIPH_AVAILABLE */


#if defined( STM32_USART2_PERIPH_AVAILABLE )
static void USART2ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::USART2_RESOURCE_INDEX;

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif  /* STM32_USART2_PERIPH_AVAILABLE */


#if defined( STM32_USART3_PERIPH_AVAILABLE )
static void USART3ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::USART3_RESOURCE_INDEX;

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif  /* STM32_USART3_PERIPH_AVAILABLE */

#endif /* THOR_HLD_USART */
