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

/* Boost Includes */
#include <boost/circular_buffer.hpp>

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/event>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/event>
#include <Thor/usart>
#include <Thor/lld/interface/usart/usart.hpp>

/*------------------------------------------------
Static Functions
------------------------------------------------*/
static void USART1ISRPostProcessorThread( void *argument );
static void USART2ISRPostProcessorThread( void *argument );
static void USART3ISRPostProcessorThread( void *argument );
static void USART6ISRPostProcessorThread( void *argument );

/*------------------------------------------------
Static Data
------------------------------------------------*/
/* clang-format off */
static std::array<Thor::USART::Driver *, Thor::LLD::USART::NUM_USART_PERIPHS> usartClassObjects = { 
  nullptr,
  nullptr,
  nullptr,
  nullptr 
};

/* Post Processor Thread Handles */
static std::array<Chimera::Threading::detail::native_thread_handle_type, Thor::LLD::USART::NUM_USART_PERIPHS> postProcessorHandle = {};

/* Post Processor Thread Wakeup Signals */
static std::array<Chimera::Threading::BinarySemaphore, Thor::LLD::USART::NUM_USART_PERIPHS> postProcessorSignal = {};

/* Post Processor Thread Function Pointers */
static std::array<Chimera::Function::void_func_void_ptr, Thor::LLD::USART::NUM_USART_PERIPHS> postProcessorThread = {
  USART1ISRPostProcessorThread, 
  USART2ISRPostProcessorThread, 
  USART3ISRPostProcessorThread,
  USART6ISRPostProcessorThread
};
/* clang-format on */

namespace Chimera::USART::Backend
{
  Chimera::Status_t prjInitialize()
  {
    return Thor::USART::initialize();
  }
}    // namespace Chimera::USART

namespace Thor::USART
{
  static size_t s_driver_initialized;

  Chimera::Status_t initialize()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::USART::initialize();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }

  Driver::Driver() : channel( 0 ), resourceIndex( 0 ), listenerIDCount( 0 ),
    awaitRXComplete( 1 ), awaitTXComplete( 1 ), rxLock( 1 ), txLock( 1 )
  {
    using namespace Chimera::Hardware;

    /*------------------------------------------------
    Register the read function pointers
    ------------------------------------------------*/
    readFuncPtrs[ static_cast<uint8_t>( PeripheralMode::BLOCKING ) ]  = &Driver::readBlocking;
    readFuncPtrs[ static_cast<uint8_t>( PeripheralMode::INTERRUPT ) ] = &Driver::readInterrupt;
    readFuncPtrs[ static_cast<uint8_t>( PeripheralMode::DMA ) ]       = &Driver::readDMA;

    /*------------------------------------------------
    Register the write function pointers
    ------------------------------------------------*/
    writeFuncPtrs[ static_cast<uint8_t>( PeripheralMode::BLOCKING ) ]  = &Driver::writeBlocking;
    writeFuncPtrs[ static_cast<uint8_t>( PeripheralMode::INTERRUPT ) ] = &Driver::writeInterrupt;
    writeFuncPtrs[ static_cast<uint8_t>( PeripheralMode::DMA ) ]       = &Driver::writeDMA;
  }

  Driver::~Driver()
  {
    usartClassObjects[ channel ] = nullptr;
  }

  Chimera::Status_t Driver::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
  {
    using namespace Thor::LLD::USART;

    /*------------------------------------------------
    Make sure the channel is actually supported
    ------------------------------------------------*/
    auto iterator = ChanneltoInstance.find( channel );
    if ( !iterator )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    auto instance = iterator->second;
    this->channel = channel;
    resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( instance ) )->second;

    /*------------------------------------------------
    Create the hardware drivers
    ------------------------------------------------*/
    hwDriver = std::make_unique<Thor::LLD::USART::Driver>( instance );
    txPin = std::make_unique<Thor::GPIO::Driver>();
    rxPin = std::make_unique<Thor::GPIO::Driver>();

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

    usartClassObjects[ resourceIndex ] = this;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::begin( const Chimera::Hardware::PeripheralMode txMode,
                                       const Chimera::Hardware::PeripheralMode rxMode )
  {
    /*------------------------------------------------
    Initialize to the desired TX/RX modes
    ------------------------------------------------*/
    setMode( Chimera::Hardware::SubPeripheral::RX, rxMode );
    setMode( Chimera::Hardware::SubPeripheral::TX, txMode );

    /*------------------------------------------------
    Register the ISR post processor thread
    ------------------------------------------------*/
    if ( postProcessorThread[ resourceIndex ] )
    {
      postProcessorHandle[ resourceIndex ] = {};

      hwDriver->attachISRWakeup( &postProcessorSignal[ resourceIndex ] );

      Chimera::Threading::Thread thread;
      thread.initialize( postProcessorThread[ resourceIndex ], nullptr, Chimera::Threading::Priority::LEVEL_5, 500, "" );
      thread.start();
      postProcessorHandle[ resourceIndex ] = thread.native_handle();
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::end()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::configure( const Chimera::Serial::Config &config )
  {
    Thor::Driver::Serial::Config cfg = hwDriver->getConfiguration();

    /*------------------------------------------------
    Convert between the generalize Chimera options into 
    MCU specific register configuration settings. If these lookup
    tables fail, you might have something wrong with the mappings.
    ------------------------------------------------*/
    cfg.BaudRate   = static_cast<uint32_t>( config.baud );
    cfg.Mode       = Thor::LLD::USART::Configuration::Modes::TX_RX;
    cfg.Parity     = Thor::LLD::USART::ParityToRegConfig[ static_cast<size_t>( config.parity ) ];
    cfg.StopBits   = Thor::LLD::USART::StopBitsToRegConfig[ static_cast<size_t>( config.stopBits ) ];
    cfg.WordLength = Thor::LLD::USART::CharWidToRegConfig[ static_cast<size_t>( config.width ) ];

    return hwDriver->init( cfg );
  }

  Chimera::Status_t Driver::setBaud( const uint32_t baud )
  {
    auto currentConfig = hwDriver->getConfiguration();
    currentConfig.BaudRate = baud;

    return hwDriver->init( currentConfig );
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

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    auto error = Chimera::CommonStatusCodes::OK;
    auto iter  = static_cast<uint8_t>( txMode );

    if ( iter >= writeFuncPtrs.size() )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      error = ( this->*( writeFuncPtrs[ iter ] ) )( buffer, length, timeout_mS );
    }

    return error;
  }

  Chimera::Status_t Driver::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    auto error = Chimera::CommonStatusCodes::OK;
    auto iter  = static_cast<uint8_t>( rxMode );

    if ( iter >= readFuncPtrs.size() )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      error = ( this->*( readFuncPtrs[ iter ] ) )( buffer, length, timeout_mS );
    }

    return error;
  }

  Chimera::Status_t Driver::flush( const Chimera::Hardware::SubPeripheral periph )
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

  void Driver::postISRProcessing()
  {
    using namespace Thor::LLD::USART;

    const auto flags = hwDriver->getFlags();
    //auto event       = Chimera::Event::TRIGGER_INVALID;

    if ( flags & Runtime::Flag::TX_COMPLETE )
    {
      hwDriver->clearFlags( Runtime::Flag::TX_COMPLETE );
      //auto tcb = hwDriver->getTCB_TX();
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

      processListeners( Chimera::Event::TRIGGER_WRITE_COMPLETE );
    }

    if ( flags & Runtime::Flag::RX_COMPLETE )
    {
      hwDriver->clearFlags( Runtime::Flag::RX_COMPLETE );
      // auto tcb = hwDriver->getTCB_RX();

      /*------------------------------------------------
      Process Receive Buffers
      ------------------------------------------------*/
      // dump into queue?
      // how many bytes were read? Need copy of TCB

      /*------------------------------------------------
      Process Event Listeners. Semaphores unlocked in preparation
      for the listeners to possibly try and read more data.
      ------------------------------------------------*/
      awaitRXComplete.release();
      rxLock.release();

      processListeners( Chimera::Event::TRIGGER_READ_COMPLETE );
    }
  }

  Chimera::Status_t Driver::readAsync( uint8_t *const buffer, const size_t len )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !buffer )
    {
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
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
        error = Chimera::CommonStatusCodes::EMPTY;
      }
    }

    return error;
  }

  Chimera::Status_t Driver::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                 boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                                 const uint32_t hwBufferSize )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

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
      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    return error;
  }

  Chimera::Status_t Driver::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

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
//      error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
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

    if ( ( event != TRIGGER_READ_COMPLETE ) && ( event != TRIGGER_WRITE_COMPLETE ) )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    if ( ( event == TRIGGER_WRITE_COMPLETE ) && !awaitTXComplete.try_acquire_for( timeout ) )
    {
      return Chimera::CommonStatusCodes::TIMEOUT;
    }
    else if ( ( event == TRIGGER_READ_COMPLETE ) && !awaitRXComplete.try_acquire_for( timeout ) )
    {
      return Chimera::CommonStatusCodes::TIMEOUT;
    }

    awaitRXComplete.release();
    awaitTXComplete.release();

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, Chimera::Threading::BinarySemaphore &notifier,
                                       const size_t timeout )
  {
    auto result = await( event, timeout );

    if ( result == Chimera::CommonStatusCodes::OK )
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

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    for ( auto iterator = eventListeners.begin(); iterator != eventListeners.end(); iterator++ )
    {
      if ( iterator->id == registrationID )
      {
        eventListeners.erase( iterator );
        return Chimera::CommonStatusCodes::OK;
        break;
      }
    }

    return Chimera::CommonStatusCodes::NOT_FOUND;
  }

  void Driver::processListeners( const Chimera::Event::Trigger event )
  {
    Chimera::Event::notifyListenerList( event, eventListeners, 0 );
  }

  Chimera::Status_t Driver::readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::BUSY;

    if ( rxLock.try_acquire_for( 1000 ))
    {
      error = hwDriver->receive( buffer, length, timeout_mS );
    }

    return error;
  }

  Chimera::Status_t Driver::readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !rxBuffers.initialized() )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }


    if ( ( length <= rxBuffers.linearSize() ) && rxLock.try_acquire_for( 1000 ) )
    {
      memset( rxBuffers.linearBuffer(), 0, rxBuffers.linearSize() );

      error = hwDriver->receiveIT( rxBuffers.linearBuffer(), length, timeout_mS );

      if ( error == Chimera::CommonStatusCodes::OK )
      {
        error = Chimera::Serial::Status::RX_IN_PROGRESS;
      }
    }
    else
    {
      error = Chimera::CommonStatusCodes::MEMORY;
    }

    return error;
  }

  Chimera::Status_t Driver::readDMA( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !rxBuffers.initialized() )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }


    if ( ( length <= rxBuffers.linearSize() ) && rxLock.try_acquire_for( 1000 ) )
    { 

      memset( rxBuffers.linearBuffer(), 0, rxBuffers.linearSize() );
      error = hwDriver->receiveDMA( rxBuffers.linearBuffer(), length, timeout_mS );

      if ( error == Chimera::CommonStatusCodes::OK )
      {
        error = Chimera::Serial::Status::RX_IN_PROGRESS;
      }
    }
    else
    {
      error = Chimera::CommonStatusCodes::MEMORY;
    }

    return error;
  }

  Chimera::Status_t Driver::writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    using namespace Chimera::Threading;
    Chimera::Status_t error = Chimera::CommonStatusCodes::BUSY;

    if ( txLock.try_acquire_for( 1000 ) )
    {
      error = hwDriver->transmit( buffer, length, timeout_mS );
    }

    return error;
  }

  Chimera::Status_t Driver::writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !txBuffers.initialized() )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Hardware is free. Send the data directly. Otherwise
    queue everything up to send later.
    ------------------------------------------------*/
    if ( txLock.try_acquire() )
    {
      error = hwDriver->transmitIT( buffer, length, timeout_mS );
    }
    else
    {
      size_t pushed = 0;
      error = Chimera::CommonStatusCodes::BUSY;
      txBuffers.push( buffer, length, pushed );
    }

    return error;
  }

  Chimera::Status_t Driver::writeDMA( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !txBuffers.initialized() )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Hardware is free. Send the data directly. Otherwise
    queue everything up to send later.
    ------------------------------------------------*/
    if ( txLock.try_acquire() )
    {
      error = hwDriver->transmitDMA( buffer, length, timeout_mS );
    }
    else
    {
      size_t pushed = 0;
      error = Chimera::CommonStatusCodes::BUSY;
      txBuffers.push( buffer, length, pushed );
    }

    return error;
  }

}    // namespace Thor::USART


static void USART1ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::USART;
  static const auto resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( USART1_PERIPH ) )->second;

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto usart = usartClassObjects[ resourceIndex ]; usart )
    {
      usart->postISRProcessing();
    }
  }
}

static void USART2ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::USART;
  static const auto resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( USART2_PERIPH ) )->second;

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto usart = usartClassObjects[ resourceIndex ]; usart )
    {
      usart->postISRProcessing();
    }
  }
}

static void USART3ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::USART;
  static const auto resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( USART3_PERIPH ) )->second;

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto usart = usartClassObjects[ resourceIndex ]; usart )
    {
      usart->postISRProcessing();
    }
  }
}

static void USART6ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::USART;
  static const auto resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( USART6_PERIPH ) )->second;

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    postProcessorSignal[ resourceIndex ].acquire();
    if ( auto usart = usartClassObjects[ resourceIndex ]; usart )
    {
      usart->postISRProcessing();
    }
  }
}