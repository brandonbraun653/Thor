/********************************************************************************
 *   File Name:
 *    thor_custom_usart.cpp
 *
 *   Description:
 *    Implements the custom driver variant of the Thor USART interface.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/threading.hpp>

/* Thor Includes */
#include <Thor/event.hpp>
#include <Thor/usart.hpp>
#include <Thor/drivers/Usart.hpp>
#include <Thor/defaults/serial_defaults.hpp>

namespace USARTDriver = Thor::Driver::USART;

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
static std::array<Thor::USART::USARTClass *, USARTDriver::NUM_USART_PERIPHS> usartObjects = { 
  nullptr,
  nullptr,
  nullptr,
  nullptr 
};

/* Post Processor Thread Handles */
static std::array<TaskHandle_t, USARTDriver::NUM_USART_PERIPHS> postProcessorHandle = { 
  nullptr,  
  nullptr, 
  nullptr, 
  nullptr 
};

/* Post Processor Thread Wakeup Signals */
static std::array<SemaphoreHandle_t, USARTDriver::NUM_USART_PERIPHS> postProcessorSignal = { 
  nullptr, 
  nullptr, 
  nullptr,
  nullptr 
};

/* Post Processor Thread Function Pointers */
static std::array<Chimera::Function::void_func_void_ptr, USARTDriver::NUM_USART_PERIPHS> postProcessorThread = {
  USART1ISRPostProcessorThread, 
  USART2ISRPostProcessorThread, 
  USART3ISRPostProcessorThread,
  USART6ISRPostProcessorThread
};
/* clang-format on */

namespace Thor::USART
{
  USARTClass::USARTClass() : resourceIndex( 0 ), channel( 0 ), listenerIDCount( 0 )
  {
    using namespace Chimera::Hardware;

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

    awaitEventRXComplete = xSemaphoreCreateBinary();
    awaitEventTXComplete = xSemaphoreCreateBinary();
  }

  USARTClass::~USARTClass()
  {
    usartObjects[ channel ] = nullptr;
  }

  Chimera::Status_t USARTClass::assignHW( const uint8_t channel, const Chimera::Serial::IOPins &pins )
  {
    using namespace Thor::Driver::USART;

    /*------------------------------------------------
    Make sure the channel is actually supported
    ------------------------------------------------*/
    auto iterator = ChanneltoInstance.find( channel );
    if ( iterator == ChanneltoInstance.end() )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    auto instance = iterator->second;
    this->channel = channel;
    resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( instance ) )->second;

    /*------------------------------------------------
    Create the hardware drivers
    ------------------------------------------------*/
    hwDriver = std::make_unique<USARTDriver::Driver>( instance );
    txPin = std::make_unique<Thor::GPIO::GPIOClass>();
    rxPin = std::make_unique<Thor::GPIO::GPIOClass>();

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

    usartObjects[ resourceIndex ] = this;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t USARTClass::begin( const Chimera::Hardware::SubPeripheralMode txMode,
                                       const Chimera::Hardware::SubPeripheralMode rxMode )
  {
    setMode( Chimera::Hardware::SubPeripheral::RX, rxMode );
    setMode( Chimera::Hardware::SubPeripheral::TX, txMode );

    /*------------------------------------------------
    Register the ISR post processor thread
    ------------------------------------------------*/
    if ( postProcessorThread[ resourceIndex ] )
    {
      postProcessorSignal[ resourceIndex ] = xSemaphoreCreateBinary();
      postProcessorHandle[ resourceIndex ] = nullptr;

      hwDriver->attachISRWakeup( postProcessorSignal[ resourceIndex ] );

      Chimera::Threading::addThread( postProcessorThread[ resourceIndex ], "", 500, NULL, 5,
                                     &postProcessorHandle[ resourceIndex ] );
    }

    xSemaphoreGive( awaitEventRXComplete );
    xSemaphoreGive( awaitEventTXComplete );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t USARTClass::end()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::configure( const Chimera::Serial::Config &config )
  {
    Thor::Driver::Serial::Config cfg = hwDriver->getConfiguration();

    /*------------------------------------------------
    Config settings that don't need a lookup table
    ------------------------------------------------*/
    cfg.BaudRate = config.baud;
    cfg.Mode     = USARTDriver::Configuration::Modes::TX_RX;

    /*------------------------------------------------
    Configure the parity register settings 
    ------------------------------------------------*/
    auto parityIterator = USARTDriver::ParityToRegConfig.find( config.parity );
    if ( parityIterator != USARTDriver::ParityToRegConfig.end() )
    {
      cfg.Parity = parityIterator->second;
    }
    else
    {
      cfg.Parity = USARTDriver::Configuration::Parity::NONE;
    }

    /*------------------------------------------------
    Configure the stop bits register settings
    ------------------------------------------------*/
    auto stopIterator = USARTDriver::StopBitsToRegConfig.find( config.stopBits );
    if ( stopIterator != USARTDriver::StopBitsToRegConfig.end() )
    {
      cfg.StopBits = stopIterator->second;
    }
    else
    {
      cfg.StopBits = USARTDriver::Configuration::Stop::BIT_1;
    }

    /*------------------------------------------------
    Configure the word length register settings
    ------------------------------------------------*/
    auto wordIterator = USARTDriver::CharWidToRegConfig.find( config.width );
    if ( wordIterator != USARTDriver::CharWidToRegConfig.end() )
    {
      cfg.WordLength = wordIterator->second;
    }
    else
    {
      cfg.WordLength = USARTDriver::Configuration::WordLength::LEN_8BIT;
    }

    cfg.BaudRate   = 115200;
    cfg.Mode       = USARTDriver::Configuration::Modes::TX_RX;
    cfg.Parity     = USARTDriver::Configuration::Parity::NONE;
    cfg.StopBits   = USARTDriver::Configuration::Stop::BIT_1;
    cfg.WordLength = USARTDriver::Configuration::WordLength::LEN_8BIT;

    return hwDriver->init( cfg );
  }

  Chimera::Status_t USARTClass::setBaud( const uint32_t baud )
  {
    auto currentConfig = hwDriver->getConfiguration();
    currentConfig.BaudRate = baud;

    return hwDriver->init( currentConfig );
  }

  Chimera::Status_t USARTClass::setMode( const Chimera::Hardware::SubPeripheral periph,
                                         const Chimera::Hardware::SubPeripheralMode mode )
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

  Chimera::Status_t USARTClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    auto error = Chimera::CommonStatusCodes::OK;
    auto iter  = static_cast<uint8_t>( txMode );

    //    if ( !PeripheralState.gpio_enabled || !PeripheralState.configured )
    //    {
    //      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    //    }
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

  Chimera::Status_t USARTClass::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    auto error = Chimera::CommonStatusCodes::OK;
    auto iter  = static_cast<uint8_t>( rxMode );

    //    if ( !PeripheralState.gpio_enabled || !PeripheralState.configured )
    //    {
    //      error = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    //    }
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
    using namespace USARTDriver;

    const auto flags = hwDriver->getFlags();
    auto event       = Chimera::Event::Trigger::INVALID;

    if ( flags & Runtime::Flag::TX_COMPLETE )
    {
      hwDriver->clearFlags( Runtime::Flag::TX_COMPLETE );
      auto tcb = hwDriver->getTCB_TX();

      /*------------------------------------------------
      Process Transmit Buffers
      ------------------------------------------------*/
      xSemaphoreGive( awaitEventTXComplete );

      if ( !txBuffers.external->empty() )
      {
        size_t bytesToTransmit = txBuffers.copyToHWBuffer( txBuffers.external->size() );
        write( txBuffers.getHWBuffer(), bytesToTransmit );
      }

      /*------------------------------------------------
      Process Event Listeners
      ------------------------------------------------*/
      processListeners( Chimera::Event::Trigger::WRITE_COMPLETE );
    }

    if ( flags & Runtime::Flag::RX_COMPLETE )
    {
      xSemaphoreGive( awaitEventRXComplete );
      hwDriver->clearFlags( Runtime::Flag::RX_COMPLETE );
      auto tcb = hwDriver->getTCB_RX();

      /*------------------------------------------------
      Process Receive Buffers
      ------------------------------------------------*/
      // dump into queue?
      // how many bytes were read? Need copy of TCB

      /*------------------------------------------------
      Process Event Listeners
      ------------------------------------------------*/
      processListeners( Chimera::Event::Trigger::READ_COMPLETE );
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

  Chimera::Status_t USARTClass::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
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

  Chimera::Status_t USARTClass::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    using namespace Chimera::Event;

    if ( ( event != Trigger::READ_COMPLETE ) || ( event != Trigger::WRITE_COMPLETE ) )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    if ( ( event == Trigger::WRITE_COMPLETE ) &&
         ( xSemaphoreTake( awaitEventTXComplete, pdMS_TO_TICKS( timeout ) ) != pdPASS ) )
    {
      return Chimera::CommonStatusCodes::TIMEOUT;
    }
    else if ( ( event == Trigger::READ_COMPLETE ) &&
              ( xSemaphoreTake( awaitEventRXComplete, pdMS_TO_TICKS( timeout ) ) != pdPASS ) )
    {
      return Chimera::CommonStatusCodes::TIMEOUT;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t USARTClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier, const size_t timeout )
  {
    auto result = await( event, timeout );

    if ( result == Chimera::CommonStatusCodes::OK )
    {
      xSemaphoreGive( notifier );
    }

    return result;
  }

  Chimera::Status_t USARTClass::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                                  size_t &registrationID )
  {
    registrationID = ++listenerIDCount;
    listener.id    = registrationID;

    eventListeners.push_back( listener );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t USARTClass::removeListener( const size_t registrationID, const size_t timeout )
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

  void USARTClass::processListeners( const Chimera::Event::Trigger event )
  {
    for ( auto &listener : eventListeners )
    {
      if ( listener.trigger != event )
      {
        continue;
      }

      Thor::Event::notifyAtomic( event, listener, static_cast<uint32_t>( event ) );
      Thor::Event::notifyThread( event, listener );
      Thor::Event::executeISRCallback( event, listener, nullptr, 0 );
    }
  }

  Chimera::Status_t USARTClass::readBlocking( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::BUSY;

    if ( xSemaphoreTake( awaitEventRXComplete, Chimera::Threading::TIMEOUT_DONT_WAIT ) == pdPASS )
    {
      error = hwDriver->receive( buffer, length, timeout_mS );
      xSemaphoreGive( awaitEventRXComplete );
    }

    return error;
  }

  Chimera::Status_t USARTClass::readInterrupt( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !rxBuffers.initialized() )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }


    if ( ( length <= rxBuffers.internalSize ) &&
         ( xSemaphoreTake( awaitEventRXComplete, Chimera::Threading::TIMEOUT_DONT_WAIT ) == pdPASS ) )
    {
      memset( rxBuffers.internal, 0, rxBuffers.internalSize );

      error = hwDriver->receiveIT( rxBuffers.internal, length, timeout_mS );

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

  Chimera::Status_t USARTClass::readDMA( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

    if ( !rxBuffers.initialized() )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }


    if ( ( length <= rxBuffers.internalSize ) &&
         ( xSemaphoreTake( awaitEventRXComplete, Chimera::Threading::TIMEOUT_DONT_WAIT ) == pdPASS ) )
    { 

      memset( rxBuffers.internal, 0, rxBuffers.internalSize );
      error = hwDriver->receiveDMA( rxBuffers.internal, length, timeout_mS );

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

  Chimera::Status_t USARTClass::writeBlocking( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    Chimera::Status_t error = Chimera::CommonStatusCodes::BUSY;

    if ( xSemaphoreTake( awaitEventTXComplete, Chimera::Threading::TIMEOUT_DONT_WAIT ) == pdPASS )
    {
      error = hwDriver->transmit( buffer, length, timeout_mS );
      xSemaphoreGive( awaitEventTXComplete );
    }

    return error;
  }

  Chimera::Status_t USARTClass::writeInterrupt( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
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
    if ( xSemaphoreTake( awaitEventTXComplete, Chimera::Threading::TIMEOUT_DONT_WAIT ) == pdPASS )
    {
      error = hwDriver->transmitIT( buffer, length, timeout_mS );
    }
    else
    {
      error = Chimera::CommonStatusCodes::BUSY;
      txBuffers.push( buffer, length );
    }

    return error;
  }

  Chimera::Status_t USARTClass::writeDMA( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
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
    if ( xSemaphoreTake( awaitEventTXComplete, Chimera::Threading::TIMEOUT_DONT_WAIT ) == pdPASS )
    {
      error = hwDriver->transmitDMA( buffer, length, timeout_mS );
    }
    else
    {
      error = Chimera::CommonStatusCodes::BUSY;
      txBuffers.push( buffer, length );
    }

    return error;
  }

}    // namespace Thor::USART


static void USART1ISRPostProcessorThread( void *argument )
{
  using namespace Thor::Driver::USART;
  static const auto resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( USART1_PERIPH ) )->second;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto usart = usartObjects[ resourceIndex ]; usart )
      {
        usart->postISRProcessing();
      }
    }
  }
}

static void USART2ISRPostProcessorThread( void *argument )
{
  using namespace Thor::Driver::USART;
  static const auto resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( USART2_PERIPH ) )->second;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto usart = usartObjects[ resourceIndex ]; usart )
      {
        usart->postISRProcessing();
      }
    }
  }
}

static void USART3ISRPostProcessorThread( void *argument )
{
  using namespace Thor::Driver::USART;
  static const auto resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( USART3_PERIPH ) )->second;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto usart = usartObjects[ resourceIndex ]; usart )
      {
        usart->postISRProcessing();
      }
    }
  }
}

static void USART6ISRPostProcessorThread( void *argument )
{
  using namespace Thor::Driver::USART;
  static const auto resourceIndex = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( USART6_PERIPH ) )->second;

  Chimera::Threading::signalSetupComplete();

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto usart = usartObjects[ resourceIndex ]; usart )
      {
        usart->postISRProcessing();
      }
    }
  }
}