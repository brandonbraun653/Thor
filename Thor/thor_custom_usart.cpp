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

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t USARTClass::end()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::configure( const Chimera::Serial::Config &config )
  {
    Thor::Driver::Serial::Config cfg;

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
    // This is a software level selection thing
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::write( const uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    hwDriver->transmitIT( buffer, length, timeout_mS );
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::read( uint8_t *const buffer, const size_t length, const uint32_t timeout_mS )
  {
    hwDriver->receiveIT( buffer, length, timeout_mS );
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::flush( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
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
      // Look at tx queue...more to transmit?? 
      // Sort into continuous block of memory first.

      /*------------------------------------------------
      Process Event Listeners
      ------------------------------------------------*/
      processListeners( Chimera::Event::Trigger::WRITE_COMPLETE );
      xSemaphoreGive( awaitEventTXComplete );
    }

    if ( flags & Runtime::Flag::RX_COMPLETE )
    {
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
      xSemaphoreGive( awaitEventRXComplete );
    }
  }

  Chimera::Status_t USARTClass::readAsync( uint8_t *const buffer, const size_t len )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::enableBuffering( const Chimera::Hardware::SubPeripheral periph,
                                                 boost::circular_buffer<uint8_t> *const userBuffer, uint8_t *const hwBuffer,
                                                 const uint32_t hwBufferSize )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::disableBuffering( const Chimera::Hardware::SubPeripheral periph )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  bool USARTClass::available( size_t *const bytes )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
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