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
static std::array<Thor::USART::USARTClass *, USARTDriver::NUM_USART_PERIPHS> usartObjects;

/* Post Processor Thread Handles */
static std::array<TaskHandle_t, USARTDriver::NUM_USART_PERIPHS> postProcessorHandle;

/* Post Processor Thread Wakeup Signals */
static std::array<SemaphoreHandle_t, USARTDriver::NUM_USART_PERIPHS> postProcessorSignal;

/* Post Processor Thread Function Pointers */
static std::array<Chimera::Function::void_func_void_ptr, USARTDriver::NUM_USART_PERIPHS> postProcessorThread = {
  /* clang-format off */
  USART1ISRPostProcessorThread, 
  USART2ISRPostProcessorThread, 
  USART3ISRPostProcessorThread,
  USART6ISRPostProcessorThread
  /* clang-format on */
};


namespace Thor::USART
{
  USARTClass::USARTClass() : resourceIndex( 0 ), channel( 0 ), listenerIDCount( 0 )
  {
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

    cfg.BaudRate   = 115200;
    cfg.Mode       = USARTDriver::Configuration::Modes::TX_RX;
    cfg.Parity     = USARTDriver::Configuration::Parity::NONE;
    cfg.StopBits   = USARTDriver::Configuration::Stop::BIT_1;
    cfg.WordLength = USARTDriver::Configuration::WordLength::LEN_8BIT;

    hwDriver->init( cfg );

    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::setBaud( const uint32_t baud )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t USARTClass::setMode( const Chimera::Hardware::SubPeripheral periph,
                                         const Chimera::Hardware::SubPeripheralMode mode )
  {
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
    const auto event = Chimera::Event::Trigger::WRITE_COMPLETE;

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

  void USARTClass::await( const Chimera::Event::Trigger event )
  {
  }

  void USARTClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier )
  {
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