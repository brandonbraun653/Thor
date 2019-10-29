/********************************************************************************
 *   File Name:
 *    thor_custom_spi.cpp
 *
 *   Description:
 *    SPI driver for Thor
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>
#include <limits>

/* Chimera Includes */
#include <Chimera/constants/common.hpp>
#include <Chimera/threading.hpp>
#include <Chimera/interface/spi_intf.hpp>

/* Thor Includes */
#include <Thor/gpio.hpp>
#include <Thor/spi.hpp>

#include <Thor/drivers/common/types/spi_types.hpp>


#if defined( THOR_CUSTOM_DRIVERS ) && ( THOR_DRIVER_SPI == 1 )


static std::array<Thor::SPI::SPIClass *, Thor::Driver::SPI::NUM_SPI_PERIPHS> SPIClassObjects;
static std::array<TaskHandle_t, Thor::Driver::SPI::NUM_SPI_PERIPHS> postProcessorHandles;
static std::array<SemaphoreHandle_t, Thor::Driver::SPI::NUM_SPI_PERIPHS> postProcessorSignals;


#if defined( STM32_SPI1_PERIPH_AVAILABLE )
static void SPI1ISRPostProcessorThread( void *argument );
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
static void SPI2ISRPostProcessorThread( void *argument );
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
static void SPI3ISRPostProcessorThread( void *argument );
#endif
#if defined( STM32_SPI4_PERIPH_AVAILABLE )
static void SPI4ISRPostProcessorThread( void *argument );
#endif


namespace Chimera::SPI
{
  Chimera::Status_t initialize()
  {
    return Thor::SPI::initialize();
  }
}    // namespace Chimera::SPI


namespace Thor::SPI
{
  using namespace Chimera::Threading;

  static size_t s_driver_initialized;


  static void destroyClassObject( const size_t channel )
  {
    if ( ( channel < SPIClassObjects.size() ) && SPIClassObjects[ channel ] )
    {
      SPIClassObjects[ channel ]->deInit();
      vPortFree( SPIClassObjects[ channel ] );
    }

    SPIClassObjects[ channel ] = nullptr;
  }

  static void destroyThreadHandle( const size_t channel )
  {
    if ( ( channel < postProcessorHandles.size() ) && postProcessorHandles[ channel ] )
    {
      vTaskDelete( postProcessorHandles[ channel ] );
      vPortFree( postProcessorHandles[ channel ] );
    }

    postProcessorHandles[ channel ] = nullptr;
  }

  static void destroyThreadSemaphore( const size_t channel )
  {
    if ( ( channel < postProcessorSignals.size() ) && postProcessorSignals[ channel ] )
    {
      vPortFree( postProcessorSignals[ channel ] );
    }

    postProcessorSignals[ channel ] = nullptr;
  }

  Chimera::Status_t initialize()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::Driver::SPI::initialize();

    /*------------------------------------------------
    Reset driver object memory
    ------------------------------------------------*/
    for ( size_t x = 0; x < SPIClassObjects.size(); x++ )
    {
      destroyClassObject( x );
    }

    /*------------------------------------------------
    Reset thread handle memory
    ------------------------------------------------*/
    for ( size_t x = 0; x < postProcessorHandles.size(); x++ )
    {
      destroyThreadHandle( x );
    }

    /*------------------------------------------------
    Reset semaphore signal memory
    ------------------------------------------------*/
    for ( size_t x = 0; x < postProcessorSignals.size(); x++ )
    {
      destroyThreadSemaphore( x );
    }

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }

  SPIClass::SPIClass()
  {
    /*------------------------------------------------
    Lazy initialize all driver memory in case the user forgot
    ------------------------------------------------*/
    if ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY )
    {
      initialize();
    }

    /*------------------------------------------------
    Default initialize class member variables
    ------------------------------------------------*/
    memset( &config, 0, sizeof( config ) );
  }

  SPIClass::~SPIClass()
  {
  }

  /*------------------------------------------------
  HW Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::init( const Chimera::SPI::DriverConfig &setupStruct )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;
    auto lockGuard           = LockGuard(*this);

    /*------------------------------------------------
    Should we even bother creating this?
    ------------------------------------------------*/
    if ( !Thor::Driver::SPI::isChannelSupported( setupStruct.HWInit.hwChannel ) )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }

    if ( !lockGuard.lock() )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    First register the driver and initialize class vars
    ------------------------------------------------*/
    auto instance      = Thor::Driver::SPI::ChannelToInstance[ setupStruct.HWInit.hwChannel ];
    auto resourceIndex = Thor::Driver::SPI::InstanceToResourceIndex[ reinterpret_cast<std::uintptr_t>( instance ) ];

    SPIClassObjects[ resourceIndex ] = this;

    /*------------------------------------------------
    Configure the GPIO
    ------------------------------------------------*/
    config = setupStruct;

    SCK  = std::make_unique<Thor::GPIO::GPIOClass>();
    MOSI = std::make_unique<Thor::GPIO::GPIOClass>();
    MISO = std::make_unique<Thor::GPIO::GPIOClass>();
    CS   = std::make_unique<Thor::GPIO::GPIOClass>();

    result |= SCK->init( config.SCKInit );
    result |= MOSI->init( config.MOSIInit );
    result |= MISO->init( config.MISOInit );
    result |= CS->init( config.CSInit );

    if ( result != Chimera::CommonStatusCodes::OK )
    {
      return Chimera::CommonStatusCodes::FAILED_INIT;
    }

    /* Make sure we aren't selecting a device by accident */
    CS->setState( Chimera::GPIO::State::HIGH );

    /*------------------------------------------------
    Configure the SPI hardware
    ------------------------------------------------*/
    driver = std::make_unique<Thor::Driver::SPI::Driver>();
    result |= driver->attach( instance );
    result |= driver->configure( config );
    result |= driver->registerConfig( &config );

    if ( result != Chimera::CommonStatusCodes::OK )
    {
      config.validity = false;
    }

    return result;
  }

  Chimera::SPI::DriverConfig SPIClass::getInit()
  {
    return config;
  }

  Chimera::Status_t SPIClass::deInit()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::setChipSelect( const Chimera::GPIO::State value )
  {
    if ( LockGuard( *this ).lock() && CS )
    {
      return CS->setState( value );
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t SPIClass::setChipSelectControlMode( const Chimera::SPI::CSMode mode )
  {
    if ( LockGuard( *this ).lock() )
    {
      config.HWInit.csMode = mode;
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t SPIClass::writeBytes( const void *const txBuffer, const size_t length, const size_t timeoutMS )
  {
    return readWriteBytes( txBuffer, nullptr, length, timeoutMS );
  }

  Chimera::Status_t SPIClass::readBytes( void *const rxBuffer, const size_t length, const size_t timeoutMS )
  {
    return readWriteBytes( nullptr, rxBuffer, length, timeoutMS );
  }

  Chimera::Status_t SPIClass::readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length,
                                              const size_t timeoutMS )
  {
    auto lockguard = LockGuard( *this );
    auto result    = Chimera::CommonStatusCodes::OK;

    /*------------------------------------------------
    Input protection & resource acquisition
    ------------------------------------------------*/
    if ( !lockguard.lock( timeoutMS ) )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }
    else if ( ( !txBuffer && !rxBuffer ) || !length || !driver )
    {
      return Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }

    /*------------------------------------------------
    Handle the chip select controller behavior
    ------------------------------------------------*/
    if ( config.HWInit.csMode != Chimera::SPI::CSMode::MANUAL ) 
    {
      CS->setState( Chimera::GPIO::State::LOW );
    }

    /*------------------------------------------------
    Call the proper transfer method
    ------------------------------------------------*/
    switch ( config.HWInit.txfrMode )
    {
      case Chimera::SPI::TransferMode::BLOCKING:
        result = driver->transfer( txBuffer, rxBuffer, length );

        if ( config.HWInit.csMode != Chimera::SPI::CSMode::MANUAL )
        {
          CS->setState( Chimera::GPIO::State::HIGH );
        }

        return result;
        break;

      case Chimera::SPI::TransferMode::INTERRUPT:
        return driver->transferIT( txBuffer, rxBuffer, length );
        break;

      case Chimera::SPI::TransferMode::DMA:
        return driver->transferDMA( txBuffer, rxBuffer, length );
        break;

      default:
        /*------------------------------------------------
        Disable the chip select for anything other than manual control
        ------------------------------------------------*/
        if ( config.HWInit.csMode != Chimera::SPI::CSMode::MANUAL )
        {
          CS->setState( Chimera::GPIO::State::HIGH );
        }

        return Chimera::CommonStatusCodes::FAIL;
        break;
    }
  }

  Chimera::Status_t SPIClass::setPeripheralMode( const Chimera::Hardware::PeripheralMode mode )
  {
    if ( LockGuard( *this ).lock() )
    {
      config.HWInit.txfrMode = mode;
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t SPIClass::setClockFrequency( const size_t freq, const size_t tolerance )
  {
    /*------------------------------------------------
    Acquire resources
    ------------------------------------------------*/
    auto lockguard = LockGuard( *this );
    if ( !lockguard.lock() )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    Input protection
    ------------------------------------------------*/
    if ( !driver || !freq )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Assign the new clock frequency and re-initialize the driver
    ------------------------------------------------*/
    config.HWInit.clockFreq = freq;
    return driver->configure( config );
  }

  size_t SPIClass::getClockFrequency()
  {
    /* Shouldn't need a lock as this is atomic on 32-bit ARM processors */
    return config.HWInit.clockFreq;
  }

  /*------------------------------------------------
  Async IO Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::await( const Chimera::Event::Trigger event, SemaphoreHandle_t notifier, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Listener Interface
  ------------------------------------------------*/
  Chimera::Status_t SPIClass::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                                size_t &registrationID )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t SPIClass::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}    // namespace Thor::SPI

#if defined( STM32_SPI1_PERIPH_AVAILABLE )
static void SPI1ISRPostProcessorThread( void *argument )
{
}
#endif

#if defined( STM32_SPI2_PERIPH_AVAILABLE )
static void SPI2ISRPostProcessorThread( void *argument )
{
}
#endif

#if defined( STM32_SPI3_PERIPH_AVAILABLE )
static void SPI3ISRPostProcessorThread( void *argument )
{
}
#endif

#if defined( STM32_SPI4_PERIPH_AVAILABLE )
static void SPI4ISRPostProcessorThread( void *argument )
{
}
#endif

#endif /* THOR_CUSTOM_DRIVERS && THOR_DRIVER_SPI */