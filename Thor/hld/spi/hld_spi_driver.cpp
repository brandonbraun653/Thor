/********************************************************************************
 *  File Name:
 *    hld_spi_driver.cpp
 *
 *  Description:
 *    SPI driver for Thor
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>
#include <limits>

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/spi>
#include <Chimera/thread>
#include <Chimera/event>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/gpio>
#include <Thor/spi>
#include <Thor/lld/interface/spi/spi_detail.hpp>
#include <Thor/lld/interface/spi/spi_intf.hpp>

#if defined( THOR_HLD_SPI )

static std::array<Thor::SPI::Driver *, Thor::LLD::SPI::NUM_SPI_PERIPHS> s_hld_drivers;
static std::array<Chimera::Threading::detail::native_thread_handle_type, Thor::LLD::SPI::NUM_SPI_PERIPHS> s_user_isr_handle;
static std::array<Chimera::Threading::BinarySemaphore, Thor::LLD::SPI::NUM_SPI_PERIPHS> s_user_isr_signal;
static std::array<Chimera::Function::void_func_void_ptr, Thor::LLD::SPI::NUM_SPI_PERIPHS> s_user_isr_thread_func;

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

namespace Thor::SPI
{
  using namespace Chimera::Threading;

  static size_t s_driver_initialized;
  static std::array<Thor::LLD::SPI::IDriver_sPtr, Thor::LLD::SPI::NUM_SPI_PERIPHS> s_lld_drivers;

  Chimera::Status_t initialize()
  {
    /*------------------------------------------------
    Prevent multiple initializations (need reset first)
    ------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::CommonStatusCodes::OK;
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::SPI::initialize();

    /*------------------------------------------------
    Initialize driver object memory
    ------------------------------------------------*/
    s_lld_drivers.fill( nullptr );

#if defined( STM32_SPI1_PERIPH_AVAILABLE )
    s_user_isr_thread_func[ Thor::LLD::SPI::SPI1_RESOURCE_INDEX ] = SPI1ISRPostProcessorThread;
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
    s_user_isr_thread_func[ Thor::LLD::SPI::SPI2_RESOURCE_INDEX ] = SPI2ISRPostProcessorThread;
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
    s_user_isr_thread_func[ Thor::LLD::SPI::SPI3_RESOURCE_INDEX ] = SPI3ISRPostProcessorThread;
#endif
#if defined( STM32_SPI4_PERIPH_AVAILABLE )
    s_user_isr_thread_func[ Thor::LLD::SPI::SPI4_RESOURCE_INDEX ] = SPI4ISRPostProcessorThread;
#endif

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }

  /*------------------------------------------------
  Class Specific Functions
  ------------------------------------------------*/
  Driver::Driver()
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
    config = {};
  }

  Driver::~Driver()
  {
  }

  void Driver::postISRProcessing()
  {
    // TODO: Add logic for listener invocation, error handling, CS toggling, and multiple transfers.

    /*------------------------------------------------
    Decide chip select behavior
    ------------------------------------------------*/
    if ( config.HWInit.csMode == Chimera::SPI::CSMode::AUTO_AFTER_TRANSFER )
    {
      setChipSelect( Chimera::GPIO::State::HIGH );
    }

    /*------------------------------------------------
    Notifiy threads waiting on the transfer complete signal
    ------------------------------------------------*/
    awaitTransferComplete.release();
  }

  /*------------------------------------------------
  HW Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::init( const Chimera::SPI::DriverConfig &setupStruct )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;
    auto lockGuard           = TimedLockGuard( *this );

    /*------------------------------------------------
    Should we even bother creating this?
    ------------------------------------------------*/
    if ( !Thor::LLD::SPI::isChannelSupported( setupStruct.HWInit.hwChannel ) )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }
    else if ( !lockGuard.try_lock_for( 100 ) )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    First register the driver and initialize class vars
    ------------------------------------------------*/
    auto instance    = Thor::LLD::SPI::ChannelToInstance[ setupStruct.HWInit.hwChannel ];
    lldResourceIndex = Thor::LLD::SPI::InstanceToResourceIndex[ reinterpret_cast<std::uintptr_t>( instance ) ];

    s_hld_drivers[ lldResourceIndex ] = this;
    s_lld_drivers[ lldResourceIndex ]  = Thor::LLD::SPI::getDriver( setupStruct.HWInit.hwChannel );

    /*------------------------------------------------
    Configure the GPIO
    ------------------------------------------------*/
    config = setupStruct;

    SCK  = std::make_unique<Thor::GPIO::Driver>();
    MOSI = std::make_unique<Thor::GPIO::Driver>();
    MISO = std::make_unique<Thor::GPIO::Driver>();

    result |= SCK->init( config.SCKInit, Chimera::Threading::TIMEOUT_DONT_WAIT );
    result |= MOSI->init( config.MOSIInit, Chimera::Threading::TIMEOUT_DONT_WAIT );
    result |= MISO->init( config.MISOInit, Chimera::Threading::TIMEOUT_DONT_WAIT );

    /* Does the driver take control of the CS pin? */
    if ( !setupStruct.externalCS )
    {
      CS = std::make_shared<Thor::GPIO::Driver>();
      result |= CS->init( config.CSInit, 100 );
    }
    else if ( !CS )
    {
      /* User hasn't attached the ChipSelect shared_ptr object yet */
      return Chimera::CommonStatusCodes::FAILED_INIT;
    }
    else
    {
      /* Disable the CS and force external control behavior */
      CS.reset();
      config.HWInit.csMode = Chimera::SPI::CSMode::MANUAL;
    }

    if ( result != Chimera::CommonStatusCodes::OK )
    {
      return Chimera::CommonStatusCodes::FAILED_INIT;
    }

    /* Make sure we aren't selecting a device by accident */
    setChipSelect( Chimera::GPIO::State::HIGH );

    /*------------------------------------------------
    Configure the SPI hardware
    ------------------------------------------------*/
    auto driver = s_lld_drivers[ lldResourceIndex ];
    result |= driver->attach( instance );
    result |= driver->configure( config );
    result |= driver->registerConfig( &config );

    if ( result != Chimera::CommonStatusCodes::OK )
    {
      config.validity = false;
    }

    /*------------------------------------------------
    Register the ISR post processor thread
    ------------------------------------------------*/
    if ( s_user_isr_thread_func[ lldResourceIndex ] )
    {
      // Yeah this is gonna be bad if someone re-initializes the SPI driver....
      // postProcessorHandles[ lldResourceIndex ] = nullptr;

      driver->attachISRWakeup( &s_user_isr_signal[ lldResourceIndex ] );

      Chimera::Threading::Thread thread;
      thread.initialize( s_user_isr_thread_func[ lldResourceIndex ], nullptr, Chimera::Threading::Priority::LEVEL_5, 500, "" );
      thread.start();
      s_user_isr_handle[ lldResourceIndex ] = thread.native_handle();
    }

    return result;
  }

  Chimera::SPI::DriverConfig Driver::getInit()
  {
    return config;
  }

  Chimera::Status_t Driver::deInit()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::setChipSelect( const Chimera::GPIO::State value )
  {
    if ( Chimera::Threading::TimedLockGuard( *this ).try_lock_for( 10 ) && CS )
    {
      return CS->setState( value, 100 );
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t Driver::setChipSelectControlMode( const Chimera::SPI::CSMode mode )
  {
    /*------------------------------------------------
    Only valid if the SPI driver has control of the chip select
    ------------------------------------------------*/
    if ( Chimera::Threading::TimedLockGuard( *this ).try_lock_for( 10 ) && !config.externalCS )
    {
      config.HWInit.csMode = mode;
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t Driver::writeBytes( const void *const txBuffer, const size_t length, const size_t timeoutMS )
  {
    return readWriteBytes( txBuffer, nullptr, length, timeoutMS );
  }

  Chimera::Status_t Driver::readBytes( void *const rxBuffer, const size_t length, const size_t timeoutMS )
  {
    return readWriteBytes( nullptr, rxBuffer, length, timeoutMS );
  }

  Chimera::Status_t Driver::readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length,
                                            const size_t timeoutMS )
  {
    auto lockguard = TimedLockGuard( *this );
    auto result    = Chimera::CommonStatusCodes::OK;
    auto driver    = s_lld_drivers[ lldResourceIndex ];

    /*------------------------------------------------
    Input protection & resource acquisition
    ------------------------------------------------*/
    if ( !lockguard.try_lock_for( timeoutMS ) )
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
      setChipSelect( Chimera::GPIO::State::LOW );
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
          setChipSelect( Chimera::GPIO::State::HIGH );
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
          CS->setState( Chimera::GPIO::State::HIGH, 100 );
        }

        return Chimera::CommonStatusCodes::FAIL;
        break;
    }
  }

  Chimera::Status_t Driver::setPeripheralMode( const Chimera::Hardware::PeripheralMode mode )
  {
    if ( Chimera::Threading::TimedLockGuard( *this ).try_lock_for( 10 ) )
    {
      config.HWInit.txfrMode = mode;
      return Chimera::CommonStatusCodes::OK;
    }

    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t Driver::setClockFrequency( const size_t freq, const size_t tolerance )
  {
    auto lockguard = TimedLockGuard( *this );
    auto driver    = s_lld_drivers[ lldResourceIndex ];

    /*------------------------------------------------
    Input protection
    ------------------------------------------------*/
    if ( !lockguard.try_lock_for( 10 ) )
    {
      return Chimera::CommonStatusCodes::LOCKED;
    }
    else if ( !driver || !freq )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Assign the new clock frequency and re-initialize the driver
    ------------------------------------------------*/
    config.HWInit.clockFreq = freq;
    return driver->configure( config );
  }

  size_t Driver::getClockFrequency()
  {
    /* Shouldn't need a lock as this is atomic on 32-bit ARM processors */
    return config.HWInit.clockFreq;
  }

  /*------------------------------------------------
  Async IO Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    if ( event != Chimera::Event::TRIGGER_TRANSFER_COMPLETE )
    {
      return Chimera::CommonStatusCodes::NOT_SUPPORTED;
    }
    else if ( awaitTransferComplete.try_acquire_for( timeout ) )
    {
      return Chimera::CommonStatusCodes::TIMEOUT;
    }
    else
    {
      return Chimera::CommonStatusCodes::OK;
    }
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

  /*------------------------------------------------
  Listener Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

}    // namespace Thor::SPI


#if defined( STM32_SPI1_PERIPH_AVAILABLE )
static void SPI1ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::SPI;
  constexpr auto index = SPI1_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    if ( auto spi = s_hld_drivers[ index ]; spi )
    {
      spi->postISRProcessing();
    }
  }
}
#endif

#if defined( STM32_SPI2_PERIPH_AVAILABLE )
static void SPI2ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::SPI;
  constexpr auto index = SPI2_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignals[ index ].acquire();
    if ( auto spi = SPIClassObjects[ index ]; spi )
    {
      spi->postISRProcessing();
    }
  }
}
#endif

#if defined( STM32_SPI3_PERIPH_AVAILABLE )
static void SPI3ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::SPI;
  constexpr auto index = SPI3_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    if ( auto spi = s_hld_drivers[ index ]; spi )
    {
      spi->postISRProcessing();
    }
  }
}
#endif

#if defined( STM32_SPI4_PERIPH_AVAILABLE )
static void SPI4ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::SPI;
  constexpr auto index = SPI4_RESOURCE_INDEX;

  while ( 1 )
  {
    postProcessorSignals[ index ].acquire();
    if ( auto spi = SPIClassObjects[ index ]; spi )
    {
      spi->postISRProcessing();
    }
  }
}
#endif

#endif /* THOR_HLD_SPI */
