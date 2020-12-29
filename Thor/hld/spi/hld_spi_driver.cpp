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
#include <Aurora/constants>

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
#include <Thor/lld/interface/spi/spi_types.hpp>

#if defined( THOR_HLD_SPI )

/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = ::Thor::SPI;
namespace LLD = ::Thor::LLD::SPI;

using ThreadHandle = Chimera::Threading::detail::native_thread_handle_type;
using BinarySemphr = Chimera::Threading::BinarySemaphore;
using ThreadFunctn = Chimera::Function::void_func_void_ptr;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = LLD::NUM_SPI_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
static size_t s_driver_initialized;                        /**< Tracks the module level initialization state */
static HLD::Driver hld_driver[ NUM_DRIVERS ];              /**< Driver objects */
static HLD::Driver_sPtr hld_shared[ NUM_DRIVERS ];         /**< Shared references to driver objects */
static ThreadHandle s_user_isr_handle[ NUM_DRIVERS ];      /**< Handle to the ISR post processing thread */
static BinarySemphr s_user_isr_signal[ NUM_DRIVERS ];      /**< Lock for each ISR post processing thread */
static ThreadFunctn s_user_isr_thread_func[ NUM_DRIVERS ]; /**< RTOS aware function to execute at end of ISR */

/*-------------------------------------------------------------------------------
Private Function Declarations
-------------------------------------------------------------------------------*/
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

    /*------------------------------------------------
    Initialize ISR post-processing routines
    ------------------------------------------------*/
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::SPI1_RESOURCE_INDEX ] = SPI1ISRPostProcessorThread;
#endif
#if defined( STM32_SPI2_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::SPI2_RESOURCE_INDEX ] = SPI2ISRPostProcessorThread;
#endif
#if defined( STM32_SPI3_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::SPI3_RESOURCE_INDEX ] = SPI3ISRPostProcessorThread;
#endif
#if defined( STM32_SPI4_PERIPH_AVAILABLE )
    s_user_isr_thread_func[::LLD::SPI4_RESOURCE_INDEX ] = SPI4ISRPostProcessorThread;
#endif

    /*-------------------------------------------------
    Initialize shared references to drivers
    -------------------------------------------------*/
    for ( size_t x = 0; x < NUM_DRIVERS; x++ )
    {
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_SPI )
      hld_shared[ x ] = ::HLD::Driver_sPtr( new ::HLD::Driver() );
#else
      hld_shared[ x ] = ::HLD::Driver_sPtr( &hld_driver[ x ] );
#endif
    }

    /*-------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  Chimera::Status_t reset()
  {
    /*------------------------------------------------
    Only allow clearing of local data during testing
    ------------------------------------------------*/
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_GPIO )
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    for ( auto x = 0; x < NUM_DRIVERS; x++ )
    {
      hld_shared[ x ].reset();
    }
#endif

    return Chimera::Status::OK;
  }


  Driver_rPtr getDriver( const Chimera::SPI::Channel channel )
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


  Driver_sPtr getDriverShared( const Chimera::SPI::Channel channel )
  {
    if ( auto idx = ::LLD::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return hld_shared[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : SCK( nullptr ), MOSI( nullptr ), MISO( nullptr ), CS( nullptr )
  {
    config.clear();
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
    auto result = Chimera::Status::OK;

    /*------------------------------------------------
    Should we even bother creating this?
    ------------------------------------------------*/
    if ( !::LLD::isSupported( setupStruct.HWInit.hwChannel ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*------------------------------------------------
    First register the driver and initialize class vars
    ------------------------------------------------*/
    auto lldResourceIndex = ::LLD::getResourceIndex( setupStruct.HWInit.hwChannel );
    if ( lldResourceIndex == ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*------------------------------------------------
    Configure the GPIO
    ------------------------------------------------*/
    config = setupStruct;

    SCK  = Chimera::GPIO::getDriver( config.SCKInit.port, config.SCKInit.pin );
    MOSI = Chimera::GPIO::getDriver( config.MOSIInit.port, config.MOSIInit.pin );
    MISO = Chimera::GPIO::getDriver( config.MISOInit.port, config.MISOInit.pin );

    result |= SCK->init( config.SCKInit );
    result |= MOSI->init( config.MOSIInit );
    result |= MISO->init( config.MISOInit );

    /* Does the driver take control of the CS pin? */
    if ( setupStruct.externalCS )
    {
      setChipSelectControlMode( Chimera::SPI::CSMode::MANUAL );
    }
    else
    {
      CS = Chimera::GPIO::getDriver( config.CSInit.port, config.CSInit.pin );
      result |= CS->init( config.CSInit );
    }

    if ( result != Chimera::Status::OK )
    {
      return Chimera::Status::FAILED_INIT;
    }

    /* Make sure we aren't selecting a device by accident */
    setChipSelect( Chimera::GPIO::State::HIGH );

    /*------------------------------------------------
    Configure the SPI hardware
    ------------------------------------------------*/
    auto driver = ::LLD::getDriver( config.HWInit.hwChannel );
    result |= driver->configure( config );
    result |= driver->registerConfig( &config );

    if ( result != Chimera::Status::OK )
    {
      config.validity = false;
    }

    /*------------------------------------------------
    Register the ISR post processor thread
    ------------------------------------------------*/
    std::array<char, 10> tmp;

    if ( s_user_isr_thread_func[ lldResourceIndex ] )
    {
      // Yeah this is gonna be bad if someone re-initializes the SPI driver....
      // postProcessorHandles[ lldResourceIndex ] = nullptr;

      driver->attachISRWakeup( &s_user_isr_signal[ lldResourceIndex ] );

      tmp.fill( 0 );
      snprintf( tmp.data(), tmp.size(), "PP_SPI%d", lldResourceIndex );
      std::string_view threadName = tmp.data();

      Chimera::Threading::Thread thread;
      thread.initialize( s_user_isr_thread_func[ lldResourceIndex ], nullptr, Chimera::Threading::Priority::LEVEL_5,
                         STACK_BYTES( 250 ), threadName );
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
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setChipSelect( const Chimera::GPIO::State value )
  {
    /*-------------------------------------------------
    Setting a chip select is only valid if we've been
    configured to have control of one.
    -------------------------------------------------*/
    if ( CS )
    {
      return CS->setState( value );
    }

    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::setChipSelectControlMode( const Chimera::SPI::CSMode mode )
  {
    /*------------------------------------------------
    Only valid if the SPI driver has control of the chip select
    ------------------------------------------------*/
    if ( !config.externalCS )
    {
      config.HWInit.csMode = mode;
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::writeBytes( const void *const txBuffer, const size_t length )
  {
    return readWriteBytes( txBuffer, nullptr, length );
  }


  Chimera::Status_t Driver::readBytes( void *const rxBuffer, const size_t length )
  {
    return readWriteBytes( nullptr, rxBuffer, length );
  }


  Chimera::Status_t Driver::readWriteBytes( const void *const txBuffer, void *const rxBuffer, const size_t length )
  {
    auto result = Chimera::Status::OK;
    auto driver = ::LLD::getDriver( config.HWInit.hwChannel );

    /*------------------------------------------------
    Input protection & resource acquisition
    ------------------------------------------------*/
    if ( ( !txBuffer && !rxBuffer ) || !length || !driver )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
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
        /*-------------------------------------------------
        Perform the transfer and optionaly disengage CS pin
        -------------------------------------------------*/
        result = driver->transfer( txBuffer, rxBuffer, length );

        if ( config.HWInit.csMode != Chimera::SPI::CSMode::MANUAL )
        {
          setChipSelect( Chimera::GPIO::State::HIGH );
        }

        /*-------------------------------------------------
        Transfer is complete at this point. Awaken any
        threads that are blocking on this signal.
        -------------------------------------------------*/
        awaitTransferComplete.release();

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

        return Chimera::Status::FAIL;
        break;
    }
  }


  Chimera::Status_t Driver::setPeripheralMode( const Chimera::Hardware::PeripheralMode mode )
  {
    config.HWInit.txfrMode = mode;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setClockFrequency( const size_t freq, const size_t tolerance )
  {
    /*------------------------------------------------
    Input protection
    ------------------------------------------------*/
    auto driver = ::LLD::getDriver( config.HWInit.hwChannel );
    if ( !driver || !freq )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Assign the new clock frequency and re-initialize the driver
    ------------------------------------------------*/
    config.HWInit.clockFreq = freq;
    return driver->configure( config );
  }


  size_t Driver::getClockFrequency()
  {
    return config.HWInit.clockFreq;
  }


  /*------------------------------------------------
  Async IO Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::await( const Chimera::Event::Trigger event, const size_t timeout )
  {
    if ( event != Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
    else if ( !awaitTransferComplete.try_acquire_for( timeout ) )
    {
      return Chimera::Status::TIMEOUT;
    }
    else
    {
      return Chimera::Status::OK;
    }
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


  /*------------------------------------------------
  Listener Interface
  ------------------------------------------------*/
  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

}    // namespace Thor::SPI


#if defined( STM32_SPI1_PERIPH_AVAILABLE )
static void SPI1ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::SPI1_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif

#if defined( STM32_SPI2_PERIPH_AVAILABLE )
static void SPI2ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::SPI2_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif

#if defined( STM32_SPI3_PERIPH_AVAILABLE )
static void SPI3ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::SPI3_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif

#if defined( STM32_SPI4_PERIPH_AVAILABLE )
static void SPI4ISRPostProcessorThread( void *argument )
{
  constexpr auto index = ::LLD::SPI4_RESOURCE_INDEX;

  while ( 1 )
  {
    s_user_isr_signal[ index ].acquire();
    hld_driver[ index ].postISRProcessing();
  }
}
#endif

#endif /* THOR_HLD_SPI */
