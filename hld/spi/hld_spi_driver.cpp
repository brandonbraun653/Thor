/******************************************************************************
 *  File Name:
 *    hld_spi_driver.cpp
 *
 *  Description:
 *    SPI driver for Thor
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/spi>
#include <Chimera/thread>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/spi>
#include <array>
#include <cstring>
#include <limits>


#if defined( THOR_SPI )
namespace Chimera::SPI
{ /*---------------------------------------------------------------------------
   Aliases
   ---------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::SPI;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_SPI_PERIPHS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ThorImpl
  {
    LLD::Driver_rPtr           pLLDriver; /**< Low level SPI driver */
    Chimera::SPI::Driver_rPtr  pHLDriver; /**< High level SPI driver */
    Chimera::GPIO::Driver_rPtr pCSDriver; /**< Current chip select line driver */
    Chimera::SPI::HardwareInit hwConfig;  /**< Hardware configuration of the driver */

    void postISRProcessing()
    {
      // TODO: Add logic for listener invocation, error handling, CS toggling, and multiple transfers.

      /*-----------------------------------------------------------------------
      Decide chip select behavior
      -----------------------------------------------------------------------*/
      if ( hwConfig.csMode == Chimera::SPI::CSMode::AUTO_AFTER_TRANSFER )
      {
        pHLDriver->setChipSelect( Chimera::GPIO::State::HIGH );
      }

      /*-----------------------------------------------------------------------
      Notify threads waiting on the transfer complete signal
      -----------------------------------------------------------------------*/
      pHLDriver->signalAIO( Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE );
    }

    void clear()
    {
      pLLDriver = nullptr;
      pHLDriver = nullptr;
      pCSDriver = nullptr;
      hwConfig.clear();
    }
  };

  /*---------------------------------------------------------------------------
  Variables
  ---------------------------------------------------------------------------*/
  static DeviceManager<Driver, Channel, NUM_DRIVERS>   s_raw_drivers;
  static DeviceManager<ThorImpl, Channel, NUM_DRIVERS> s_impl_drivers;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Common thread for handling SPI interrupts in userspace
   *
   * @param arg   Unused
   */
  static void SPIxISRUserThread( void *arg )
  {
    using namespace Chimera::Thread;

    Chimera::SPI::Channel instance_list[ EnumValue( Chimera::SPI::Channel::NUM_OPTIONS ) ];

    while ( 1 )
    {
      /*-----------------------------------------------------------------------
      Wait for something to wake this thread directly
      -----------------------------------------------------------------------*/
      if ( !this_thread::pendTaskMsg( ITCMsg::TSK_MSG_ISR_HANDLER ) )
      {
        continue;
      }

      /*-----------------------------------------------------------------------
      Handle every ISR. Don't know which triggered this.
      -----------------------------------------------------------------------*/
      const size_t count = s_impl_drivers.registeredInstances( instance_list, ARRAY_COUNT( instance_list ) );
      for ( size_t idx = 0; idx < count; idx++ )
      {
        auto impl = s_impl_drivers.get( instance_list[ idx ] );
        impl->postISRProcessing();
      }
    }
  }


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init( const Chimera::SPI::DriverConfig &setupStruct )
  {
    using namespace Chimera::Thread;
    auto result = Chimera::Status::OK;

    /*-------------------------------------------------------------------------
    Is the driver supported?
    -------------------------------------------------------------------------*/
    if ( !LLD::isSupported( setupStruct.HWInit.hwChannel ) )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Initialize the AsyncIO drivers
    -------------------------------------------------------------------------*/
    this->initAIO();

    /*-------------------------------------------------------------------------
    Allocate the drivers
    -------------------------------------------------------------------------*/
    auto impl       = s_impl_drivers.getOrCreate( setupStruct.HWInit.hwChannel );
    impl->pLLDriver = LLD::getDriver( setupStruct.HWInit.hwChannel );
    RT_DBG_ASSERT( impl && impl->pLLDriver );

    mImpl           = reinterpret_cast<void *>( impl );
    impl->pHLDriver = this;
    impl->hwConfig  = setupStruct.HWInit;

    /*-------------------------------------------------------------------------
    Configure the GPIO
    -------------------------------------------------------------------------*/
    auto SCK  = Chimera::GPIO::getDriver( setupStruct.SCKInit.port, setupStruct.SCKInit.pin );
    auto MOSI = Chimera::GPIO::getDriver( setupStruct.MOSIInit.port, setupStruct.MOSIInit.pin );
    auto MISO = Chimera::GPIO::getDriver( setupStruct.MISOInit.port, setupStruct.MISOInit.pin );

    result |= SCK->init( setupStruct.SCKInit );
    result |= MOSI->init( setupStruct.MOSIInit );
    result |= MISO->init( setupStruct.MISOInit );

    if ( setupStruct.CSInit.validity )
    {
      impl->pCSDriver = Chimera::GPIO::getDriver( setupStruct.CSInit.port, setupStruct.CSInit.pin );
      result |= impl->pCSDriver->init( setupStruct.CSInit );
    }

    if ( result != Chimera::Status::OK )
    {
      return Chimera::Status::FAILED_INIT;
    }

    /* Make sure we aren't selecting a device by accident */
    setChipSelect( Chimera::GPIO::State::HIGH );

    /*-------------------------------------------------------------------------
    Configure the SPI hardware
    -------------------------------------------------------------------------*/
    result |= impl->pLLDriver->configure( impl->hwConfig );
    result |= impl->pLLDriver->registerConfig( &impl->hwConfig );

    return result;
  }


  Chimera::SPI::HardwareInit Driver::getInit()
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    return impl->hwConfig;
  }


  Chimera::Status_t Driver::deInit()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::assignChipSelect( const Chimera::GPIO::Driver_rPtr cs )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl       = reinterpret_cast<ThorImpl *>( mImpl );
    impl->pCSDriver = cs;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setChipSelect( const Chimera::GPIO::State value )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    if ( impl->pCSDriver )
    {
      return impl->pCSDriver->setState( value );
    }

    return Chimera::Status::NOT_AVAILABLE;
  }


  Chimera::Status_t Driver::setChipSelectControlMode( const Chimera::SPI::CSMode mode )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl             = reinterpret_cast<ThorImpl *>( mImpl );
    impl->hwConfig.csMode = mode;
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
    RT_DBG_ASSERT( mImpl );
    auto impl   = reinterpret_cast<ThorImpl *>( mImpl );
    auto result = Chimera::Status::OK;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( ( !txBuffer && !rxBuffer ) || !length )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Handle the chip select controller behavior
    -------------------------------------------------------------------------*/
    if ( impl->hwConfig.csMode != Chimera::SPI::CSMode::MANUAL )
    {
      result |= setChipSelect( Chimera::GPIO::State::LOW );
    }

    /*-------------------------------------------------------------------------
    Call the proper transfer method
    -------------------------------------------------------------------------*/
    switch ( impl->hwConfig.txfrMode )
    {
      case Chimera::SPI::TransferMode::BLOCKING:
        result |= impl->pLLDriver->transfer( txBuffer, rxBuffer, length );

        /*---------------------------------------------------------------------
        Disengage the CS line if required
        ---------------------------------------------------------------------*/
        if ( impl->hwConfig.csMode != Chimera::SPI::CSMode::MANUAL )
        {
          result |= setChipSelect( Chimera::GPIO::State::HIGH );
        }

        /*---------------------------------------------------------------------
        Transfer is done. Awaken any threads that are blocking on this signal.
        ---------------------------------------------------------------------*/
        signalAIO( Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE );

        return result;
        break;

      case Chimera::SPI::TransferMode::INTERRUPT:
        result |= impl->pLLDriver->transferIT( txBuffer, rxBuffer, length );
        break;

      case Chimera::SPI::TransferMode::DMA:
        result |= impl->pLLDriver->transferDMA( txBuffer, rxBuffer, length );
        break;

      default:
        /*---------------------------------------------------------------------
        Disable the chip select for anything other than manual control
        ---------------------------------------------------------------------*/
        if ( impl->hwConfig.csMode != Chimera::SPI::CSMode::MANUAL )
        {
          setChipSelect( Chimera::GPIO::State::HIGH );
        }

        result = Chimera::Status::FAIL;
        break;
    }

    return result;
  }


  Chimera::Status_t Driver::setPeripheralMode( const Chimera::Hardware::PeripheralMode mode )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    impl->hwConfig.txfrMode = mode;
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::setClockFrequency( const size_t freq, const size_t tolerance )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    /*-------------------------------------------------------------------------
    Assign the new clock frequency and re-initialize the driver
    -------------------------------------------------------------------------*/
    impl->hwConfig.clockFreq = freq;
    return impl->pLLDriver->configure( impl->hwConfig );
  }


  size_t Driver::getClockFrequency()
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    return impl->hwConfig.clockFreq;
  }

}    // namespace Chimera::SPI

namespace Chimera::SPI::Backend
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static size_t s_driver_initialized;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Prevent multiple initializations
    -------------------------------------------------------------------------*/
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*-------------------------------------------------------------------------
    Register the ISR post-processor thread
    -------------------------------------------------------------------------*/
    Task       userThread;
    TaskConfig cfg;

    cfg.arg        = nullptr;
    cfg.function   = SPIxISRUserThread;
    cfg.priority   = Priority::MAXIMUM;
    cfg.stackWords = STACK_BYTES( 512 );
    cfg.type       = TaskInitType::DYNAMIC;
    cfg.name       = "PP_SPIx";

    userThread.create( cfg );
    ::Thor::LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_SPI, userThread.start() );

    /*-------------------------------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------------------------------*/
    LLD::initialize();

    /*-------------------------------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  Chimera::Status_t reset()
  {
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  Driver_rPtr getDriver( const Chimera::SPI::Channel channel )
  {
    if ( !LLD::isSupported( channel ) )
    {
      return nullptr;
    }

    return s_raw_drivers.getOrCreate( channel );
  }


  Chimera::Status_t registerDriver( Chimera::SPI::Backend::DriverConfig &registry )
  {
    registry.isSupported = true;
    registry.getDriver   = ::Chimera::SPI::Backend::getDriver;
    registry.initialize  = ::Chimera::SPI::Backend::initialize;
    registry.reset       = ::Chimera::SPI::Backend::reset;
    return Chimera::Status::OK;
  }

}    // namespace Chimera::SPI::Backend

#endif /* THOR_SPI */
