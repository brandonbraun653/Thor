/******************************************************************************
 *  File Name:
 *    hld_i2c_driver.cpp
 *
 *  Description:
 *    HLD I2C driver
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/i2c>
#include <Chimera/thread>
#include <Chimera/peripheral>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/i2c>
#include <Thor/lld/interface/inc/interrupt>
#include <array>
#include <cstring>
#include <limits>


#if defined( THOR_I2C )
namespace Chimera::I2C
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::I2C;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_I2C_PERIPHS;

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct ThorImpl
  {
    LLD::Driver_rPtr               lldriver;
    Chimera::I2C::Driver_rPtr      hldriver;
    Chimera::I2C::DriverConfig     mConfig;
    Chimera::Event::ActionableList eventListeners;

    void postISRProcessing()
    {
      /*-------------------------------------------------------------------------
      Get what happened from the hardware driver
      -------------------------------------------------------------------------*/
      const auto txfr = lldriver->whatHappened();

      /*-------------------------------------------------------------------------
      Parse the IRQ event based on transfer status
      -------------------------------------------------------------------------*/
      switch ( txfr.state )
      {
        case LLD::TxfrState::COMPLETE:
          hldriver->signalAIO( Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE );
          break;

        case LLD::TxfrState::ERROR:
          if ( txfr.errorBF & ( 1u << LLD::TxfrError::ERR_NACK ) )
          {
            hldriver->signalAIO( Chimera::Event::Trigger::TRIGGER_NACK );
          }
          else
          {
            hldriver->signalAIO( Chimera::Event::Trigger::TRIGGER_SYSTEM_ERROR );
          }
          break;

        default:
          return;
      };
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
   * @brief Common thread for handling I2C interrupts in userspace
   *
   * @param arg   Unused
   */
  static void I2CxISRUserThread( void *arg )
  {
    using namespace Chimera::Thread;

    Chimera::I2C::Channel instance_list[ EnumValue( Chimera::I2C::Channel::NUM_OPTIONS ) ];

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
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver()
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::open( const Chimera::I2C::DriverConfig &cfg )
  {
    /*-------------------------------------------------------------------------
    Ensure the AsyncIO driver is ready
    -------------------------------------------------------------------------*/
    this->initAIO();

    /*-------------------------------------------------------------------------
    Reset the I2C bus by twiddling the SCL line
    -------------------------------------------------------------------------*/
    Chimera::GPIO::PinInit sclCfg;

    sclCfg.clear();
    sclCfg.alternate = Chimera::GPIO::Alternate::NONE;
    sclCfg.drive     = Chimera::GPIO::Drive::OUTPUT_PUSH_PULL;
    sclCfg.pull      = Chimera::GPIO::Pull::NO_PULL;
    sclCfg.pin       = cfg.SCLInit.pin;
    sclCfg.port      = cfg.SCLInit.port;
    sclCfg.state     = Chimera::GPIO::State::HIGH;
    sclCfg.validity  = true;

    auto gpio = Chimera::GPIO::getDriver( sclCfg.port, sclCfg.pin );
    gpio->init( sclCfg );
    for ( size_t cnt = 0; cnt < 30; cnt++ )
    {
      gpio->toggle();
      Chimera::blockDelayMicroseconds( 10 );
    }

    Chimera::blockDelayMicroseconds( 150 );

    /*-------------------------------------------------------------------------
    Configure the SCL/SDA GPIO for peripheral function
    -------------------------------------------------------------------------*/
    auto pin    = Chimera::GPIO::getDriver( cfg.SCLInit.port, cfg.SCLInit.pin );
    auto result = pin->init( cfg.SCLInit );

    pin = Chimera::GPIO::getDriver( cfg.SDAInit.port, cfg.SDAInit.pin );
    result |= pin->init( cfg.SDAInit );

    if ( result != Chimera::Status::OK )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Configure the low level driver
    -------------------------------------------------------------------------*/
    auto impl = s_impl_drivers.getOrCreate( cfg.HWInit.channel );
    mImpl     = reinterpret_cast<void *>( impl );
    RT_DBG_ASSERT( impl );

    impl->lldriver = LLD::getLLDriver( cfg.HWInit.channel );
    impl->hldriver = this;
    RT_DBG_ASSERT( impl->lldriver );

    if ( impl->lldriver->configure( cfg ) == Chimera::Status::OK )
    {
      impl->mConfig = cfg;
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  Chimera::Status_t Driver::close()
  {
    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::read( const uint16_t address, void *const data, const size_t length )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    return impl->lldriver->read( address, data, length );
  }


  Chimera::Status_t Driver::write( const uint16_t address, const void *const data, const size_t length )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    return impl->lldriver->write( address, data, length );
  }


  Chimera::Status_t Driver::transfer( const uint16_t address, const void *const tx_data, void *const rx_data,
                                      const size_t length )
  {
    RT_DBG_ASSERT( mImpl );
    auto impl = reinterpret_cast<ThorImpl *>( mImpl );

    return impl->lldriver->transfer( address, tx_data, rx_data, length );
  }


  Chimera::Status_t Driver::stop()
  {
    /*-------------------------------------------------------------------------
    Technically supported in hardware, but not needed in software.
    -------------------------------------------------------------------------*/
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::start()
  {
    /*-------------------------------------------------------------------------
    Technically supported in hardware, but not needed in software.
    -------------------------------------------------------------------------*/
    return Chimera::Status::NOT_SUPPORTED;
  }

}    // namespace Chimera::I2C


namespace Chimera::I2C::Backend
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static size_t s_driver_initialized;

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static Chimera::Status_t initialize()
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
    cfg.function   = I2CxISRUserThread;
    cfg.priority   = Priority::MAXIMUM;
    cfg.stackWords = STACK_BYTES( 512 );
    cfg.type       = TaskInitType::DYNAMIC;
    cfg.name       = "PP_I2Cx";

    userThread.create( cfg );
    ::Thor::LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_I2C, userThread.start() );

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


  static Chimera::Status_t reset()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  static Driver_rPtr getDriver( const Chimera::I2C::Channel channel )
  {
    if ( !LLD::isSupported( channel ) )
    {
      return nullptr;
    }

    return s_raw_drivers.getOrCreate( channel );
  }


  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::I2C::Backend::DriverConfig &registry )
  {
    registry.isSupported = true;
    registry.getDriver   = ::Chimera::I2C::Backend::getDriver;
    registry.initialize  = ::Chimera::I2C::Backend::initialize;
    registry.reset       = ::Chimera::I2C::Backend::reset;
    return Chimera::Status::OK;
  }
}    // namespace Chimera::I2C::Backend
#endif /* THOR_I2C */
