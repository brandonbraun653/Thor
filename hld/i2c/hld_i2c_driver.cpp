/******************************************************************************
 *  File Name:
 *    hld_i2c_driver.cpp
 *
 *  Description:
 *    HLD I2C driver
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/event>
#include <Chimera/gpio>
#include <Chimera/i2c>
#include <Chimera/thread>
#include <Thor/cfg>
#include <Thor/i2c>
#include <Thor/lld/interface/inc/i2c>
#include <Thor/lld/interface/inc/interrupt>
#include <array>
#include <cstring>
#include <limits>


#if defined( THOR_HLD_I2C )
/*-----------------------------------------------------------------------------
Aliases
-----------------------------------------------------------------------------*/
namespace HLD = ::Thor::I2C;
namespace LLD = ::Thor::LLD::I2C;

/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
static constexpr size_t NUM_DRIVERS = LLD::NUM_I2C_PERIPHS;

/*-----------------------------------------------------------------------------
Variables
-----------------------------------------------------------------------------*/
static size_t s_driver_initialized;           /**< Tracks the module level initialization state */
static HLD::Driver hld_driver[ NUM_DRIVERS ]; /**< Driver objects */

namespace Thor::I2C
{
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
      for ( size_t index = 0; index < NUM_DRIVERS; index++ )
      {
        hld_driver[ index ].postISRProcessing();
      }
    }
  }

  /*---------------------------------------------------------------------------
  Public Functions
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
    Task userThread;
    TaskConfig cfg;

    cfg.arg        = nullptr;
    cfg.function   = I2CxISRUserThread;
    cfg.priority   = Priority::MAXIMUM;
    cfg.stackWords = STACK_BYTES( 512 );
    cfg.type       = TaskInitType::DYNAMIC;
    cfg.name       = "PP_I2Cx";

    userThread.create( cfg );
    LLD::INT::setUserTaskId( Chimera::Peripheral::Type::PERIPH_I2C, userThread.start() );

    /*-------------------------------------------------------------------------
    Initialize the low level driver
    -------------------------------------------------------------------------*/
    ::LLD::initialize();

    /*-------------------------------------------------------------------------
    Lock the init sequence and exit
    -------------------------------------------------------------------------*/
    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  Chimera::Status_t reset()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  Driver_rPtr getDriver( const Chimera::I2C::Channel channel )
  {
    if ( auto idx = ::LLD::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return &hld_driver[ idx ];
    }
    else
    {
      RT_HARD_ASSERT( false );
      return nullptr;
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
    gpio->init( sclCfg);
    for( size_t cnt = 0; cnt < 30; cnt++ )
    {
      gpio->toggle();
    }

    /*-------------------------------------------------------------------------
    Configure the SCL/SDA GPIO for peripheral function
    -------------------------------------------------------------------------*/
    auto pin = Chimera::GPIO::getDriver( cfg.SCLInit.port, cfg.SCLInit.pin );
    auto result = pin->init( cfg.SCLInit );

    pin = Chimera::GPIO::getDriver( cfg.SDAInit.port, cfg.SDAInit.pin );
    result |= pin->init( cfg.SDAInit );

    if( result != Chimera::Status::OK )
    {
      return Chimera::Status::FAIL;
    }

    /*-------------------------------------------------------------------------
    Configure the low level driver
    -------------------------------------------------------------------------*/
    if( ::LLD::getDriver( cfg.HWInit.channel )->configure( cfg ) == Chimera::Status::OK )
    {
      mConfig = cfg;
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
    return ::LLD::getDriver( mConfig.HWInit.channel )->read( address, data, length );
  }


  Chimera::Status_t Driver::write( const uint16_t address, const void *const data, const size_t length )
  {
    return ::LLD::getDriver( mConfig.HWInit.channel )->write( address, data, length );
  }


  Chimera::Status_t Driver::transfer( const uint16_t address, const void *const tx_data, void *const rx_data,
                                      const size_t length )
  {
    return ::LLD::getDriver( mConfig.HWInit.channel )->transfer( address, tx_data, rx_data, length );
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


  Chimera::Status_t Driver::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  Chimera::Status_t Driver::removeListener( const size_t registrationID, const size_t timeout )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }


  void Driver::postISRProcessing()
  {
    /*-------------------------------------------------------------------------
    Get what happened from the hardware driver
    -------------------------------------------------------------------------*/
    const auto txfr = ::LLD::getDriver( mConfig.HWInit.channel )->whatHappened();

    /*-------------------------------------------------------------------------
    Parse the IRQ event based on transfer status
    -------------------------------------------------------------------------*/
    switch( txfr.state )
    {
      case ::LLD::TxfrState::COMPLETE:
        this->signalAIO( Chimera::Event::Trigger::TRIGGER_TRANSFER_COMPLETE );
        break;

      case ::LLD::TxfrState::ERROR:
        if( txfr.errorBF & ( 1u << ::LLD::TxfrError::ERR_NACK ) )
        {
          this->signalAIO( Chimera::Event::Trigger::TRIGGER_NACK );
        }
        else
        {
          this->signalAIO( Chimera::Event::Trigger::TRIGGER_SYSTEM_ERROR );
        }
        break;

      default:
        return;
    };
  }

}    // namespace Thor::I2C

#endif /* THOR_HLD_I2C */
