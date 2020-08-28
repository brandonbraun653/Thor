/********************************************************************************
 *  File Name:
 *    thor_custom_gpio.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor GPIO interface.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <array>
#include <cstring>

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/gpio>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_detail.hpp>

#if defined( THOR_HLD_GPIO )

namespace Thor::GPIO
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  namespace HLD = ::Thor::GPIO;
  namespace LLD = ::Thor::LLD::GPIO;

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_GPIO_PERIPHS;

  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  // Tracks if the module data has been initialized correctly
  static size_t s_driver_initialized;

  // Stores the high/low level driver instances
  static HLD::Driver hld_driver[ NUM_DRIVERS ];
  static HLD::Driver_sPtr hld_shared[ NUM_DRIVERS ];


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*------------------------------------------------
    Prevent re-initialization from occurring
    ------------------------------------------------*/
    auto result = Chimera::Status::OK;
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return result;
    }

    /*------------------------------------------------
    Initialize local memory
    ------------------------------------------------*/
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    for ( size_t x = 0; x < NUM_DRIVERS; x++ )
    {
      hld_shared[ x ] = HLD::Driver_sPtr( &hld_driver[ x ] );
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    result = Thor::LLD::GPIO::initialize();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }


  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }


  Chimera::GPIO::IGPIO_sPtr getDriver( const Chimera::GPIO::Port port )
  {
    if ( ( port >= Chimera::GPIO::Port::NUM_OPTIONS ) || ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY ) )
    {
      return nullptr;
    }

    return hld_shared[ static_cast<size_t>( port ) ];
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver()
  {
    mChannel   = static_cast<size_t>( NUM_DRIVERS );
    mLastState = Chimera::GPIO::State::LOW;
    mInitSettings.clear();
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init( const Chimera::GPIO::PinInit &pinInit )
  {
    using namespace Thor::LLD::GPIO;
    using namespace Chimera::Threading;

    auto result = Chimera::Status::FAIL;
    auto port   = static_cast<size_t>( pinInit.port );
    auto locked = false;

    /*------------------------------------------------
    Ensure the port enumeration matches up with the LLD
    ------------------------------------------------*/
    if ( port < NUM_DRIVERS )
    {
      mInitSettings = pinInit;
      mChannel      = port;
      result        = setMode( pinInit.drive, pinInit.pull );

      // Ignore the return code because if setMode doesn't work, this won't either
      setState( pinInit.state );
    }
    else
    {
      mInitSettings.clear();
    }

    return result;
  }


  Chimera::Status_t Driver::init( const Chimera::GPIO::Port port, const uint8_t pin )
  {
    Chimera::GPIO::PinInit cfg;
    cfg.clear();

    cfg.port      = port;
    cfg.pin       = pin;
    cfg.alternate = Chimera::GPIO::Alternate::NONE;
    cfg.drive     = Chimera::GPIO::Drive::INPUT;
    cfg.state     = Chimera::GPIO::State::LOW;
    cfg.pull      = Chimera::GPIO::Pull::PULL_UP;

    return init( cfg );
  }


  Chimera::Status_t Driver::setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull )
  {
    /*------------------------------------------------
    Function entrancy checks
    ------------------------------------------------*/
    if ( ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY ) || !LLD::getDriver( mChannel ) )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Set up the hardware to implement the desired mode
    ------------------------------------------------*/
    auto result = Chimera::Status::FAIL;
    result      = LLD::getDriver( mChannel )->driveSet( mInitSettings.pin, drive );
    result |= LLD::getDriver( mChannel )->pullSet( mInitSettings.pin, pull );
    result |= LLD::getDriver( mChannel )->speedSet( mInitSettings.pin, Thor::LLD::GPIO::Speed::HIGH );

    /*------------------------------------------------
    Configure the alternate function options
    ------------------------------------------------*/
    if ( ( drive == Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN ) || ( drive == Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL ) )
    {
      result |= LLD::getDriver( mChannel )->alternateFunctionSet( mInitSettings.pin, mInitSettings.alternate );
    }

    return result;
  }


  Chimera::Status_t Driver::setState( const Chimera::GPIO::State state )
  {
    /*------------------------------------------------
    Function entrancy checks
    ------------------------------------------------*/
    if ( ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY ) || !LLD::getDriver( mChannel ) )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Grab the driver reference and invoke the function
    ------------------------------------------------*/
    return LLD::getDriver( mChannel )->write( mInitSettings.pin, state );
  }


  Chimera::Status_t Driver::getState( Chimera::GPIO::State &state )
  {
    /*------------------------------------------------
    Function entrancy checks
    ------------------------------------------------*/
    if ( ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY ) || !LLD::getDriver( mChannel ) )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Grab the driver reference and invoke the function
    ------------------------------------------------*/
    state = LLD::getDriver( mChannel )->read( mInitSettings.pin );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::toggle()
  {
    using namespace Chimera::GPIO;

    mLastState = ( mLastState == State::HIGH ) ? State::LOW : State::HIGH;
    return setState( mLastState );
  }
}    // namespace Thor::GPIO

#endif /* THOR_HLD_GPIO */
