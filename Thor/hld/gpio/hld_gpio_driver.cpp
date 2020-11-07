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
#include <cstring>

/* Aurora Includes */
#include <Aurora/constants>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/utility>

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
  // Each HLD driver is associated with a pin, not the peripheral instance
  static constexpr size_t NUM_DRIVERS = LLD::NUM_GPIO_PINS;

  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static size_t s_driver_initialized;                /**< Tracks the module level initialization state */
  static HLD::Driver hld_driver[ NUM_DRIVERS ];      /**< Driver objects */
  static HLD::Driver_sPtr hld_shared[ NUM_DRIVERS ]; /**< Shared references to driver objects */


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
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_GPIO )
      hld_shared[ x ] = HLD::Driver_sPtr( new HLD::Driver() );
#else
      hld_shared[ x ] = HLD::Driver_sPtr( &hld_driver[ x ] );
#endif
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


  Driver_rPtr getDriver( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    if ( auto idx = LLD::getPinResourceIndex( port, pin ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return &hld_driver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  Driver_sPtr getDriverShared( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    if ( auto idx = LLD::getPinResourceIndex( port, pin ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
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
  Driver::Driver() :
      mAlternate( Chimera::GPIO::Alternate::NONE ), mPin( std::numeric_limits<decltype( mPin )>::max() ),
      mPort( Chimera::GPIO::Port::UNKNOWN_PORT )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init( const Chimera::GPIO::PinInit &pinInit )
  {
    auto result = Chimera::Status::FAIL;
    auto idx    = LLD::getPinResourceIndex( pinInit.port, pinInit.pin );
    auto locked = false;

    /*------------------------------------------------
    Ensure the port enumeration matches up with the LLD
    ------------------------------------------------*/
    if ( idx < NUM_DRIVERS )
    {
      mAlternate = pinInit.alternate;
      mPin       = pinInit.pin;
      mPort      = pinInit.port;
      result     = setMode( pinInit.drive, pinInit.pull );

      // Ignore the return code because if setMode doesn't work, this won't either
      setState( pinInit.state );
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
    auto result = Chimera::Status::FAIL;
    auto driver = LLD::getDriver( mPort, mPin );
    if ( ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY ) || !driver )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Set up the hardware to implement the desired mode
    ------------------------------------------------*/
    result = driver->driveSet( mPin, drive );
    result |= driver->pullSet( mPin, pull );
    result |= driver->speedSet( mPin, Thor::LLD::GPIO::Speed::HIGH );

    /*------------------------------------------------
    Configure the alternate function options
    ------------------------------------------------*/
    if ( ( drive == Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN ) || ( drive == Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL ) )
    {
      result |= driver->alternateFunctionSet( mPin, mAlternate );
    }

    return result;
  }


  Chimera::Status_t Driver::setState( const Chimera::GPIO::State state )
  {
    /*------------------------------------------------
    Function entrancy checks
    ------------------------------------------------*/
    auto driver = LLD::getDriver( mPort, mPin );
    if ( ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY ) || !driver )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Grab the driver reference and invoke the function
    ------------------------------------------------*/
    return driver->write( mPin, state );
  }


  Chimera::Status_t Driver::getState( Chimera::GPIO::State &state )
  {
    /*------------------------------------------------
    Function entrancy checks
    ------------------------------------------------*/
    auto driver = LLD::getDriver( mPort, mPin );
    if ( ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY ) || !driver )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Grab the driver reference and invoke the function
    ------------------------------------------------*/
    state = driver->read( mPin );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::toggle()
  {
    using namespace Chimera::GPIO;

    auto state = LLD::getDriver( mPort, mPin )->read( mPin );
    state      = ( state == State::HIGH ) ? State::LOW : State::HIGH;
    return setState( state );
  }


  Chimera::Status_t Driver::attachInterrupt( Chimera::Function::vGeneric &func, const Chimera::EXTI::EdgeTrigger trigger )
  {
    return LLD::getDriver( mPort, mPin )->attachInterrupt( mPin, func, trigger );
  }


  void Driver::detachInterrupt()
  {
    return LLD::getDriver( mPort, mPin )->detachInterrupt( mPin );
  }
}    // namespace Thor::GPIO

#endif /* THOR_HLD_GPIO */
