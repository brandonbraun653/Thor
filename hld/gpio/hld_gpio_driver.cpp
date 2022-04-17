/********************************************************************************
 *  File Name:
 *    hld_gpio_driver.cpp
 *
 *  Description:
 *    Implements the custom driver variant of the Thor GPIO interface.
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/constants>
#include <Chimera/common>
#include <Chimera/gpio>
#include <Chimera/peripheral>
#include <Chimera/utility>
#include <Thor/cfg>
#include <Thor/gpio>
#include <Thor/lld/interface/inc/gpio>
#include <cstring>

#if defined( THOR_HLD_GPIO )
namespace Thor::GPIO
{
  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  namespace HLD = ::Thor::GPIO;
  namespace LLD = ::Thor::LLD::GPIO;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DRIVERS = LLD::NUM_GPIO_PINS;

  /*---------------------------------------------------------------------------
  Static Variables
  ---------------------------------------------------------------------------*/
  static size_t s_driver_initialized;           /**< Tracks the module level initialization state */
  static Chimera::DeviceManager<HLD::Driver, size_t, NUM_DRIVERS> s_hld_drivers;

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
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


  Driver_rPtr getDriver( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    auto idx = LLD::getPinResourceIndex( port, pin );
    if( idx == ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return nullptr;
    }

    return s_hld_drivers.getOrCreate( idx );
  }

  /*---------------------------------------------------------------------------
  Driver Class Implementation
  ---------------------------------------------------------------------------*/
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

    /*------------------------------------------------
    Ensure the requested pin is supported
    ------------------------------------------------*/
    auto idx = LLD::getPinResourceIndex( pinInit.port, pinInit.pin );
    if ( idx < NUM_DRIVERS )
    {
      result     = Chimera::Status::OK;
      mAlternate = pinInit.alternate;
      mPin       = pinInit.pin;
      mPort      = pinInit.port;

      /*-------------------------------------------------
      Enable the peripheral clock
      -------------------------------------------------*/
      auto driver = LLD::getLLDriver( mPort, mPin );
      driver->clockEnable();

      /*-------------------------------------------------
      Configure the basic IO mode settings
      -------------------------------------------------*/
      result |= setMode( pinInit.drive, pinInit.pull );
      result |= setState( pinInit.state );
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
    auto driver = LLD::getLLDriver( mPort, mPin );
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
    auto driver = LLD::getLLDriver( mPort, mPin );
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
    auto driver = LLD::getLLDriver( mPort, mPin );
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

    auto state = LLD::getLLDriver( mPort, mPin )->read( mPin );
    state      = ( state == State::HIGH ) ? State::LOW : State::HIGH;
    return setState( state );
  }


  Chimera::Status_t Driver::attachInterrupt( Chimera::Function::vGeneric &func, const Chimera::EXTI::EdgeTrigger trigger )
  {
    return LLD::getLLDriver( mPort, mPin )->attachInterrupt( mPin, func, trigger );
  }


  void Driver::detachInterrupt()
  {
    return LLD::getLLDriver( mPort, mPin )->detachInterrupt( mPin );
  }


  Chimera::EXTI::EventLine_t Driver::getInterruptLine()
  {
    return LLD::findEventLine( mPort, mPin );
  }
}    // namespace Thor::GPIO

#endif /* THOR_HLD_GPIO */
