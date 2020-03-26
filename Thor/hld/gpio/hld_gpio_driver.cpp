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
#include <Thor/hld/gpio/hld_gpio_driver.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_detail.hpp>

#if defined( THOR_HLD_GPIO )

namespace Thor::GPIO
{
  // Tracks if the module data has been initialized correctly
  static size_t s_driver_initialized;

  // Stores the low level driver instances
  static std::array<Thor::LLD::GPIO::IDriver_sPtr, Thor::LLD::GPIO::NUM_GPIO_PERIPHS> drivers;

  /*------------------------------------------------
  High Level Driver Free Functions
  ------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*------------------------------------------------
    Prevent re-initialization from occurring
    ------------------------------------------------*/
    auto result = Chimera::CommonStatusCodes::OK;
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return result;
    }
    
    /*------------------------------------------------
    Initialize local memory
    ------------------------------------------------*/
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    for ( size_t x = 0; x < drivers.size(); x++ )
    {
      drivers[ x ] = nullptr;
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    result = Thor::LLD::GPIO::initialize();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }

  /*------------------------------------------------
  High Level Driver Class Implementation
  ------------------------------------------------*/
  Driver::Driver()
  {
    channel   = drivers.size();
    lastState = Chimera::GPIO::State::OFF;
    initSettings.clear();
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::init( const Chimera::GPIO::PinInit &pinInit, const size_t timeout )
  {
    using namespace Thor::LLD::GPIO;
    using namespace Chimera::Threading;

    auto result = Chimera::CommonStatusCodes::FAIL;
    auto port   = static_cast<size_t>( pinInit.port );
    auto locked = false;

    /*------------------------------------------------
    Optionally invoke thread-safe access
    ------------------------------------------------*/
    if ( pinInit.threaded && this->try_lock_for( timeout ) ) 
    {
      locked = true;
    }

    /*------------------------------------------------
    Ensure the port enumeration matches up with the LLD
    ------------------------------------------------*/
    if ( port < drivers.size() )
    {
      initSettings    = pinInit;
      channel         = port;
      drivers[ port ] = getDriver( port );
      result          = setMode( pinInit.drive, pinInit.pull );

      // Ignore the return code because if setMode doesn't work, this won't either
      setState( pinInit.state );
    }
    else
    {
      initSettings.clear();
    }

    /*------------------------------------------------
    Clear the lock if nessessary
    ------------------------------------------------*/
    if ( locked ) 
    {
      this->unlock();
    }

    return result;
  }

  Chimera::Status_t Driver::init( const Chimera::GPIO::Port port, const uint8_t pin, const size_t timeout )
  {
    Chimera::GPIO::PinInit cfg;
    cfg.clear();

    cfg.port      = port;
    cfg.pin       = pin;
    cfg.alternate = Chimera::GPIO::Alternate::NONE;
    cfg.drive     = Chimera::GPIO::Drive::INPUT;
    cfg.state     = Chimera::GPIO::State::OFF;
    cfg.pull      = Chimera::GPIO::Pull::PULL_UP;

    return init( cfg, timeout );
  }

  Chimera::Status_t Driver::setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull, const size_t timeout )
  {
    /*------------------------------------------------
    Function entrancy checks
    ------------------------------------------------*/
    if ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( !drivers[ channel ] )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Optionally invoke thread-safe access
    ------------------------------------------------*/
    auto locked = false;
    auto result = Chimera::CommonStatusCodes::FAIL;
    if ( initSettings.threaded && this->try_lock_for( timeout ) ) 
    {
      locked = true;
    }

    /*------------------------------------------------
    Set up the hardware to implement the desired mode
    ------------------------------------------------*/
    result = drivers[ channel ]->driveSet( initSettings.pin, drive );
    result |= drivers[ channel ]->pullSet( initSettings.pin, pull );
    result |= drivers[ channel ]->speedSet( initSettings.pin, Thor::LLD::GPIO::Speed::HIGH );

    /*------------------------------------------------
    Configure the alternate function options
    ------------------------------------------------*/
    if ( ( drive == Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN ) || ( drive == Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL ) ) 
    {
      result |= drivers[ channel ]->alternateFunctionSet( initSettings.pin, initSettings.alternate );
    }
    
    /*------------------------------------------------
    Clear the lock if nessessary
    ------------------------------------------------*/
    if ( locked ) 
    {
      this->unlock();
    }

    return result;
  }

  Chimera::Status_t Driver::setState( const Chimera::GPIO::State state, const size_t timeout )
  {
    /*------------------------------------------------
    Function entrancy checks
    ------------------------------------------------*/
    if ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( !drivers[ channel ] )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Optionally invoke thread-safe access
    ------------------------------------------------*/
    auto locked = false;
    auto result = Chimera::CommonStatusCodes::FAIL;
    if ( initSettings.threaded && this->try_lock_for( timeout ) ) 
    {
      locked = true;
    }

    /*------------------------------------------------
    Grab the driver reference and invoke the function
    ------------------------------------------------*/
    result = drivers[ channel ]->write( initSettings.pin, state );

    /*------------------------------------------------
    Clear the lock if nessessary
    ------------------------------------------------*/
    if ( locked ) 
    {
      this->unlock();
    }

    return result;
  }

  Chimera::Status_t Driver::getState( Chimera::GPIO::State &state, const size_t timeout )
  {
    /*------------------------------------------------
    Function entrancy checks
    ------------------------------------------------*/
    if ( s_driver_initialized != Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
    else if ( !drivers[ channel ] )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Optionally invoke thread-safe access
    ------------------------------------------------*/
    auto locked = false;
    auto result = Chimera::CommonStatusCodes::FAIL;
    if ( initSettings.threaded && this->try_lock_for( timeout ) ) 
    {
      locked = true;
    }

    /*------------------------------------------------
    Grab the driver reference and invoke the function
    ------------------------------------------------*/
    state = drivers[ channel ]->read( initSettings.pin );

    /*------------------------------------------------
    Clear the lock if nessessary
    ------------------------------------------------*/
    if ( locked ) 
    {
      this->unlock();
    }

    return result;
  }

  Chimera::Status_t Driver::toggle( const size_t timeout )
  {
    using namespace Chimera::GPIO;

    lastState = ( lastState == State::HIGH ) ? State::LOW : State::HIGH;
    return setState( lastState, timeout );
  }
}

#endif  /* THOR_DRIVER_GPIO */