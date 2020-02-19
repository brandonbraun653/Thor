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
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/hld/gpio/hld_gpio_driver.hpp>
#include <Thor/lld/interface/gpio/gpio.hpp>

#if defined( THOR_HLD_GPIO )

namespace Thor::GPIO
{
  static size_t s_driver_initialized;

  Chimera::Status_t initialize()
  {
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::GPIO::initialize();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }


  Driver::Driver() : driver( nullptr ) 
  {
    initSettings.clear();
  }

  Driver::~Driver()
  {
    free( driver );
  }

  Chimera::Status_t Driver::init( const Chimera::GPIO::PinInit &pinInit, const size_t timeout )
  {
    using namespace Thor::LLD::GPIO;
    
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;
    initSettings = pinInit;

    switch ( pinInit.accessMode )
    {
      case Chimera::Hardware::AccessMode::BARE_METAL:
        driver = new Thor::LLD::GPIO::DriverBare();
        break;
        
      case Chimera::Hardware::AccessMode::THREADED:
        driver = new Thor::LLD::GPIO::DriverThreaded();
        break;
        
      case Chimera::Hardware::AccessMode::ATOMIC:
        driver = new Thor::LLD::GPIO::DriverAtomic();
        break;
        
      default:
        driver = nullptr;
        result = Chimera::CommonStatusCodes::FAIL;
        return result;
        break;
    };
    
    driver->attach( PortToInstanceMap.find( pinInit.port )->second );

    result = setMode( pinInit.drive, pinInit.pull, timeout );

    return result;
  }

  Chimera::Status_t Driver::init( const Chimera::GPIO::Port port, const uint8_t pin, const size_t timeout )
  {
    // TODO
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull, const size_t timeout )
  {
    /*------------------------------------------------
    Set the basic IO mode type
    ------------------------------------------------*/
    auto result = driver->driveSet( initSettings.pin, drive, timeout );

    /*------------------------------------------------
    Configure the pullup/pulldown resistors
    ------------------------------------------------*/
    result |= driver->pullSet( initSettings.pin, pull, timeout );

    /*------------------------------------------------
    Configure the GPIO speed
    ------------------------------------------------*/
    result |= driver->speedSet( initSettings.pin, Thor::LLD::GPIO::Speed::HIGH, timeout );

    /*------------------------------------------------
    Configure the alternate function options
    ------------------------------------------------*/
    if ( ( drive == Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN ) || ( drive == Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL ) ) 
    {
      result |= driver->alternateFunctionSet( initSettings.pin, initSettings.alternate, timeout );
    }

    return result;
  }

  Chimera::Status_t Driver::setState( const Chimera::GPIO::State state, const size_t timeout )
  {
    return driver->write( initSettings.pin, state, timeout );
  }

  Chimera::Status_t Driver::getState( Chimera::GPIO::State &state, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::toggle( const size_t timeout )
  {
    // TODO
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

#endif  /* THOR_DRIVER_GPIO */
}