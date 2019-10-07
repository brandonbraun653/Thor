/********************************************************************************
 *   File Name:
 *    thor_custom_gpio.cpp
 *
 *   Description:
 *    Implements the custom driver variant of the Thor GPIO interface.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>

/* Thor Includes */
#include <Thor/gpio.hpp>
#include <Thor/drivers/GPIO.hpp>

namespace Thor::GPIO
{
#if ( THOR_DRIVER_GPIO == 1 ) && ( THOR_DRIVER_GPIO == 1 )

  GPIOClass::GPIOClass() : driver( nullptr ) 
  {
    memset( &initSettings, 0, sizeof( initSettings ) );
  }

  GPIOClass::~GPIOClass()
  {
    free( driver );
  }

  Chimera::Status_t GPIOClass::init( const Chimera::GPIO::PinInit &pinInit, const size_t timeout )
  {
    using namespace Thor::Driver::GPIO;
    
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;
    initSettings = pinInit;

    switch ( pinInit.accessMode )
    {
      case Chimera::Hardware::AccessMode::BARE_METAL:
        driver = new Thor::Driver::GPIO::DriverBare();
        break;
        
      case Chimera::Hardware::AccessMode::THREADED:
        driver = new Thor::Driver::GPIO::DriverThreaded();
        break;
        
      case Chimera::Hardware::AccessMode::ATOMIC:
        driver = new Thor::Driver::GPIO::DriverAtomic();
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

  Chimera::Status_t GPIOClass::init( const Chimera::GPIO::Port port, const uint8_t pin, const size_t timeout )
  {
    // TODO
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GPIOClass::setMode( const Chimera::GPIO::Drive drive, const Chimera::GPIO::Pull pull, const size_t timeout )
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
    Configure the alternate function options
    ------------------------------------------------*/
    if ( ( drive == Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN ) || ( drive == Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL ) ) 
    {
      result |= driver->alternateFunctionSet( initSettings.pin, initSettings.alternate, timeout );
    }

    return result;
  }

  Chimera::Status_t GPIOClass::setState( const Chimera::GPIO::State state, const size_t timeout )
  {
    return driver->write( initSettings.pin, state, timeout );
  }

  Chimera::Status_t GPIOClass::getState( Chimera::GPIO::State &state, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GPIOClass::toggle( const size_t timeout )
  {
    // TODO
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

#endif  /* THOR_DRIVER_GPIO */
}