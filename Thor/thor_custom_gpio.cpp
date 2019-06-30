/********************************************************************************
 *   File Name:
 *    thor_custom_gpio.cpp
 *
 *   Description:
 *    Implements the custom driver variant of the Thor GPIO interface.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/


/* Thor Includes */
#include <Thor/gpio.hpp>
#include <Thor/drivers/GPIO.hpp>

namespace Thor::GPIO
{
  GPIOClass::GPIOClass()
  {
    
  }

  GPIOClass::~GPIOClass()
  {
    
  }

  Chimera::Status_t GPIOClass::init( const Chimera::GPIO::PinInit &pinInit )
  {
    //  gpio.attach( GPIOA_PERIPH );
    //  gpio.driveSet( pin, Chimera::GPIO::Drive::OUTPUT_PUSH_PULL );
    //  gpio.pullSet( pin, Chimera::GPIO::Pull::NO_PULL );
    //  gpio.speedSet( pin, Thor::Driver::GPIO::Speed::FAST );
    //  gpio.alternateFunctionSet( pin, AF_NONE );
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GPIOClass::init( const Chimera::GPIO::Port port, const uint8_t pin )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GPIOClass::setMode( const Chimera::GPIO::Drive drive, const bool pullup )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GPIOClass::setState( const Chimera::GPIO::State state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GPIOClass::getState( Chimera::GPIO::State &state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t GPIOClass::toggle()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }
}