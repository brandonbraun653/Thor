/********************************************************************************
 *  File Name:
 *    hw_gpio_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series GPIO hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/gpio/gpio_prv_data.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_types.hpp>

#if defined( TARGET_LLD_TEST ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Sadly this will consume far more memory on larger
  devices. Given that a single pin is the "absolute
  unit" for a GPIO control, this is just how it is.
  -------------------------------------------------*/
  static Driver s_gpio_drivers[ NUM_GPIO_PINS ];

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();

    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_gpio_drivers, NUM_GPIO_PINS ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }

  Driver_rPtr getDriver( const Chimera::GPIO::Port port, const Chimera::GPIO::Pin pin )
  {
    if ( auto idx = getPinResourceIndex( port, pin ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_gpio_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }

  /*-----------------------------------------------------
  Low Level Driver Implementation
  -----------------------------------------------------*/
  Driver::Driver()
  {
  }

  Driver::~Driver()
  {
  }

  void Driver::attach( RegisterMap *const peripheral )
  {
  }

  void Driver::clockEnable()
  {
  }

  void Driver::clockDisable()
  {
  }

  Chimera::Status_t Driver::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive )
  {
    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::speedSet( const uint8_t pin, const Thor::LLD::GPIO::Speed speed )
  {
    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull )
  {
    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::write( const uint8_t pin, const Chimera::GPIO::State state )
  {
    return Chimera::Status::OK;
  }

  Chimera::Status_t Driver::alternateFunctionSet( const uint8_t pin, const Chimera::GPIO::Alternate val )
  {
    return Chimera::Status::OK;
  }

  Chimera::GPIO::State Driver::read( const uint8_t pin )
  {
    return Chimera::GPIO::State::LOW;
  }

  Chimera::GPIO::Drive Driver::driveGet( const uint8_t pin )
  {
    return Chimera::GPIO::Drive::UNKNOWN_DRIVE;
  }

  Thor::LLD::GPIO::Speed Driver::speedGet( const uint8_t pin )
  {
    return Thor::LLD::GPIO::Speed::UNKNOWN_SPEED;
  }

  Chimera::GPIO::Pull Driver::pullGet( const uint8_t pin )
  {
    return Chimera::GPIO::Pull::UNKNOWN_PULL;
  }

  Chimera::GPIO::Alternate Driver::alternateFunctionGet( const uint8_t pin )
  {
    return Chimera::GPIO::Alternate::NONE;
  }

}    // namespace Thor::LLD::GPIO

#endif /* TARGET_STM32L4 && THOR_DRIVER_GPIO */
