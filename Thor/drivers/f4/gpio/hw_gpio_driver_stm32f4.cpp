/********************************************************************************
 *   File Name:
 *    hw_gpio_driver_stm32f4.cpp
 *
 *   Description:
 *    Implements the low level driver for the GPIO peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/assert.hpp>

/* Driver Includes */
#include <Thor/drivers/common/cortex-m4/utilities.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_driver.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_prj.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_types.hpp>


namespace Thor::Driver::GPIO
{
  /*-----------------------------------------------------
  Bare Metal Implementation
  -----------------------------------------------------*/
  DriverBare::DriverBare() : periph( nullptr )
  {
  }

  DriverBare::~DriverBare()
  {
  }

  void DriverBare::attach( RegisterMap *const peripheral )
  {
    periph = peripheral;
  }

  Chimera::Status_t DriverBare::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverBare::speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverBare::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverBare::write( const uint8_t pin, const Chimera::GPIO::State state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverBare::alternateFunctionSet( const uint8_t pin, const size_t val )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t DriverBare::read()
  {
    return periph->IDR;
  }

  size_t DriverBare::driveGet( const uint8_t pin )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )
    return 0;
  }

  size_t DriverBare::speedGet( const uint8_t pin )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )

    const auto shift_val = pin * OSPEEDR_CFG_X_WID;
    const auto current_val = periph->OSPEEDR & ( OSPEEDR_CFG_X_MSK << shift_val );

    return static_cast<OPT_OSPEEDR>( current_val >> shift_val );
  }

  size_t DriverBare::pullGet( const uint8_t pin )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )



    return 0;
  }

  size_t DriverBare::alternateFunctionGet( const uint8_t pin )
  {
    return 0;
  }

  /*-----------------------------------------------------
  Threaded Implementation
  -----------------------------------------------------*/
  DriverThreaded::DriverThreaded()
  {
  }

  DriverThreaded::~DriverThreaded()
  {
  }

  void DriverThreaded::attach( RegisterMap *const peripheral )
  {
    gpio.attach( peripheral );
    mutex = Chimera::Threading::createRecursiveMutex();
  }

  Chimera::Status_t DriverThreaded::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverThreaded::speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverThreaded::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverThreaded::write( const uint8_t pin, const size_t val, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverThreaded::alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t DriverThreaded::read( const size_t timeout )
  {
    return 0;
  }

  size_t DriverThreaded::driveGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }

  size_t DriverThreaded::speedGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }

  size_t DriverThreaded::pullGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }

  size_t DriverThreaded::alternateFunctionGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }


  /*-----------------------------------------------------
  Atomic Implementation
  -----------------------------------------------------*/
  DriverAtomic::DriverAtomic()
  {

  }

  DriverAtomic::~DriverAtomic()
  {

  }

  void attach( RegisterMap *const peripheral )
  {
    
  }

  Chimera::Status_t driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t write( const uint8_t pin, const Chimera::GPIO::State state )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t alternateFunctionSet( const uint8_t pin, const size_t val )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t read()
  {
    return 0;
  }

  size_t driveGet( const uint8_t pin )
  {
    return 0;
  }

  size_t speedGet( const uint8_t pin )
  {
    return 0;
  }

  size_t pullGet( const uint8_t pin )
  {
    return 0;
  }

  size_t alternateFunctionGet( const uint8_t pin )
  {
    return 0;
  }
}    // namespace Thor::Driver::GPIO
