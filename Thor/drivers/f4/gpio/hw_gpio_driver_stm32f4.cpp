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
#include <Thor/drivers/f4/gpio/hw_gpio_mapping.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_prj.hpp>
#include <Thor/drivers/f4/gpio/hw_gpio_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>

namespace Thor::Driver::GPIO
{
  /*-----------------------------------------------------
  Bare Metal Implementation
  -----------------------------------------------------*/
  DriverBare::DriverBare() : periph( nullptr ), accessIndex( 0 )
  {
  }

  DriverBare::~DriverBare()
  {
  }

  void DriverBare::attach( volatile RegisterMap *const peripheral )
  {
    periph = peripheral;

    /*------------------------------------------------
    Cache the GPIO instance accessor for mapping functions
    ------------------------------------------------*/
    auto portValue = InstanceToPortMap.find( reinterpret_cast<std::uintptr_t>( peripheral ) )->second;
    accessIndex    = PortToIteratorMap.find( portValue )->second;
  }

  void DriverBare::clockEnable()
  {
    auto rccGPIO = Thor::Driver::RCC::GPIOPeriph::get();
    rccGPIO->enableClock( accessIndex );
  }

  void DriverBare::clockDisable()
  {
    auto rccGPIO = Thor::Driver::RCC::GPIOPeriph::get();
    rccGPIO->disableClock( accessIndex );
  }

  Chimera::Status_t DriverBare::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )
    auto const shift_val = pin * MODER_CFG_X_WID;
    auto tmp             = periph->MODER;

    /*------------------------------------------------
    Use read-modify-write
    ------------------------------------------------*/
    tmp &= ~( MODER_CFG_X_MSK << shift_val );
    tmp |= ( ModeMap.find( static_cast<uint8_t>( drive ) )->second & MODER_CFG_X_MSK ) << shift_val;
    periph->MODER = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DriverBare::speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )
    auto const shift_val = pin * OSPEEDR_CFG_X_WID;
    auto tmp             = periph->OSPEEDR;

    tmp &= ~( OSPEEDR_CFG_X_MSK << shift_val );
    tmp |= ( SpeedMap.find( static_cast<uint8_t>( speed ) )->second & OSPEEDR_CFG_X_MSK ) << shift_val;
    periph->OSPEEDR = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DriverBare::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )
    auto const shift_val = pin * PUPDR_CFG_X_WID;
    auto tmp             = periph->PUPDR;

    tmp &= ~( PUPDR_CFG_X_MSK << shift_val );
    tmp |= ( PullMap.find( static_cast<uint8_t>( pull ) )->second & PUPDR_CFG_X_MSK ) << shift_val;
    periph->PUPDR = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DriverBare::write( const uint8_t pin, const Chimera::GPIO::State state )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )

    auto temp = periph->ODR;

    if ( static_cast<bool>( state ) )
    {
      temp |= 1u << pin;
    }
    else
    {
      temp &= ~( 1u << pin );
    }

    periph->ODR = temp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DriverBare::alternateFunctionSet( const uint8_t pin, const size_t val )
  {
    static_assert( sizeof( uint64_t ) == sizeof( RegisterMap::AFR ), "Invalid register memory map" );

    uint64_t temp         = 0u;
    const uint64_t offset = pin * AFR_CFG_X_WID;

    /*------------------------------------------------
    64-bit wide read-modify-write sequence to AFRL & AFRH
    ------------------------------------------------*/
    temp = *periph->AFR;
    temp &= ~( AFR_CFG_X_MSK << offset );
    temp |= ( val & AFR_CFG_X_MSK ) << offset;
    *periph->AFR = temp;

    return Chimera::CommonStatusCodes::OK;
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

    auto const shift_val   = pin * OSPEEDR_CFG_X_WID;
    auto const current_val = periph->OSPEEDR & ( OSPEEDR_CFG_X_MSK << shift_val );

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

  void DriverThreaded::attach( volatile RegisterMap *const peripheral )
  {
    bareMetalDriver.attach( peripheral );
  }

  Chimera::Status_t DriverThreaded::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( lock( timeout ) != Chimera::CommonStatusCodes::OK )
    {
      result = Chimera::CommonStatusCodes::LOCKED;
    }
    else
    {
      result = bareMetalDriver.driveSet( pin, drive );
      unlock();
    }

    return result;
  }

  Chimera::Status_t DriverThreaded::speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( lock( timeout ) != Chimera::CommonStatusCodes::OK )
    {
      result = Chimera::CommonStatusCodes::LOCKED;
    }
    else
    {
      result = bareMetalDriver.speedSet( pin, speed );
      unlock();
    }

    return result;
  }

  Chimera::Status_t DriverThreaded::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( lock( timeout ) != Chimera::CommonStatusCodes::OK )
    {
      result = Chimera::CommonStatusCodes::LOCKED;
    }
    else
    {
      result = bareMetalDriver.pullSet( pin, pull );
      unlock();
    }

    return result;
  }

  Chimera::Status_t DriverThreaded::write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( lock( timeout ) != Chimera::CommonStatusCodes::OK )
    {
      result = Chimera::CommonStatusCodes::LOCKED;
    }
    else
    {
      result = bareMetalDriver.write( pin, state );
      unlock();
    }

    return result;
  }

  Chimera::Status_t DriverThreaded::alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( lock( timeout ) != Chimera::CommonStatusCodes::OK )
    {
      result = Chimera::CommonStatusCodes::LOCKED;
    }
    else
    {
      result = bareMetalDriver.alternateFunctionSet( pin, val );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::read( const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.read();
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::driveGet( const uint8_t pin, const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.driveGet( pin );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::speedGet( const uint8_t pin, const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.speedGet( pin );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::pullGet( const uint8_t pin, const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.pullGet( pin );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::alternateFunctionGet( const uint8_t pin, const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.alternateFunctionGet( pin );
      unlock();
    }

    return result;
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
