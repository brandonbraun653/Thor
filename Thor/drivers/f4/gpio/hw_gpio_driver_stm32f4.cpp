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

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

namespace Thor::Driver::GPIO
{
  bool isGPIO( const std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
      }
    }

    return result;
  }

  /*-----------------------------------------------------
  Bare Metal Implementation
  -----------------------------------------------------*/
  DriverBare::DriverBare() : periph( nullptr )
  {
  }

  DriverBare::~DriverBare()
  {
  }

  void DriverBare::attach( volatile RegisterMap *const peripheral )
  {
    periph = peripheral;

    /*------------------------------------------------
    Perform any initialization steps needed
    ------------------------------------------------*/
    clockEnable();
  }

  void DriverBare::clockEnable()
  {
    auto rcc = Thor::Driver::RCC::PeripheralController::get();
    rcc->enableClock( reinterpret_cast<std::uintptr_t>( periph ) );
  }

  void DriverBare::clockDisable()
  {
    auto rcc = Thor::Driver::RCC::PeripheralController::get();
    rcc->disableClock( reinterpret_cast<std::uintptr_t>( periph ) );
  }

  Chimera::Status_t DriverBare::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )
    auto const shift_val = pin * MODER_CFG_X_WID;
    auto tmp             = periph->MODER;

    /*------------------------------------------------
    Use read-modify-write
    ------------------------------------------------*/
    tmp &= ~( MODER_CFG_X_MSK << shift_val );
    tmp |= ( ModeMap.find( drive )->second & MODER_CFG_X_MSK ) << shift_val;
    periph->MODER = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DriverBare::speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )
    auto const shift_val = pin * OSPEEDR_CFG_X_WID;
    auto tmp             = periph->OSPEEDR;

    tmp &= ~( OSPEEDR_CFG_X_MSK << shift_val );
    tmp |= ( SpeedMap.find( speed )->second & OSPEEDR_CFG_X_MSK ) << shift_val;
    periph->OSPEEDR = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DriverBare::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )
    auto const shift_val = pin * PUPDR_CFG_X_WID;
    auto tmp             = periph->PUPDR;

    tmp &= ~( PUPDR_CFG_X_MSK << shift_val );
    tmp |= ( PullMap.find( pull )->second & PUPDR_CFG_X_MSK ) << shift_val;
    periph->PUPDR = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t DriverBare::write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout )
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

  Chimera::Status_t DriverBare::alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout )
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

  size_t DriverBare::read( const size_t timeout )
  {
    return periph->IDR;
  }

  size_t DriverBare::driveGet( const uint8_t pin, const size_t timeout )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )
    return 0;
  }

  size_t DriverBare::speedGet( const uint8_t pin, const size_t timeout )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )

    auto const shift_val   = pin * OSPEEDR_CFG_X_WID;
    auto const current_val = periph->OSPEEDR & ( OSPEEDR_CFG_X_MSK << shift_val );

    return static_cast<OPT_OSPEEDR>( current_val >> shift_val );
  }

  size_t DriverBare::pullGet( const uint8_t pin, const size_t timeout )
  {
    DBG_ASSERT( pin < MAX_NUM_PINS )


    return 0;
  }

  size_t DriverBare::alternateFunctionGet( const uint8_t pin, const size_t timeout )
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

  void DriverThreaded::clockEnable()
  {
    bareMetalDriver.clockEnable();
  }

  void DriverThreaded::clockDisable()
  {
    bareMetalDriver.clockDisable();
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
      result = bareMetalDriver.driveSet( pin, drive, timeout );
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
      result = bareMetalDriver.speedSet( pin, speed, timeout );
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
      result = bareMetalDriver.pullSet( pin, pull, timeout );
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
      result = bareMetalDriver.write( pin, state, timeout );
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
      result = bareMetalDriver.alternateFunctionSet( pin, val, timeout );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::read( const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.read( timeout );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::driveGet( const uint8_t pin, const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.driveGet( pin, timeout );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::speedGet( const uint8_t pin, const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.speedGet( pin, timeout );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::pullGet( const uint8_t pin, const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.pullGet( pin, timeout );
      unlock();
    }

    return result;
  }

  size_t DriverThreaded::alternateFunctionGet( const uint8_t pin, const size_t timeout )
  {
    size_t result = defaultError;

    if ( lock( timeout ) == Chimera::CommonStatusCodes::OK )
    {
      result = bareMetalDriver.alternateFunctionGet( pin, timeout );
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

  void DriverAtomic::attach( volatile RegisterMap *const peripheral )
  {
  }

  void DriverAtomic::clockEnable()
  {
  }

  void DriverAtomic::clockDisable()
  {
  }

  Chimera::Status_t DriverAtomic::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverAtomic::speedSet( const uint8_t pin, const Thor::Driver::GPIO::Speed speed, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverAtomic::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverAtomic::write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t DriverAtomic::alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  size_t DriverAtomic::read( const size_t timeout )
  {
    return 0;
  }

  size_t DriverAtomic::driveGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }

  size_t DriverAtomic::speedGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }

  size_t DriverAtomic::pullGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }

  size_t DriverAtomic::alternateFunctionGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }
}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */