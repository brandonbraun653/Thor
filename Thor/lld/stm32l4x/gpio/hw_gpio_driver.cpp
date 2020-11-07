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
#include <Chimera/utility>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/exti/exti_intf.hpp>
#include <Thor/lld/interface/gpio/gpio_prv_data.hpp>
#include <Thor/lld/interface/gpio/gpio_intf.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32l4x/system/hw_sys_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  static Driver s_gpio_drivers[ NUM_GPIO_PERIPHS ];


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();

    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_gpio_drivers, ARRAY_COUNT( s_gpio_drivers ) ) )
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
    if ( auto idx = getResourceIndex( port ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_gpio_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Low Level Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr )
  {
  }


  Driver::~Driver()
  {
  }


  void Driver::attach( RegisterMap *const peripheral )
  {
    mPeriph = peripheral;
    clockEnable();
  }


  void Driver::clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::getPeripheralClock();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( mPeriph ) );

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_GPIO, index );
  }


  void Driver::clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::getPeripheralClock();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( mPeriph ) );

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_GPIO, index );
  }


  Chimera::Status_t Driver::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive )
  {
    /* Determine how far to shift into the register based on the config bit width */
    Reg32_t const shift_val = pin * MODER_CFG_X_WID;

    /*------------------------------------------------
    1. Read the current state of the register
    2. Clear the appropriate bits using a shifted mask
    3. Assign bits from (2) with new value, masked appropriately
    4. Push into the device register
    ------------------------------------------------*/
    Reg32_t tmp = mPeriph->MODER;
    tmp &= ~( MODER_CFG_X_MSK << shift_val );
    tmp |= ( ConfigMap::ModeMap[ static_cast<size_t>( drive ) ] & MODER_CFG_X_MSK ) << shift_val;
    mPeriph->MODER = tmp;

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::speedSet( const uint8_t pin, const Thor::LLD::GPIO::Speed speed )
  {
    /* Determine how far to shift into the register based on the config bit width */
    Reg32_t const shift_val = pin * OSPEEDR_CFG_X_WID;

    /*------------------------------------------------
    1. Read the current state of the register
    2. Clear the appropriate bits using a shifted mask
    3. Assign bits from (2) with new value, masked appropriately
    4. Push into the device register
    ------------------------------------------------*/
    Reg32_t tmp = mPeriph->OSPEEDR;
    tmp &= ~( OSPEEDR_CFG_X_MSK << shift_val );
    tmp |= ( ConfigMap::SpeedMap[ static_cast<size_t>( speed ) ] & OSPEEDR_CFG_X_MSK ) << shift_val;
    mPeriph->OSPEEDR = tmp;

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull )
  {
    /* Determine how far to shift into the register based on the config bit width */
    Reg32_t const shift_val = pin * PUPDR_CFG_X_WID;

    /*------------------------------------------------
    1. Read the current state of the register
    2. Clear the appropriate bits using a shifted mask
    3. Assign bits from (2) with new value, masked appropriately
    4. Push into the device register
    ------------------------------------------------*/
    Reg32_t tmp = mPeriph->PUPDR;
    tmp &= ~( PUPDR_CFG_X_MSK << shift_val );
    tmp |= ( ConfigMap::PullMap[ static_cast<size_t>( pull ) ] & PUPDR_CFG_X_MSK ) << shift_val;
    mPeriph->PUPDR = tmp;

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::write( const uint8_t pin, const Chimera::GPIO::State state )
  {
    /*------------------------------------------------
    Atomically set/clr the appropriate bit
    ------------------------------------------------*/
    if ( static_cast<bool>( state ) )
    {
      /* The lower 16 bits control the "set" functionality */
      mPeriph->BSRR = static_cast<Reg32_t>( 1u << pin ) & 0x0000FFFF;
    }
    else
    {
      /* The upper 16 bits control the "clr" functionality */
      auto x        = static_cast<Reg32_t>( 1u << ( pin + 16 ) ) & 0xFFFF0000;
      mPeriph->BSRR = x;
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::alternateFunctionSet( const uint8_t pin, const Chimera::GPIO::Alternate val )
  {
    /*------------------------------------------------
    Initialize some working variables
    ------------------------------------------------*/
    const auto port       = getPort( reinterpret_cast<std::uintptr_t>( mPeriph ) );
    uint64_t temp         = 0u;
    const uint64_t offset = pin * AFR_CFG_X_WID;
    const uint64_t mask   = AFR_CFG_X_MSK;
    const uint64_t AFcfg  = static_cast<uint64_t>( findAlternateFunction( port, pin, val ) );

    /*------------------------------------------------
    64-bit wide read-modify-write sequence to AFRL & AFRH
    ------------------------------------------------*/
    temp = mPeriph->AFR;
    temp &= ~( mask << offset );
    temp |= ( AFcfg & mask ) << offset;
    mPeriph->AFR = temp;

    return Chimera::Status::OK;
  }


  Chimera::GPIO::State Driver::read( const uint8_t pin )
  {
    /* Read the input data register and mask off the desired bit */
    const bool state = ( mPeriph->IDR & ( 1u << pin ) ) >> pin;

    if ( state )
    {
      return Chimera::GPIO::State::HIGH;
    }
    else
    {
      return Chimera::GPIO::State::LOW;
    }
  }


  Chimera::GPIO::Drive Driver::driveGet( const uint8_t pin )
  {
    /* Determine how far to shift into the register based on the config bit width */
    Reg32_t const shift_val    = pin * MODER_CFG_X_WID;
    Reg32_t const shifted_mask = MODER_CFG_X_MSK << shift_val;

    /* Read the current configuration value and shift it to the zero-th bit*/
    Reg32_t cfg_setting = ( mPeriph->MODER & shifted_mask ) >> shift_val;

    /*------------------------------------------------
    Iterate over the possible configuration options and return the
    first one that matches. Otherwise we don't know what this is.
    ------------------------------------------------*/
    for ( const auto &cfg_option : ConfigMap::ModeMap )
    {
      if ( cfg_option == cfg_setting )
      {
        return static_cast<Chimera::GPIO::Drive>( cfg_option );
      }
    }

    return Chimera::GPIO::Drive::UNKNOWN_DRIVE;
  }


  Thor::LLD::GPIO::Speed Driver::speedGet( const uint8_t pin )
  {
    /* Determine how far to shift into the register based on the config bit width */
    Reg32_t const shift_val    = pin * OSPEEDR_CFG_X_WID;
    Reg32_t const shifted_mask = OSPEEDR_CFG_X_MSK << shift_val;

    /* Read the current configuration value and shift it to the zero-th bit*/
    Reg32_t cfg_setting = ( mPeriph->OSPEEDR & shifted_mask ) >> shift_val;

    /*------------------------------------------------
    Iterate over the possible configuration options and return the
    first one that matches. Otherwise we don't know what this is.
    ------------------------------------------------*/
    for ( const auto &cfg_option : ConfigMap::SpeedMap )
    {
      if ( cfg_option == cfg_setting )
      {
        return static_cast<Thor::LLD::GPIO::Speed>( cfg_option );
      }
    }

    return Thor::LLD::GPIO::Speed::UNKNOWN_SPEED;
  }


  Chimera::GPIO::Pull Driver::pullGet( const uint8_t pin )
  {
    /* Determine how far to shift into the register based on the config bit width */
    Reg32_t const shift_val    = pin * PUPDR_CFG_X_WID;
    Reg32_t const shifted_mask = PUPDR_CFG_X_MSK << shift_val;

    /* Read the current configuration value and shift it to the zero-th bit*/
    Reg32_t cfg_setting = ( mPeriph->PUPDR & shifted_mask ) >> shift_val;

    /*------------------------------------------------
    Iterate over the possible configuration options and return the
    first one that matches. Otherwise we don't know what this is.
    ------------------------------------------------*/
    for ( const auto &cfg_option : ConfigMap::PullMap )
    {
      if ( cfg_option == cfg_setting )
      {
        return static_cast<Chimera::GPIO::Pull>( cfg_option );
      }
    }

    return Chimera::GPIO::Pull::UNKNOWN_PULL;
  }


  Chimera::GPIO::Alternate Driver::alternateFunctionGet( const uint8_t pin )
  {
    // Currently not supported
    return Chimera::GPIO::Alternate::NONE;
  }


  Chimera::Status_t Driver::attachInterrupt( const uint8_t pin, Chimera::Function::vGeneric &func,
                                             const Chimera::EXTI::EdgeTrigger trigger )
  {
    /*-------------------------------------------------
    Derive the GPIO port and EXTI line being used
    -------------------------------------------------*/
    auto port = getPort( reinterpret_cast<std::uintptr_t>( mPeriph ) );
    auto line = findEventLine( port, pin );

    /*-------------------------------------------------
    Select the proper source for the interrupt line
    -------------------------------------------------*/
    SYS::configureExtiSource( port, pin );

    /*-------------------------------------------------
    Configure the EXTI hardware to enable the interrupt
    -------------------------------------------------*/
    return EXTI::attach( line, trigger, func );
  }


  void Driver::detachInterrupt( const uint8_t pin )
  {
    alkjsdflkjs
  }
}    // namespace Thor::LLD::GPIO

#endif /* TARGET_STM32L4 && THOR_DRIVER_GPIO */
