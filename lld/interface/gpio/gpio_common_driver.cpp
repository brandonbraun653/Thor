/********************************************************************************
 *  File Name:
 *    gpio_common_driver.cpp
 *
 *  Description:
 *    Common low level driver for the STM32 GPIO. It seems that a lot of devices
 *    share the same core hardware interface.
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/utilities.hpp>
#include <Thor/lld/interface/inc/gpio>
#include <Thor/lld/interface/inc/rcc>
#include <Thor/lld/interface/inc/exti>
#include <Thor/lld/interface/inc/sys>

#if defined( THOR_GPIO ) && ( defined( TARGET_STM32F4 ) || defined( TARGET_STM32L4 ) )

namespace Thor::LLD::GPIO
{
  /*-----------------------------------------------------
  Low Level Driver Implementation
  -----------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::attach( RegisterMap *const mPeripheral )
  {
    mPeriph = mPeripheral;
    return Chimera::Status::OK;
  }


  void Driver::clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( mPeriph ) );

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_GPIO, index );
  }

  void Driver::clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( mPeriph ) );

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_GPIO, index );
  }


  Chimera::Status_t Driver::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive )
  {
    /* Determine how far to shift into the register based on the config bit width */
    Reg32_t const shift_val = pin * MODER_CFG_X_WID;
    Reg32_t tmp = 0;

    /*------------------------------------------------
    1. Read the current state of the register
    2. Clear the appropriate bits using a shifted mask
    3. Assign bits from (2) with new value, masked appropriately
    4. Push into the device register
    ------------------------------------------------*/
    tmp = mPeriph->MODER;
    tmp &= ~( MODER_CFG_X_MSK << shift_val );
    tmp |= ( ConfigMap::ModeMap[ static_cast<size_t>( drive ) ] & MODER_CFG_X_MSK ) << shift_val;
    mPeriph->MODER = tmp;

    /*-------------------------------------------------------------------------
    Set the output drive type
    -------------------------------------------------------------------------*/
    tmp = mPeriph->OTYPER;
    switch( drive )
    {
      case Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN:
      case Chimera::GPIO::Drive::OUTPUT_OPEN_DRAIN:
        tmp |= ( 1u << pin );
        break;

      case Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL:
      case Chimera::GPIO::Drive::OUTPUT_PUSH_PULL:
        tmp &= ~( 1u << pin );
        break;

      default:
        // Nothing to modify
        break;
    };
    mPeriph->OTYPER = tmp;

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

    /*-------------------------------------------------
    Check to make sure the AF was found
    -------------------------------------------------*/
    RT_HARD_ASSERT( AFcfg != BAD_ALT_FUNC );

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
    auto port = getPort( reinterpret_cast<std::uintptr_t>( mPeriph ) );
    auto line = findEventLine( port, pin );

    EXTI::detach( line );
  }

}    // namespace Thor::LLD::GPIO

#endif /* THOR_LLD_GPIO && TARGET_XXXXX */