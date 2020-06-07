/********************************************************************************
 *  File Name:
 *    hw_gpio_driver_STM32L4.cpp
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
//#include <Thor/lld/common/cortex-m4/utilities.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_driver.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_mapping.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32l4x/gpio/hw_gpio_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
  static std::array<IDriver_sPtr, NUM_GPIO_PERIPHS> s_gpio_drivers;

  /*-------------------------------------------------
  LLD->HLD Interface Implementation
  -------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();
    initializeMapping();

    return Chimera::CommonStatusCodes::OK;
  }

  IDriver_sPtr getDriver( const size_t channel )
  {
    if ( !( channel < NUM_GPIO_PERIPHS ) )
    {
      return nullptr;
    }
    else if ( !s_gpio_drivers[ channel ] )
    {
      s_gpio_drivers[ channel ] = std::make_shared<Driver>();
      s_gpio_drivers[ channel ]->attach( reinterpret_cast<RegisterMap*>( PeripheralList[ channel ] ) );
    }

    return s_gpio_drivers[ channel ];
  }

  size_t availableChannels()
  {
    return NUM_GPIO_PERIPHS;
  }

  /*-------------------------------------------------
  Private LLD Function Implementation
  -------------------------------------------------*/
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
  Low Level Driver Implementation
  -----------------------------------------------------*/
  Driver::Driver() : periph( nullptr )
  {
  }

  Driver::~Driver()
  {
  }

  void Driver::attach( RegisterMap *const peripheral )
  {
    periph = peripheral;
    clockEnable();
  }

  void Driver::clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::getSystemPeripheralController();
    auto index = InstanceToResourceIndex.at( reinterpret_cast<std::uintptr_t>( periph ) ).second;

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_GPIO, index );
  }

  void Driver::clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::getSystemPeripheralController();
    auto index = InstanceToResourceIndex.at( reinterpret_cast<std::uintptr_t>( periph ) ).second;

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
    Reg32_t tmp = periph->MODER;
    tmp &= ~( MODER_CFG_X_MSK << shift_val );
    tmp |= ( ModeMap[ static_cast<size_t>( drive ) ] & MODER_CFG_X_MSK ) << shift_val;
    periph->MODER = tmp;

    return Chimera::CommonStatusCodes::OK;
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
    Reg32_t tmp = periph->OSPEEDR;
    tmp &= ~( OSPEEDR_CFG_X_MSK << shift_val );
    tmp |= ( SpeedMap[ static_cast<size_t>( speed ) ] & OSPEEDR_CFG_X_MSK ) << shift_val;
    periph->OSPEEDR = tmp;

    return Chimera::CommonStatusCodes::OK;
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
    Reg32_t tmp = periph->PUPDR;
    tmp &= ~( PUPDR_CFG_X_MSK << shift_val );
    tmp |= ( PullMap[ static_cast<size_t>( pull ) ] & PUPDR_CFG_X_MSK ) << shift_val;
    periph->PUPDR = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::write( const uint8_t pin, const Chimera::GPIO::State state )
  {
    /*------------------------------------------------
    Atomically set/clr the appropriate bit 
    ------------------------------------------------*/
    if ( static_cast<bool>( state ) )
    {
      /* The lower 16 bits control the "set" functionality */
      periph->BSRR = static_cast<Reg32_t>( 1u << pin ) & 0x0000FFFF;
    }
    else
    {
      /* The upper 16 bits control the "clr" functionality */
      auto x       = static_cast<Reg32_t>( 1u << ( pin + 16 ) ) & 0xFFFF0000;
      periph->BSRR = x;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::alternateFunctionSet( const uint8_t pin, const Chimera::GPIO::Alternate val )
  {
    static_assert( sizeof( uint64_t ) == sizeof( RegisterMap::AFR ), "Invalid register memory map" );

    /*------------------------------------------------
    Determine the alternate function configuration by 
    going through all the lovely lookup tables.
    ------------------------------------------------*/
    const auto instanceToPinMap = InstanceToAlternateMap.at( periph );
    if ( !instanceToPinMap.second )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    const auto pinMap      = reinterpret_cast<const PinToAFMap *>( instanceToPinMap.second );
    const auto afConfigMap = pinMap->at( pin );

    const AFToReg *registerMap     = reinterpret_cast<const AFToReg *>( afConfigMap.second );
    const auto afRegisterConfigMap = registerMap->at( val );


    uint64_t temp         = 0u;
    const uint64_t offset = pin * AFR_CFG_X_WID;
    const uint64_t mask   = AFR_CFG_X_MSK;
    const uint64_t AFcfg  = afRegisterConfigMap.second;

    /*------------------------------------------------
    64-bit wide read-modify-write sequence to AFRL & AFRH
    ------------------------------------------------*/
    temp = periph->AFR;
    temp &= ~( mask << offset );
    temp |= ( AFcfg & mask ) << offset;
    periph->AFR = temp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::GPIO::State Driver::read( const uint8_t pin )
  {
    /* Read the input data register and mask off the desired bit */
    const bool state = ( periph->IDR & ( 1u << pin ) ) >> pin;

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
    Reg32_t cfg_setting = ( periph->MODER & shifted_mask ) >> shift_val;

    /*------------------------------------------------
    Iterate over the possible configuration options and return the 
    first one that matches. Otherwise we don't know what this is.
    ------------------------------------------------*/
    for ( const auto &cfg_option : ModeMap )
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
    Reg32_t cfg_setting = ( periph->OSPEEDR & shifted_mask ) >> shift_val;

    /*------------------------------------------------
    Iterate over the possible configuration options and return the 
    first one that matches. Otherwise we don't know what this is.
    ------------------------------------------------*/
    for ( const auto &cfg_option : SpeedMap )
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
    Reg32_t cfg_setting = ( periph->PUPDR & shifted_mask ) >> shift_val;

    /*------------------------------------------------
    Iterate over the possible configuration options and return the 
    first one that matches. Otherwise we don't know what this is.
    ------------------------------------------------*/
    for ( const auto &cfg_option : PullMap )
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
    /*------------------------------------------------
    Determine the alternate function configuration by 
    going through all the lovely lookup tables.
    ------------------------------------------------*/
//    AlternateMap::value_type *instanceToPinMap = nullptr;
//    if ( !InstanceToAlternateMap.exists( periph, &instanceToPinMap ) )
//    {
//      return Chimera::GPIO::Alternate::NONE;
//    }
//
//    const PinToAFMap *pinMap             = reinterpret_cast<const PinToAFMap *>( instanceToPinMap->second );
//    PinToAFMap::value_type *afConfigMap = nullptr;
//    if ( !pinMap->exists( pin, &afConfigMap ) )
//    {
//      return Chimera::GPIO::Alternate::NONE;
//    }
//
//
//
//    const uint64_t offset = pin * AFR_CFG_X_WID;
//    const uint64_t mask   = AFR_CFG_X_MSK << offset;
//
//    /*------------------------------------------------
//    64-bit wide read-modify-write sequence to AFRL & AFRH
//    ------------------------------------------------*/
//    Reg8_t currentConfig = static_cast<Reg8_t>( ( periph->AFR & mask ) >> offset );
//    const AFToReg *registerMap = reinterpret_cast<const AFToReg *>( afConfigMap->second );
//
//    const AFToReg::value_type *cfg = registerMap->findWithValue( currentConfig );
//    if ( cfg ) 
//    {
//      return cfg->first;
//    }

    return Chimera::GPIO::Alternate::NONE;
  }

}    // namespace Thor::LLD::GPIO

#endif /* TARGET_STM32L4 && THOR_DRIVER_GPIO */
