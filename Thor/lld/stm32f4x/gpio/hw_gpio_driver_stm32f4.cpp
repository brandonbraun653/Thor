/********************************************************************************
 *  File Name:
 *    hw_gpio_driver_stm32f4.cpp
 *
 *  Description:
 *    Implements the low level driver for the GPIO peripheral
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/utilities.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_driver.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_mapping.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_prj.hpp>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
  static std::array<IGPIO_sPtr, NUM_GPIO_PERIPHS> s_gpio_drivers;

  /*-------------------------------------------------
  LLD->HLD Interface Implementation
  -------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();
    initializeMapping();

    return Chimera::CommonStatusCodes::OK;
  }

  IGPIO_sPtr getDriver( const size_t channel )
  {
    if ( !( channel < NUM_GPIO_PERIPHS ) )
    {
      return nullptr;
    }
    else if ( !s_gpio_drivers[ channel ] )
    {
      s_gpio_drivers[ channel ] = std::make_shared<Driver>();
      s_gpio_drivers[ channel ]->attach( PeripheralList[ channel ] );
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
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_GPIO, index );
  }

  void Driver::clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::getSystemPeripheralController();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_GPIO, index );
  }

  Chimera::Status_t Driver::driveSet( const uint8_t pin, const Chimera::GPIO::Drive drive, const size_t timeout )
  {
    auto const shift_val = pin * MODER_CFG_X_WID;
    auto tmp             = periph->MODER;

    /*------------------------------------------------
    Use read-modify-write
    ------------------------------------------------*/
    tmp &= ~( MODER_CFG_X_MSK << shift_val );
    tmp |= ( ModeMap[ static_cast<size_t>( drive ) ] & MODER_CFG_X_MSK ) << shift_val;
    periph->MODER = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::speedSet( const uint8_t pin, const Thor::LLD::GPIO::Speed speed, const size_t timeout )
  {
    auto const shift_val = pin * OSPEEDR_CFG_X_WID;
    auto tmp             = periph->OSPEEDR;

    tmp &= ~( OSPEEDR_CFG_X_MSK << shift_val );
    tmp |= ( SpeedMap[ static_cast<size_t>( speed ) ] & OSPEEDR_CFG_X_MSK ) << shift_val;
    periph->OSPEEDR = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::pullSet( const uint8_t pin, const Chimera::GPIO::Pull pull, const size_t timeout )
  {
    auto const shift_val = pin * PUPDR_CFG_X_WID;
    auto tmp             = periph->PUPDR;

    tmp &= ~( PUPDR_CFG_X_MSK << shift_val );
    tmp |= ( PullMap[ static_cast<size_t>( pull ) ] & PUPDR_CFG_X_MSK ) << shift_val;
    periph->PUPDR = tmp;

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::write( const uint8_t pin, const Chimera::GPIO::State state, const size_t timeout )
  {
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

  Chimera::Status_t Driver::alternateFunctionSet( const uint8_t pin, const size_t val, const size_t timeout )
  {
    static_assert( sizeof( uint64_t ) == sizeof( RegisterMap::AFR ), "Invalid register memory map" );

    uint64_t temp         = 0u;
    const uint64_t offset = pin * AFR_CFG_X_WID;
    const uint64_t mask   = AFR_CFG_X_MSK;
    const uint64_t AFcfg  = val;

    /*------------------------------------------------
    64-bit wide read-modify-write sequence to AFRL & AFRH
    ------------------------------------------------*/
    temp = periph->AFR;
    temp &= ~( mask << offset );
    temp |= ( AFcfg & mask ) << offset;
    periph->AFR = temp;

    return Chimera::CommonStatusCodes::OK;
  }

  size_t Driver::read( const size_t timeout )
  {
    return periph->IDR;
  }

  size_t Driver::driveGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }

  size_t Driver::speedGet( const uint8_t pin, const size_t timeout )
  {
    auto const shift_val   = pin * OSPEEDR_CFG_X_WID;
    auto const current_val = periph->OSPEEDR & ( OSPEEDR_CFG_X_MSK << shift_val );

    return static_cast<OPT_OSPEEDR>( current_val >> shift_val );
  }

  size_t Driver::pullGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }

  size_t Driver::alternateFunctionGet( const uint8_t pin, const size_t timeout )
  {
    return 0;
  }
}    // namespace Thor::LLD::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */