/********************************************************************************
* File Name:
*   thor_gpio.cpp
*
* Description:
*   Implements the Thor GPIO driver
*
* 2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* Project Includes */
#include <Thor/gpio.hpp>

namespace Thor
{
  namespace GPIO
  {
    using namespace Thor::GPIO;

    static const uint32_t getRCCGPIOMask( const GPIO_TypeDef *const instance )
    {
      auto i = reinterpret_cast<std::uintptr_t>( instance );
      switch ( i )
      {
// STM32F7 PDF: RM0410 pg.181
// STM32F4 PDF: RM0390 pg.143
#if defined( TARGET_STM32F7 ) || defined( TARGET_STM32F4 )
        case GPIOA_BASE:
          return ( 1u << 0 );
          break;

        case GPIOB_BASE:
          return ( 1u << 1 );
          break;

        case GPIOC_BASE:
          return ( 1u << 2 );
          break;

        case GPIOD_BASE:
          return ( 1u << 3 );
          break;

        case GPIOE_BASE:
          return ( 1u << 4 );
          break;

        case GPIOF_BASE:
          return ( 1u << 5 );
          break;

        case GPIOG_BASE:
          return ( 1u << 6 );
          break;

        case GPIOH_BASE:
          return ( 1u << 7 );
          break;
#endif

#if defined( TARGET_STM32F7 )
        case GPIOI_BASE:
          return ( 1u << 8 );
          break;

        case GPIOJ_BASE:
          return ( 1u << 9 );
          break;

        case GPIOK_BASE:
          return ( 1u << 10 );
          break;
#endif

        default:
          return 0u;
          break;
      };
    }



    Chimera::Status_t GPIOClass::init( const Chimera::GPIO::Port port, const uint8_t pin )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      const PinNum stm32_pin = convertPinNum( pin );
      const PinPort stm32_port = convertPort( port );

      if ( ( stm32_pin == PinNum::NOT_A_PIN ) || !stm32_port )
      {
        return Chimera::GPIO::Status::INVAL_FUNC_PARAM;
      }
      else
      {
        pinConfig.GPIOx  = stm32_port;
        pinConfig.pinNum = stm32_pin;

        auto cfg = getHALInit( pinConfig );
        GPIO_Init( pinConfig.GPIOx, &cfg );
      }

      return error;
    }

    Chimera::Status_t GPIOClass::setMode( const Chimera::GPIO::Drive drive, const bool pullup )
    {
      Chimera::Status_t error = Chimera::CommonStatusCodes::OK;

      const PinMode pinMode = convertDrive( drive );
      const PinPull pinPull = pullup ? PinPull::PULLUP : PinPull::NOPULL;

      if ( pinMode == PinMode::UNKNOWN_MODE )
      {
        error = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
      }
      else
      {
        pinConfig.mode = pinMode;
        pinConfig.pull = pinPull;

        auto cfg = getHALInit( pinConfig );
        GPIO_Init( pinConfig.GPIOx, &cfg );
      }

      return error;
    }

    Chimera::Status_t GPIOClass::setState( const Chimera::GPIO::State state )
    {
      HAL_GPIO_WritePin( pinConfig.GPIOx, static_cast<uint16_t>( pinConfig.pinNum ), static_cast<GPIO_PinState>( state ) );
      return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t GPIOClass::getState( Chimera::GPIO::State &state )
    {
     state = static_cast<Chimera::GPIO::State>( HAL_GPIO_ReadPin( pinConfig.GPIOx, static_cast<uint16_t>( pinConfig.pinNum ) ) );
     return Chimera::CommonStatusCodes::OK;
    }

    Chimera::Status_t GPIOClass::toggle()
    {
      HAL_GPIO_TogglePin( pinConfig.GPIOx, static_cast<uint16_t>( pinConfig.pinNum ) );
      return Chimera::CommonStatusCodes::OK;
    }

    void GPIOClass::initAdvanced(const PinPort port, const PinNum pin, const PinSpeed speed, const uint32_t alt)
    {
      pinConfig.GPIOx     = port;
      pinConfig.pinNum    = pin;
      pinConfig.speed     = speed;
      pinConfig.alternate = alt;

      auto cfg = getHALInit(pinConfig);
      GPIO_Init(pinConfig.GPIOx, &cfg);
    }

    const PinNum convertPinNum( const uint8_t num )
    {
      PinNum pinNum = PinNum::NOT_A_PIN;

      switch ( num )
      {
        case 0:
          pinNum = Thor::GPIO::PinNum::PIN_0;
          break;
        case 1:
          pinNum = Thor::GPIO::PinNum::PIN_1;
          break;
        case 2:
          pinNum = Thor::GPIO::PinNum::PIN_2;
          break;
        case 3:
          pinNum = Thor::GPIO::PinNum::PIN_3;
          break;
        case 4:
          pinNum = Thor::GPIO::PinNum::PIN_4;
          break;
        case 5:
          pinNum = Thor::GPIO::PinNum::PIN_5;
          break;
        case 6:
          pinNum = Thor::GPIO::PinNum::PIN_6;
          break;
        case 7:
          pinNum = Thor::GPIO::PinNum::PIN_7;
          break;
        case 8:
          pinNum = Thor::GPIO::PinNum::PIN_8;
          break;
        case 9:
          pinNum = Thor::GPIO::PinNum::PIN_9;
          break;
        case 10:
          pinNum = Thor::GPIO::PinNum::PIN_10;
          break;
        case 11:
          pinNum = Thor::GPIO::PinNum::PIN_11;
          break;
        case 12:
          pinNum = Thor::GPIO::PinNum::PIN_12;
          break;
        case 13:
          pinNum = Thor::GPIO::PinNum::PIN_13;
          break;
        case 14:
          pinNum = Thor::GPIO::PinNum::PIN_14;
          break;
        case 15:
          pinNum = Thor::GPIO::PinNum::PIN_15;
          break;

        default:
          pinNum = Thor::GPIO::PinNum::NOT_A_PIN;
          break;
      };

      return pinNum;
    }

    const PinPort convertPort( const Chimera::GPIO::Port port )
    {
      PinPort pinPort = nullptr;

      switch ( port )
      {
#if defined( STM32F446xx ) || defined( STM32F767xx )
        case Chimera::GPIO::Port::PORTA:
          pinPort = GPIOA;
          break;

        case Chimera::GPIO::Port::PORTB:
          pinPort = GPIOB;
          break;

        case Chimera::GPIO::Port::PORTC:
          pinPort = GPIOC;
          break;

        case Chimera::GPIO::Port::PORTD:
          pinPort = GPIOD;
          break;

        case Chimera::GPIO::Port::PORTE:
          pinPort = GPIOE;
          break;

        case Chimera::GPIO::Port::PORTF:
          pinPort = GPIOF;
          break;

        case Chimera::GPIO::Port::PORTG:
          pinPort = GPIOG;
          break;

        case Chimera::GPIO::Port::PORTH:
          pinPort = GPIOH;
          break;
#endif

#if defined( STM32F767xx )
        case Chimera::GPIO::Port::PORTI:
          pinPort = GPIOI;
          break;

        case Chimera::GPIO::Port::PORTJ:
          pinPort = GPIOJ;
          break;

        case Chimera::GPIO::Port::PORTK:
          pinPort = GPIOK;
          break;
#endif

        default:
          pinPort = nullptr;
          break;
      };

      return pinPort;
    }

    const PinMode convertDrive( const Chimera::GPIO::Drive drive )
    {
      PinMode mode = PinMode::UNKNOWN_MODE;

      switch ( drive )
      {
        case Chimera::GPIO::Drive::INPUT:
          mode = PinMode::INPUT;
          break;

        case Chimera::GPIO::Drive::OUTPUT_PUSH_PULL:
          mode = PinMode::OUTPUT_PP;
          break;

        case Chimera::GPIO::Drive::OUTPUT_OPEN_DRAIN:
          mode = PinMode::OUTPUT_OD;
          break;

        case Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL:
          mode = PinMode::ALT_PP;
          break;

        case Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN:
          mode = PinMode::ALT_OD;
          break;

        case Chimera::GPIO::Drive::ANALOG:
          mode = PinMode::ANALOG;
          break;

        case Chimera::GPIO::Drive::HIZ:
          mode = PinMode::INPUT;
          break;

        default:
          mode = PinMode::UNKNOWN_MODE;
          break;
      };

      return mode;
    }

    const PinPull convertPull( const Chimera::GPIO::Pull pull )
    {
      PinPull pinPull = PinPull::UNKNOWN_PULL;

      switch ( pull )
      {
        case Chimera::GPIO::Pull::NO_PULL:
          pinPull = PinPull::NOPULL;
          break;

        case Chimera::GPIO::Pull::PULL_UP:
          pinPull = PinPull::PULLUP;
          break;

        case Chimera::GPIO::Pull::PULL_DN:
          pinPull = PinPull::PULLDN;
          break;

        default:
          pinPull = PinPull::UNKNOWN_PULL;
          break;
      };

      return pinPull;
    }

    const PinConfig convertPinInit( const Chimera::GPIO::PinInit &pin )
    {
      PinConfig cfg;

      cfg.GPIOx     = convertPort( pin.port );
      cfg.mode      = convertDrive( pin.drive );
      cfg.pinNum    = convertPinNum( pin.pin );
      cfg.pull      = convertPull( pin.pull );
      cfg.alternate = pin.alternate;

      return cfg;
    }

    const GPIO_TypeDef *const portMap( const Chimera::GPIO::Port port )
    {
      switch ( port )
      {
#if defined( TARGET_STM32F7 ) || defined( TARGET_STM32F4 )
        case Chimera::GPIO::Port::PORTA:
          return GPIOA;
          break;

        case Chimera::GPIO::Port::PORTB:
          return GPIOB;
          break;

        case Chimera::GPIO::Port::PORTC:
          return GPIOC;
          break;

        case Chimera::GPIO::Port::PORTD:
          return GPIOD;
          break;

        case Chimera::GPIO::Port::PORTE:
          return GPIOE;
          break;

        case Chimera::GPIO::Port::PORTF:
          return GPIOF;
          break;

        case Chimera::GPIO::Port::PORTG:
          return GPIOG;
          break;

        case Chimera::GPIO::Port::PORTH:
          return GPIOH;
          break;
#endif

#if defined( TARGET_STM32F7 )
        case Chimera::GPIO::Port::PORTI:
          return GPIOI;
          break;

        case Chimera::GPIO::Port::PORTJ:
          return GPIOJ;
          break;

        case Chimera::GPIO::Port::PORTK:
          return GPIOK;
          break;
#endif

        /* If we get here, something is wrong */
        default:
          return GPIOA;
          break;
      };
    }

    GPIO_InitTypeDef GPIOClass::getHALInit( const PinConfig &config )
    {
      GPIO_InitTypeDef InitStruct;

      InitStruct.Pin       = static_cast<uint32_t>( config.pinNum );
      InitStruct.Speed     = static_cast<uint32_t>( config.speed );
      InitStruct.Mode      = static_cast<uint32_t>( config.mode );
      InitStruct.Pull      = static_cast<uint32_t>( config.pull );
      InitStruct.Alternate = config.alternate;

      return InitStruct;
    }

    void GPIOClass::GPIO_Init( PinPort port, GPIO_InitTypeDef *initStruct )
    {
      GPIO_ClockEnable( port );
      HAL_GPIO_Init( port, initStruct );
    }

    void GPIOClass::GPIO_ClockEnable( PinPort port )
    {
#if defined( TARGET_STM32F7 ) || defined( TARGET_STM32F4 )
      SET_BIT( RCC->AHB1ENR, getRCCGPIOMask( port ) );
#endif
    }

    void GPIOClass::GPIO_ClockDisable( PinPort port )
    {
#if defined( TARGET_STM32F7 ) || defined( TARGET_STM32F4 )
      CLEAR_BIT( RCC->AHB1ENR, getRCCGPIOMask( port ) );
#endif
    }
  }    // namespace GPIO
}    // namespace Thor
