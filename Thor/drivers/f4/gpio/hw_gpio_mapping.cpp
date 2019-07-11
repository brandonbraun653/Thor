/********************************************************************************
 *   File Name:
 *    hw_gpio_mapping.cpp
 *
 *   Description:
 *    Useful maps for the GPIO peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/gpio/hw_gpio_mapping.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

namespace Thor::Driver::GPIO
{
  /* clang-format off */

  const std::unordered_map<Chimera::GPIO::Pull, size_t> PullMap
  {
    { Chimera::GPIO::Pull::NO_PULL,       OPT_PUPDR::NOPULL   }, 
    { Chimera::GPIO::Pull::PULL_DN,       OPT_PUPDR::PULLDOWN },
    { Chimera::GPIO::Pull::PULL_UP,       OPT_PUPDR::PULLUP   },
    { Chimera::GPIO::Pull::UNKNOWN_PULL,  OPT_PUPDR::NOPULL   }
  };

  const std::unordered_map<Chimera::GPIO::Drive, size_t> ModeMap
  { 
    { Chimera::GPIO::Drive::INPUT,                OPT_MODER::INPUT   },
    { Chimera::GPIO::Drive::OUTPUT_PUSH_PULL,     OPT_MODER::OUTPUT  },
    { Chimera::GPIO::Drive::OUTPUT_OPEN_DRAIN,    OPT_MODER::OUTPUT  },
    { Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL,  OPT_MODER::AF      },
    { Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN, OPT_MODER::AF      },
    { Chimera::GPIO::Drive::ANALOG,               OPT_MODER::ANALOG  },
    { Chimera::GPIO::Drive::HIZ,                  OPT_MODER::INPUT   },
  };

  const std::unordered_map<Thor::Driver::GPIO::Speed, size_t> SpeedMap
  {
    { Speed::LOW,       OPT_OSPEEDR::LOW        },
    { Speed::MEDIUM,    OPT_OSPEEDR::MEDIUM     },
    { Speed::HIGH,      OPT_OSPEEDR::HIGH       },
    { Speed::FAST,      OPT_OSPEEDR::VERY_HIGH  },
    { Speed::MAX_SPEED, OPT_OSPEEDR::VERY_HIGH  }
  };

  const std::unordered_map<Chimera::GPIO::Port, uint8_t> PortToIteratorMap
  {
    { Chimera::GPIO::Port::PORTA, 0u },
    { Chimera::GPIO::Port::PORTB, 1u },
    { Chimera::GPIO::Port::PORTC, 2u },
    { Chimera::GPIO::Port::PORTD, 3u },
    { Chimera::GPIO::Port::PORTE, 4u },
    { Chimera::GPIO::Port::PORTF, 5u },
    { Chimera::GPIO::Port::PORTG, 6u },
    { Chimera::GPIO::Port::PORTH, 7u }
  };
  
  const std::unordered_map<std::uintptr_t, Chimera::GPIO::Port> InstanceToPortMap
  {
    { reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), Chimera::GPIO::Port::PORTA },
    { reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), Chimera::GPIO::Port::PORTB },
    { reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), Chimera::GPIO::Port::PORTC },
    { reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), Chimera::GPIO::Port::PORTD },
    { reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), Chimera::GPIO::Port::PORTE },
    { reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ), Chimera::GPIO::Port::PORTF },
    { reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ), Chimera::GPIO::Port::PORTG },
    { reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), Chimera::GPIO::Port::PORTH }
  };

  const std::unordered_map<Chimera::GPIO::Port, decltype( GPIOA_PERIPH )> PortToInstanceMap
  {
    { Chimera::GPIO::Port::PORTA, GPIOA_PERIPH },
    { Chimera::GPIO::Port::PORTB, GPIOB_PERIPH },
    { Chimera::GPIO::Port::PORTC, GPIOC_PERIPH },
    { Chimera::GPIO::Port::PORTD, GPIOD_PERIPH },
    { Chimera::GPIO::Port::PORTE, GPIOE_PERIPH },
    { Chimera::GPIO::Port::PORTF, GPIOF_PERIPH },
    { Chimera::GPIO::Port::PORTG, GPIOG_PERIPH },
    { Chimera::GPIO::Port::PORTH, GPIOH_PERIPH }
  };

  /* clang-format on */

}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
