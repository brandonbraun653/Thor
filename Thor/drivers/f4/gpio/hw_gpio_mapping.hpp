/********************************************************************************
 *   File Name:
 *    hw_gpio_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_MAPPING_HPP
#define THOR_HW_GPIO_MAPPING_HPP

/* C++ Includes */
#include <unordered_map>

/* Chimera Includes */
#include <Chimera/types/gpio_types.hpp>

/* Driver Includes */
#include <Thor/drivers/f4/gpio/hw_gpio_types.hpp>

namespace Thor::Driver::GPIO
{

  /* clang-format off */

  /**
   *  Maps a Chimera GPIO Pull type into the appropriate register configuration value
   */
  static const std::unordered_map<uint8_t, size_t> PullMap{
    { static_cast<uint8_t>( Chimera::GPIO::Pull::NO_PULL ),       OPT_PUPDR::NOPULL   }, 
    { static_cast<uint8_t>( Chimera::GPIO::Pull::PULL_DN ),       OPT_PUPDR::PULLDOWN },
    { static_cast<uint8_t>( Chimera::GPIO::Pull::PULL_UP ),       OPT_PUPDR::PULLUP   },
    { static_cast<uint8_t>( Chimera::GPIO::Pull::UNKNOWN_PULL ),  OPT_PUPDR::NOPULL   }
  };

  /**
   *  Maps a Chimera GPIO Drive type into the appropriate register configuration value
   */
  static const std::unordered_map<uint8_t, size_t> ModeMap{ 
    { static_cast<uint8_t>( Chimera::GPIO::Drive::INPUT ),                OPT_MODER::INPUT   },
    { static_cast<uint8_t>( Chimera::GPIO::Drive::OUTPUT_PUSH_PULL ),     OPT_MODER::OUTPUT  },
    { static_cast<uint8_t>( Chimera::GPIO::Drive::OUTPUT_OPEN_DRAIN ),    OPT_MODER::OUTPUT  },
    { static_cast<uint8_t>( Chimera::GPIO::Drive::ALTERNATE_PUSH_PULL ),  OPT_MODER::AF      },
    { static_cast<uint8_t>( Chimera::GPIO::Drive::ALTERNATE_OPEN_DRAIN ), OPT_MODER::AF      },
    { static_cast<uint8_t>( Chimera::GPIO::Drive::ANALOG ),               OPT_MODER::ANALOG  },
    { static_cast<uint8_t>( Chimera::GPIO::Drive::HIZ ),                  OPT_MODER::INPUT   },
  };

  /**
   *  Maps a Chimera GPIO Speed type into the appropriate register configuration value
   */
  static const std::unordered_map<uint8_t, size_t> SpeedMap{
    { static_cast<uint8_t>( Speed::LOW ),       OPT_OSPEEDR::LOW        },
    { static_cast<uint8_t>( Speed::MEDIUM ),    OPT_OSPEEDR::MEDIUM     },
    { static_cast<uint8_t>( Speed::HIGH ),      OPT_OSPEEDR::HIGH       },
    { static_cast<uint8_t>( Speed::FAST ),      OPT_OSPEEDR::VERY_HIGH  },
    { static_cast<uint8_t>( Speed::MAX_SPEED ), OPT_OSPEEDR::VERY_HIGH  }
  };

  /**
   *  Maps a Chimera Port enum type into a value that can be used as an
   *  access index into register configuration settings caches.
   *  
   *  @note An example of this is Thor::Driver::RCC::ClockConfig_GPIO
   */
  static const std::unordered_map<uint8_t, uint8_t> PortToIteratorMap{
    { static_cast<uint8_t>( Chimera::GPIO::Port::PORTA ), 0u },
    { static_cast<uint8_t>( Chimera::GPIO::Port::PORTB ), 1u },
    { static_cast<uint8_t>( Chimera::GPIO::Port::PORTC ), 2u },
    { static_cast<uint8_t>( Chimera::GPIO::Port::PORTD ), 3u },
    { static_cast<uint8_t>( Chimera::GPIO::Port::PORTE ), 4u },
    { static_cast<uint8_t>( Chimera::GPIO::Port::PORTF ), 5u },
    { static_cast<uint8_t>( Chimera::GPIO::Port::PORTG ), 6u },
    { static_cast<uint8_t>( Chimera::GPIO::Port::PORTH ), 7u }
  };
  
  /**
   *  Maps a GPIO peripheral into the corresponding Chimera Port enum type
   */
  static const std::unordered_map<std::uintptr_t, uint8_t> InstanceToPortMap{
    { reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), static_cast<uint8_t>( Chimera::GPIO::Port::PORTA ) },
    { reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), static_cast<uint8_t>( Chimera::GPIO::Port::PORTB ) },
    { reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), static_cast<uint8_t>( Chimera::GPIO::Port::PORTC ) },
    { reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), static_cast<uint8_t>( Chimera::GPIO::Port::PORTD ) },
    { reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), static_cast<uint8_t>( Chimera::GPIO::Port::PORTE ) },
    { reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ), static_cast<uint8_t>( Chimera::GPIO::Port::PORTF ) },
    { reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ), static_cast<uint8_t>( Chimera::GPIO::Port::PORTG ) },
    { reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), static_cast<uint8_t>( Chimera::GPIO::Port::PORTH ) }
  };

  /* clang-format on */
}    // namespace Thor::Driver::GPIO

#endif /* !THOR_HW_GPIO_MAPPING_HPP */
