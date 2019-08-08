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

  const std::array<uint32_t, static_cast<size_t>(Chimera::GPIO::Pull::NUM_OPTIONS)> PullMap =
  {
    OPT_PUPDR::NOPULL, 
    OPT_PUPDR::PULLDOWN,
    OPT_PUPDR::PULLUP,
    OPT_PUPDR::NOPULL  
  };

  const std::array<uint32_t, static_cast<size_t>(Chimera::GPIO::Drive::NUM_OPTIONS)> ModeMap =
  { 
    OPT_MODER::INPUT,
    OPT_MODER::OUTPUT,
    OPT_MODER::OUTPUT,
    OPT_MODER::AF,
    OPT_MODER::AF,
    OPT_MODER::ANALOG,
    OPT_MODER::INPUT
  };

  const std::array<uint32_t, static_cast<size_t>(Thor::Driver::GPIO::Speed::NUM_OPTIONS)> SpeedMap =
  {
    OPT_OSPEEDR::LOW,
    OPT_OSPEEDR::MEDIUM,
    OPT_OSPEEDR::HIGH,
    OPT_OSPEEDR::VERY_HIGH
  };

  const std::array<uint8_t, static_cast<size_t>(Chimera::GPIO::Port::NUM_OPTIONS)> PortToIteratorMap =
  {
    0u,
    1u,
    2u,
    3u,
    4u,
    5u,
    6u,
    7u
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

  const std::unordered_map<std::uintptr_t, size_t> InstanceToResourceIndex
  {
    { reinterpret_cast<std::uintptr_t>( GPIOA_PERIPH ), 0 },
    { reinterpret_cast<std::uintptr_t>( GPIOB_PERIPH ), 1 },
    { reinterpret_cast<std::uintptr_t>( GPIOC_PERIPH ), 2 },
    { reinterpret_cast<std::uintptr_t>( GPIOD_PERIPH ), 3 },
    { reinterpret_cast<std::uintptr_t>( GPIOE_PERIPH ), 4 },
    { reinterpret_cast<std::uintptr_t>( GPIOF_PERIPH ), 5 },
    { reinterpret_cast<std::uintptr_t>( GPIOG_PERIPH ), 6 },
    { reinterpret_cast<std::uintptr_t>( GPIOH_PERIPH ), 7 }
  };

  /* clang-format on */

}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
