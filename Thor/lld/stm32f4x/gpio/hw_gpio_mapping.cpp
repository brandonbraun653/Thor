/********************************************************************************
 *  File Name:
 *    hw_gpio_mapping.cpp
 *
 *  Description:
 *    Useful maps for the GPIO peripherals
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/gpio/hw_gpio_mapping.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_GPIO )

namespace Thor::LLD::GPIO
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;


  void initializeMapping()
  {
  }


  /* clang-format off */

  const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Pull::NUM_OPTIONS )> PullMap =
  {
    OPT_PUPDR::NOPULL, 
    OPT_PUPDR::PULLDOWN,
    OPT_PUPDR::PULLUP,
    OPT_PUPDR::NOPULL  
  };

  const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Drive::NUM_OPTIONS )> ModeMap =
  { 
    OPT_MODER::INPUT,
    OPT_MODER::OUTPUT,
    OPT_MODER::OUTPUT,
    OPT_MODER::AF,
    OPT_MODER::AF,
    OPT_MODER::ANALOG,
    OPT_MODER::INPUT
  };

  const std::array<uint32_t, static_cast<size_t>( Thor::LLD::GPIO::Speed::NUM_OPTIONS )> SpeedMap =
  {
    OPT_OSPEEDR::LOW,
    OPT_OSPEEDR::MEDIUM,
    OPT_OSPEEDR::HIGH,
    OPT_OSPEEDR::VERY_HIGH
  };

  const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Port::NUM_OPTIONS )> PortToIteratorMap =
  {
    GPIOA_RESOURCE_INDEX,
    GPIOB_RESOURCE_INDEX,
    GPIOC_RESOURCE_INDEX,
    GPIOD_RESOURCE_INDEX,
    GPIOE_RESOURCE_INDEX,
    GPIOF_RESOURCE_INDEX,
    GPIOG_RESOURCE_INDEX,
    GPIOH_RESOURCE_INDEX
  };

  /* clang-format on */

}    // namespace Thor::LLD::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
