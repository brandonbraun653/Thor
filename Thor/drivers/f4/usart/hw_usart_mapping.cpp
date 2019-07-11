/********************************************************************************
 *   File Name:
 *    hw_usart_mapping.cpp
 *
 *   Description:
 *    Useful maps for the USART peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/usart/hw_usart_mapping.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  /* clang-format off */

  const std::unordered_map<std::uintptr_t, size_t> InstanceToResourceIndex
  {
    { reinterpret_cast<std::uintptr_t>( USART1_PERIPH ), 0 },
    { reinterpret_cast<std::uintptr_t>( USART2_PERIPH ), 1 },
    { reinterpret_cast<std::uintptr_t>( USART3_PERIPH ), 2 },
    { reinterpret_cast<std::uintptr_t>( USART6_PERIPH ), 3 }
  };

  /* clang-format on */
}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
