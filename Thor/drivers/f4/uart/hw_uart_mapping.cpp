/********************************************************************************
 *   File Name:
 *    hw_uart_mapping.cpp
 *
 *   Description:
 *    Useful maps for the UART peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/drivers/f4/uart/hw_uart_mapping.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )

namespace Thor::Driver::UART
{
  /* clang-format off */

  const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex
  {
    { reinterpret_cast<std::uintptr_t>( UART4_PERIPH ), 0 },
    { reinterpret_cast<std::uintptr_t>( UART5_PERIPH ), 1 }
  };

  /* clang-format on */
}    // namespace Thor::Driver::UART

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
