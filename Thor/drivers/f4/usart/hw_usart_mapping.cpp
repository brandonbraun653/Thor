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
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/usart/hw_usart_mapping.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 )

namespace Thor::Driver::USART
{
  const Thor::Driver::RCC::ResourceMap_t InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( USART1_PERIPH ), 0 },
    { reinterpret_cast<std::uintptr_t>( USART2_PERIPH ), 1 },
    { reinterpret_cast<std::uintptr_t>( USART3_PERIPH ), 2 },
    { reinterpret_cast<std::uintptr_t>( USART6_PERIPH ), 3 }
  };

  const IRQn_Type USART_IRQn[ NUM_USART_PERIPHS ] = {
    /* USART 1 */
    USART1_IRQn,
    /* USART 2 */
    USART2_IRQn,
    /* USART 3 */
    USART3_IRQn,
    /* USART 6 */
    USART6_IRQn
  };
}    // namespace Thor::Driver::USART

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
