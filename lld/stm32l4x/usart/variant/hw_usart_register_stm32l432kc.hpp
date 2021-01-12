/********************************************************************************
 *  File Name:
 *    hw_usart_register_stm32l432kc.hpp
 *
 *  Description:
 *    USART definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_USART_VARIANT_STM32L432KC_HPP
#define THOR_LLD_USART_VARIANT_STM32L432KC_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_USART1_PERIPH_AVAILABLE
#define STM32_USART2_PERIPH_AVAILABLE

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_USART_PERIPHS = 2;

  static constexpr RIndex_t USART1_RESOURCE_INDEX = 0u;
  static constexpr RIndex_t USART2_RESOURCE_INDEX = 1u;
  static constexpr RIndex_t USART3_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;

}  // namespace Thor::LLD::USART

#endif  /* !THOR_LLD_USART_VARIANT_STM32L432KC_HPP */
