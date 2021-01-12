/********************************************************************************
 *  File Name:
 *    hw_exti_register_stm32f446re.hpp
 *
 *  Description:
 *    EXTI definitions for the STM32F446RE series chips.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_EXTI_REGISTER_STM32F432KC_HPP
#define THOR_HW_EXTI_REGISTER_STM32F432KC_HPP

/* Thor Includes  */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_EXTI1_PERIPH_AVAILABLE


namespace Thor::LLD::EXTI
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_EXTI_PERIPHS       = 1;
  static constexpr RIndex_t EXTI1_RESOURCE_INDEX = 0;

  static constexpr size_t NUM_EXTI_LINES = 23;

}  // namespace Thor::LLD::EXTI

#endif  /* !THOR_HW_EXTI_REGISTER_STM32F432KC_HPP */
