/********************************************************************************
 *  File Name:
 *    hw_exti_register_stm32l432kc.hpp
 *
 *  Description:
 *    EXTI definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_EXTI_REGISTER_STM32L432KC_HPP
#define THOR_HW_EXTI_REGISTER_STM32L432KC_HPP

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


  static constexpr size_t NUM_EXTI_LINES = 38;

}  // namespace Thor::LLD::EXTI

#endif  /* !THOR_HW_EXTI_REGISTER_STM32L432KC_HPP */
