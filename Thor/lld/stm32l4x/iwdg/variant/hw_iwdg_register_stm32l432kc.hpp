/********************************************************************************
 *  File Name:
 *    hw_iwdg_register_stm32l432kc.hpp
 *
 *  Description:
 *    IWDG register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_IWDG_REGISTER_STM32L432KC_HPP
#define THOR_LLD_IWDG_REGISTER_STM32L432KC_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_IWDG1_PERIPH_AVAILABLE


namespace Thor::LLD::IWDG
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_IWDG_PERIPHS = 1;

  static constexpr RIndex_t IWDG1_RESOURCE_INDEX = 0u;

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_LLD_IWDG_REGISTER_STM32L432KC_HPP */
