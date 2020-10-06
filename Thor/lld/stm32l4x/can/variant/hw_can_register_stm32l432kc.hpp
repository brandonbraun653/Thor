/********************************************************************************
 *  File Name:
 *    hw_can_register_stm32l432kc.hpp
 *
 *  Description:
 *    CAN definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_CAN_REGISTER_STM32L432KC_HPP
#define THOR_LLD_CAN_REGISTER_STM32L432KC_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_CAN1_PERIPH_AVAILABLE

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_CAN_PERIPHS = 1;

  static constexpr RIndex_t CAN1_RESOURCE_INDEX = 0u;

}  // namespace

#endif  /* !THOR_LLD_CAN_REGISTER_STM32L432KC_HPP */
