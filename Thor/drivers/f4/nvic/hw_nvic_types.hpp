/********************************************************************************
 *   File Name:
 *    hw_nvic_types.hpp
 *
 *   Description:
 *    Implements NVIC peripheral types
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_NVIC_TYPES_HPP
#define THOR_HW_DRIVER_NVIC_TYPES_HPP

/* Driver Includes */
#include <Thor/headers.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_NVIC == 1 )

namespace Thor::Driver::Interrupt
{
  using PriorityGrouping_t = uint32_t;

  static constexpr PriorityGrouping_t GROUP0 = 0x00000007u; /**< 0 bits for pre-emption priority 4 bits for subpriority */
  static constexpr PriorityGrouping_t GROUP1 = 0x00000006u; /**< 1 bits for pre-emption priority 3 bits for subpriority */
  static constexpr PriorityGrouping_t GROUP2 = 0x00000005u; /**< 2 bits for pre-emption priority 2 bits for subpriority */
  static constexpr PriorityGrouping_t GROUP3 = 0x00000004u; /**< 3 bits for pre-emption priority 1 bits for subpriority */
  static constexpr PriorityGrouping_t GROUP4 = 0x00000003u; /**< 4 bits for pre-emption priority 0 bits for subpriority */

}    // namespace Thor::Driver::Interrupt

#endif /* TARGET_STM32F4 && THOR_DRIVER_NVIC */
#endif /* !THOR_HW_DRIVER_NVIC_TYPES_HPP */