/********************************************************************************
 *  File Name:
 *    hw_sys_register_stm32l432kc.hpp
 *
 *  Description:
 *    SYS definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_SYS_REGISTER_STM32L432KC_HPP
#define THOR_HW_SYS_REGISTER_STM32L432KC_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>


namespace Thor::LLD::SYS
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_SYSCFG_PERIPHS       = 1;
  static constexpr RIndex_t SYSCFG1_RESOURCE_INDEX = 0;
  static constexpr size_t NUM_PERIPHERAL_TYPES     = 21;  /**< Unique peripheral types on device */

}    // namespace Thor::LLD::SYS

#endif /* !THOR_HW_SYS_REGISTER_STM32L432KC_HPP */
