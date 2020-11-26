/********************************************************************************
 *  File Name:
 *    hw_usb_register_stm32l432kc.hpp
 *
 *  Description:
 *    USB definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_USB_REGISTER_STM32L432KC_HPP
#define THOR_LLD_USB_REGISTER_STM32L432KC_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_USB1_PERIPH_AVAILABLE


namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_USB_PERIPHS       = 1;
  static constexpr RIndex_t USB1_RESOURCE_INDEX = 0u;
  static constexpr size_t NUM_USB_ENDPOINTS     = 8;
  static constexpr size_t PACKET_MEMORY_AREA_SZ = 1024;
}    // namespace Thor::LLD::USB

#endif /* !THOR_LLD_USB_REGISTER_STM32L432KC_HPP */
