/********************************************************************************
 *  File Name:
 *    hw_usb_register_stm32f446re.hpp
 *
 *  Description:
 *    USB definitions for the STM32F446RE series chips
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_USB_REGISTER_STM32F446RE_HPP
#define THOR_LLD_USB_REGISTER_STM32F446RE_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>


namespace Thor::LLD::USB
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_USB_PERIPHS           = 2;
  static constexpr size_t USB_OTG_FS_RESOURCE_INDEX = 0u;
  static constexpr size_t USB_OTG_HS_RESOURCE_INDEX = 1u;

}    // namespace Thor::LLD::USB

#endif /* !THOR_LLD_USB_REGISTER_STM32F446RE_HPP */
