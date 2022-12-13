/******************************************************************************
 *  File Name:
 *    hw_iwdg_register_stm32f446re.hpp
 *
 *  Description:
 *    IWDG register definitions for the STM32F446RE series chips.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_IWDG_REGISTER_STM32F446RE_HPP
#define THOR_LLD_IWDG_REGISTER_STM32F446RE_HPP

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
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_IWDG_PERIPHS       = 1;
  static constexpr RIndex_t IWDG1_RESOURCE_INDEX = 0u;

}    // namespace Thor::LLD::IWDG

#endif /* !THOR_LLD_IWDG_REGISTER_STM32F446RE_HPP */
