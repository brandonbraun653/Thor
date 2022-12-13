/******************************************************************************
 *  File Name:
 *    hw_wwdg_register_stm32f446re.hpp
 *
 *  Description:
 *    WWDG register definitions for the STM32F446RE series chips.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_WWDG_REGISTER_STM32F446RE_HPP
#define THOR_LLD_WWDG_REGISTER_STM32F446RE_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_WWDG1_PERIPH_AVAILABLE


namespace Thor::LLD::WWDG
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_WWDG_PERIPHS       = 1;
  static constexpr RIndex_t WWDG1_RESOURCE_INDEX = 0u;

}    // namespace Thor::LLD::WWDG

#endif /* !THOR_LLD_WWDG_REGISTER_STM32F446RE_HPP */
