/********************************************************************************
 *  File Name:
 *    hw_wwdg_types.hpp
 *
 *  Description:
 *    LLD types for the WWDG Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_WWDG_TYPES_HPP
#define THOR_HW_WWDG_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32l4x/wwdg/hw_wwdg_prj.hpp>

namespace Thor::LLD::WWDG
{
  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t CR;  /**< WWDG Control register,       Address offset: 0x00 */
    volatile uint32_t CFR; /**< WWDG Configuration register, Address offset: 0x04 */
    volatile uint32_t SR;  /**< WWDG Status register,        Address offset: 0x08 */
  };

}    // namespace Thor::LLD::WWDG

#endif /* !THOR_HW_WWDG_TYPES_HPP*/
