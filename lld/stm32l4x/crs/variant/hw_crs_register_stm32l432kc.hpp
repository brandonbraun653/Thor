/******************************************************************************
 *  File Name:
 *    hw_crs_register_stm32l432kc.hpp
 *
 *  Description:
 *    CRS definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_CRS_REGISTER_STM32L432KC_HPP
#define THOR_LLD_CRS_REGISTER_STM32L432KC_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_CRS1_PERIPH_AVAILABLE


namespace Thor::LLD::CRS
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_CRS_PERIPHS       = 1;
  static constexpr size_t NUM_CRS_IRQ_HANDLERS  = 4;
  static constexpr RIndex_t CRS1_RESOURCE_INDEX = 0u;

}    // namespace Thor::LLD::CRS

#endif /* !THOR_LLD_CRS_REGISTER_STM32L432KC_HPP */
