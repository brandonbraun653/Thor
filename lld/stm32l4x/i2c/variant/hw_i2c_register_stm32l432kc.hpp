/******************************************************************************
 *  File Name:
 *    hw_i2c_register_stm32l432kc.hpp
 *
 *  Description:
 *    I2C register definitions for the STM32L432KC series chips
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_I2C_REGISTER_STM32L432KC_HPP
#define THOR_LLD_I2C_REGISTER_STM32L432KC_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <Thor/lld/common/types.hpp>

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/
#define STM32_I2C1_PERIPH_AVAILABLE
#define STM32_I2C3_PERIPH_AVAILABLE


namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_I2C_PERIPHS       = 2;
  static constexpr size_t NUM_I2C_IRQ_HANDLERS  = 4;
  static constexpr RIndex_t I2C1_RESOURCE_INDEX = 0u;
  static constexpr RIndex_t I2C3_RESOURCE_INDEX = 1u;
}  // namespace Thor::LLD::I2C

#endif  /* !THOR_LLD_I2C_REGISTER_STM32L432KC_HPP */
