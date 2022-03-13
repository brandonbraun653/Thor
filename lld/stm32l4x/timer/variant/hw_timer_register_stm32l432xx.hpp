/******************************************************************************
 *  File Name:
 *    hw_timer_register_stm32l432xx.hpp
 *
 *  Description:
 *    STM32L432xx specific definitions
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_STM32L432XX_TIMER_HPP
#define THOR_LLD_STM32L432XX_TIMER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace Thor::LLD::TIMER
{
  static constexpr size_t NUM_TIMER_PERIPHS     = 6;
  static constexpr size_t NUM_ADVANCED_PERIPHS  = 1;
  static constexpr size_t NUM_BASIC_PERIPHS     = 2;
  static constexpr size_t NUM_GENERAL_PERIPHS   = 2;
}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_STM32L432XX_TIMER_HPP */
