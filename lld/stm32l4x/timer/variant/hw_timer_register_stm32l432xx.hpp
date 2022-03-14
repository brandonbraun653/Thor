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
  /*---------------------------------------------------------------------------
  All Timers
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_TIMER_PERIPHS = 6;

  static constexpr size_t TIMER1_GLOBAL_RESOURCE_INDEX  = 0;
  static constexpr size_t TIMER2_GLOBAL_RESOURCE_INDEX  = 1;
  static constexpr size_t TIMER6_GLOBAL_RESOURCE_INDEX  = 2;
  static constexpr size_t TIMER7_GLOBAL_RESOURCE_INDEX  = 3;
  static constexpr size_t TIMER15_GLOBAL_RESOURCE_INDEX = 4;
  static constexpr size_t TIMER16_GLOBAL_RESOURCE_INDEX = 5;

  /*---------------------------------------------------------------------------
  Advanced Timers
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_ADVANCED_PERIPHS       = 1;
  static constexpr size_t TIMER1_TYPE_RESOURCE_INDEX = 0;

  /*---------------------------------------------------------------------------
  General Timers
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_GENERAL_PERIPHS         = 3;
  static constexpr size_t TIMER2_TYPE_RESOURCE_INDEX  = 0;
  static constexpr size_t TIMER15_TYPE_RESOURCE_INDEX = 1;
  static constexpr size_t TIMER16_TYPE_RESOURCE_INDEX = 2;

  /*---------------------------------------------------------------------------
  Basic Timers
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_BASIC_PERIPHS          = 2;
  static constexpr size_t TIMER6_TYPE_RESOURCE_INDEX = 0;
  static constexpr size_t TIMER7_TYPE_RESOURCE_INDEX = 1;

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_STM32L432XX_TIMER_HPP */
