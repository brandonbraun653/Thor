/******************************************************************************
 *  File Name:
 *    hw_timer_register_stm32f446xx.hpp
 *
 *  Description:
 *    STM32F446xx specific definitions
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_STM32F446XX_TIMER_HPP
#define THOR_LLD_STM32F446XX_TIMER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  All Timers
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_TIMER_PERIPHS = 14;

  static constexpr size_t TIMER1_GLOBAL_RESOURCE_INDEX  = 0;
  static constexpr size_t TIMER2_GLOBAL_RESOURCE_INDEX  = 1;
  static constexpr size_t TIMER3_GLOBAL_RESOURCE_INDEX  = 2;
  static constexpr size_t TIMER4_GLOBAL_RESOURCE_INDEX  = 3;
  static constexpr size_t TIMER5_GLOBAL_RESOURCE_INDEX  = 4;
  static constexpr size_t TIMER6_GLOBAL_RESOURCE_INDEX  = 5;
  static constexpr size_t TIMER7_GLOBAL_RESOURCE_INDEX  = 6;
  static constexpr size_t TIMER8_GLOBAL_RESOURCE_INDEX  = 7;
  static constexpr size_t TIMER9_GLOBAL_RESOURCE_INDEX  = 8;
  static constexpr size_t TIMER10_GLOBAL_RESOURCE_INDEX = 9;
  static constexpr size_t TIMER11_GLOBAL_RESOURCE_INDEX = 10;
  static constexpr size_t TIMER12_GLOBAL_RESOURCE_INDEX = 11;
  static constexpr size_t TIMER13_GLOBAL_RESOURCE_INDEX = 12;
  static constexpr size_t TIMER14_GLOBAL_RESOURCE_INDEX = 13;

  /*---------------------------------------------------------------------------
  Advanced Timers
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_ADVANCED_PERIPHS       = 2;
  static constexpr size_t TIMER1_TYPE_RESOURCE_INDEX = 0;
  static constexpr size_t TIMER8_TYPE_RESOURCE_INDEX = 1;

  /*---------------------------------------------------------------------------
  General Timers
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_GENERAL_PERIPHS         = 10;
  static constexpr size_t TIMER2_TYPE_RESOURCE_INDEX  = 0;
  static constexpr size_t TIMER3_TYPE_RESOURCE_INDEX  = 1;
  static constexpr size_t TIMER4_TYPE_RESOURCE_INDEX  = 2;
  static constexpr size_t TIMER5_TYPE_RESOURCE_INDEX  = 3;
  static constexpr size_t TIMER9_TYPE_RESOURCE_INDEX  = 4;
  static constexpr size_t TIMER10_TYPE_RESOURCE_INDEX = 5;
  static constexpr size_t TIMER11_TYPE_RESOURCE_INDEX = 6;
  static constexpr size_t TIMER12_TYPE_RESOURCE_INDEX = 7;
  static constexpr size_t TIMER13_TYPE_RESOURCE_INDEX = 8;
  static constexpr size_t TIMER14_TYPE_RESOURCE_INDEX = 9;

  /*---------------------------------------------------------------------------
  Basic Timers
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_BASIC_PERIPHS          = 2;
  static constexpr size_t TIMER6_TYPE_RESOURCE_INDEX = 0;
  static constexpr size_t TIMER7_TYPE_RESOURCE_INDEX = 1;

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_STM32F446XX_TIMER_HPP */
