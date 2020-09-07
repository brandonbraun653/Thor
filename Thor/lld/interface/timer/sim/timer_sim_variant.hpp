/********************************************************************************
 *  File Name:
 *    timer_sim_variant.hpp
 *
 *  Description:
 *    Simulator version of the peripheral, used for LLD testing
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_SIM_VARIANT_HPP
#define THOR_LLD_TIMER_SIM_VARIANT_HPP

/* include description */
#include <cstddef>

namespace Thor::LLD::TIMER
{
  static constexpr size_t NUM_ADVANCED_PERIPHS  = 1;
  static constexpr size_t NUM_BASIC_PERIPHS     = 1;
  static constexpr size_t NUM_LOW_POWER_PERIPHS = 1;

  static constexpr size_t NUM_GENERAL_PERIPHS = 1;
}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_SIM_VARIANT_HPP */
