/******************************************************************************
 *  File Name:
 *    lld_timer_output_compare.hpp
 *
 *  Description:
 *    Output compare functionality driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_OUTPUT_COMPARE_HPP
#define THOR_LLD_TIMER_OUTPUT_COMPARE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/timer>
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <cstdint>


namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum OCMode : uint32_t
  {
    OC_MODE_FROZEN              = 0b0000,
    OC_MODE_ACTIVE_LVL_MATCH    = 0b0001,
    OC_MODE_INACTIVE_LVL_MATCH  = 0b0010,
    OC_MODE_TOGGLE_MATCH        = 0b0011,
    OC_MODE_FORCE_INACTIVE      = 0b0100,
    OC_MODE_FORCE_ACTIVE        = 0b0101,
    OC_MODE_PWM_MODE_1          = 0b0110,
    OC_MODE_PWM_MODE_2          = 0b0111,
    OC_MODE_RETRIG_OPM_MODE_1   = 0b1000,
    OC_MODE_RETRIG_OPM_MODE_2   = 0b1001,
    OC_MODE_COMBINED_PWM_MODE_1 = 0b1100,
    OC_MODE_COMBINED_PWM_MODE_2 = 0b1101,
    OC_MODE_ASYM_PWM_MODE_1     = 0b1110,
    OC_MODE_ASYM_PWM_MODE_2     = 0b1111,
    OC_MODE_INVALID             = 0b1111'1111
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Enable/disable preload of capture compare register
   *
   * @param timer   Which timer to act on
   * @param ch      Which channel to act on
   * @param use     Should use or not?
   */
  void useOCPreload( Handle_rPtr timer, const Chimera::Timer::Channel ch, const bool use );

  /**
   * @brief Set the output compare mode
   *
   * @param timer   Which timer to act on
   * @param ch      Which channel to act on
   * @param mode    Desired mode to set
   * @return Chimera::Status_t
   */
  Chimera::Status_t setOCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch, const OCMode mode );

  /**
   * @brief Get the currently set output compare mode
   *
   * @param timer   Which timer to read
   * @param ch      Which channel to read
   * @return OCMode
   */
  OCMode getOCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch );

  /**
   * @brief Set the output comparison reference
   *
   * @param timer   Which timer to act on
   * @param ch      Which channel to act on
   * @param ref     The reference to set
   * @return Chimera::Status_t
   */
  Chimera::Status_t setOCReference( Handle_rPtr timer, const Chimera::Timer::Channel ch, const uint32_t ref );

  /**
   * @brief Get the current output compare reference
   *
   * @param timer   The timer to read
   * @param ch      Which channel to read
   * @return uint32_t
   */
  uint32_t getOCReference( Handle_rPtr timer, const Chimera::Timer::Channel ch );

}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_OUTPUT_COMPARE_HPP */
