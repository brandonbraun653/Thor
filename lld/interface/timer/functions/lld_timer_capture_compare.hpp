/******************************************************************************
 *  File Name:
 *    lld_timer_capture_compare.hpp
 *
 *  Description:
 *    Capture compare aspects of a timer driver
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_CAPTURE_COMPARE_HPP
#define THOR_LLD_TIMER_CAPTURE_COMPARE_HPP

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
  enum CCMode : uint32_t
  {
    CCM_OUTPUT,
    CCM_INPUT_MAP_TI2,
    CCM_INPUT_MAP_TI1,
    CCM_INPUT_MAP_TRC,
    CCM_INVALID
  };

  enum CCPolarity : uint32_t
  {
    CCP_OUT_ACTIVE_HIGH,
    CCP_OUT_ACTIVE_LOW,
    CCP_IN_RISING_EDGE,
    CCP_IN_FALLING_EDGE,
    CCP_IN_BOTH_EDGE,
    CCP_INVALID
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Disables a capture compare channel
   *
   * @param timer   Which timer to act on
   * @param ch      Which channel to disable
   * @return Chimera::Status_t
   */
  Chimera::Status_t disableCCChannel( Handle_rPtr timer, const Chimera::Timer::Channel ch );

  /**
   * @brief Enables a capture compare channel
   *
   * @param timer   Which timer to act on
   * @param ch      Which channel to enable
   * @return Chimera::Status_t
   */
  Chimera::Status_t enableCCChannel( Handle_rPtr timer, const Chimera::Timer::Channel ch );

  /**
   * @brief Sets the capture/compare polarity of a channel
   *
   * @param timer   Which timer to act on
   * @param ch      The channel to act on
   * @param pol     What polarity is being set
   * @return Chimera::Status_t
   */
  Chimera::Status_t setCCPolarity( Handle_rPtr timer, const Chimera::Timer::Channel ch, const CCPolarity pol );

  /**
   * @brief Sets the capture/compare mode behavior for a timer channel
   *
   * @param timer   Which timer to act on
   * @param ch      The channel to act on
   * @param mode    Mode being set
   * @return Chimera::Status_t
   */
  Chimera::Status_t setCCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch, const CCMode mode );

  /**
   * @brief Gets the capture/compare mode set on the channel
   *
   * @param timer   Which timer to act on
   * @param ch      Which channel to inspect
   * @return CCMode
   */
  CCMode getCCMode( Handle_rPtr timer, const Chimera::Timer::Channel ch );

}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_CAPTURE_COMPARE_HPP */
