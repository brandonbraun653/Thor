/******************************************************************************
 *  File Name:
 *    lld_timer_time_base.hpp
 *
 *  Description:
 *    Common driver for Advanced, Basic, General timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_TIME_BASE_DRIVER_HPP
#define THOR_LLD_TIMER_TIME_BASE_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <cstdint>
#include <limits>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr float RATE_UNKNOWN = std::numeric_limits<float>::max();

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class ClockSource
  {
    INTERNAL,
    EXTERNAL_MODE_1,
    EXTERNAL_MODE_2,
    INTERNAL_TRIGGER
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct TickConfig
  {
    float       rate_ns;   /**< Base "tick" period in nanoseconds */
    float       tolerance; /**< Acceptable error rate percent */
    ClockSource clock_src; /**< Clock driving the timer */

    TickConfig() : rate_ns( 0.0f ), tolerance( 0.0f ), clock_src( ClockSource::INTERNAL )
    {
    }
  };

  /*---------------------------------------------------------------------------
  Public Methods
  ---------------------------------------------------------------------------*/
  /**
   * @brief Sets the tick resolution for the entire Timer
   * @note The timer is disabled when this function exits
   *
   * @param timer     The timer being acted on
   * @param cfg       Configuration to apply
   * @return Chimera::Status_t
   */
  Chimera::Status_t setBaseTickPeriod( Handle_rPtr timer, const TickConfig &cfg );

  /**
   * @brief Reads register settings to determine the current tick period
   *
   * @param timer     The timer being acted on
   * @return float    Time base period in nanoseconds
   */
  float getBaseTickPeriod( const Handle_rPtr timer );

  /**
   * @brief Sets the event generation rate of a timer
   * @note Only valid if the timer is clocked from an internal source
   *
   * @param timer     The timer being acted on
   * @param rate_ns   What rate an event should be generated at (nanoseconds)
   * @return Chimera::Status_t
   */
  Chimera::Status_t setEventRate( Handle_rPtr timer, const float rate_ns );

  /**
   * @brief Get rate at which events are being generated by a timer
   * @note Only valid if the timer is clocked from an internal source
   *
   * @param timer     The timer being acted on
   * @return float    The event rate period in nanoseconds
   */
  float getEventRate( Handle_rPtr timer );

  /**
   * @brief Get's the value assigned to the TIMx_ARR register
   *
   * @param timer     Which timer to read from
   * @return uint32_t
   */
  uint32_t getAutoReload( Handle_rPtr timer );

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_TIME_BASE_DRIVER_HPP */
