/******************************************************************************
 *  File Name:
 *    lld_timer_helpers.hpp
 *
 *  Description:
 *    Helpers for interacting with timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_HELPERS_HPP
#define THOR_LLD_TIMER_HELPERS_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/timer>
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Methods
  ---------------------------------------------------------------------------*/
  /**
   * @brief Allocates a new timer with the system and prepares it for operation
   *
   * @param instance    Which timer to allocate
   * @return Chimera::Status_t
   */
  Chimera::Status_t allocate( const Chimera::Timer::Instance &instance );

  namespace Master
  {
    /**
     * @brief Initializes a timer using the settings from a CoreConfig object
     * @warning Erases previously configured register settings for this timer
     *
     * This assumes that the timer is being driven from an internal clock and
     * will run as a master.
     *
     * @param timer   The timer to act on
     * @param cfg     The configuration to use
     * @return Chimera::Status_t
     */
    Chimera::Status_t initCore( Handle_rPtr timer, const Chimera::Timer::CoreConfig &cfg );

    // Chimera::Status_t initCaptureCompare( Handle_rPtr timer, )
  }

}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_HELPERS_HPP */
