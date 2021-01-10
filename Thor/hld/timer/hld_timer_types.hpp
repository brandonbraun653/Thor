/********************************************************************************
 *  File Name:
 *    hld_timer_types.hpp
 *
 *  Description:
 *    Types for Thor Timer HLD
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cstddef>
#include <memory>

#pragma once
#ifndef THOR_HLD_TIMER_TYPES_HPP
#define THOR_HLD_TIMER_TYPES_HPP

namespace Thor::TIMER
{
  class AdvancedDriver;
  using AdvancedDriver_rPtr = std::shared_ptr<AdvancedDriver>;
  using AdvancedDriver_uPtr = std::shared_ptr<AdvancedDriver>;

  class BasicDriver;
  using BasicDriver_rPtr = std::shared_ptr<BasicDriver>;
  using BasicDriver_uPtr = std::shared_ptr<BasicDriver>;


  class GeneralDriver;
  using GeneralDriver_rPtr = std::shared_ptr<GeneralDriver>;
  using GeneralDriver_uPtr = std::shared_ptr<GeneralDriver>;

  class LowPowerDriver;
  using LowPowerDriver_rPtr = std::shared_ptr<LowPowerDriver>;
  using LowPowerDriver_uPtr = std::shared_ptr<LowPowerDriver>;

  /**
   *  Describes a timer peripheral and what it can do
   */
  struct PeripheralState
  {
    bool validity; /**< Are the current settings/data valid? */

    bool inUse;           /**< Whether or not the peripheral is used by another system */
    size_t resourceIndex; /**< Lookup index for various hardware resources */
    void *timerDriver;    /**< Driver object for the channel */
  };

}  // namespace Thor::TIMER

#endif  /* !THOR_HLD_TIMER_TYPES_HPP */
