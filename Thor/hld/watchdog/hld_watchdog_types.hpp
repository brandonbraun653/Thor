/********************************************************************************
 *  File Name:
 *    hld_watchdog_types.hpp
 *
 *  Description:
 *    Watchdog Types
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_WATCHDOG_TYPES_HPP
#define THOR_HLD_WATCHDOG_TYPES_HPP

/* STL Includes */
#include <memory>

namespace Thor::Watchdog
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class WindowDriver;
  class IndependentDriver;


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using WindowDriver_rPtr = WindowDriver *;
  using WindowDriver_sPtr = std::shared_ptr<WindowDriver>;

  using IndependentDriver_rPtr = IndependentDriver *;
  using IndependentDriver_sPtr = std::shared_ptr<IndependentDriver>;

}    // namespace Thor::Watchdog

#endif /* !THOR_HLD_WATCHDOG_TYPES_HPP */
