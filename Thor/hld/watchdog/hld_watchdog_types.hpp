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
  class Window;
  class Independent;


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using WindowDriver_sPtr = std::shared_ptr<Window>;
  using IndependentDriver_sPtr = std::shared_ptr<Independent>;

}  // namespace Thor::Watchdog

#endif  /* !THOR_HLD_WATCHDOG_TYPES_HPP */
