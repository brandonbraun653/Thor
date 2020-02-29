/********************************************************************************
 *  File Name:
 *    hld_watchdog_driver.hpp
 *
 *  Description:
 *    Implements Chimera hooks for the watchdog
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_WATCHDOG_CHIMERA_HOOKS_HPP
#define THOR_WATCHDOG_CHIMERA_HOOKS_HPP

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/watchdog>

namespace Chimera::Watchdog::Backend
{
  Chimera::Status_t initialize();

  Chimera::Status_t reset();

  Chimera::Watchdog::Watchdog_sPtr create_shared_ptr();

  Chimera::Watchdog::Watchdog_uPtr create_unique_ptr();
}    // namespace Chimera::Watchdog::Backend

#endif /* !THOR_WATCHDOG_CHIMERA_HOOKS_HPP */
