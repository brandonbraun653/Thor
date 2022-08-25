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
  Independent_rPtr getDriver( const IChannel channel );
  Window_rPtr getDriver( const WChannel channel );
}    // namespace Chimera::Watchdog::Backend

#endif /* !THOR_WATCHDOG_CHIMERA_HOOKS_HPP */
