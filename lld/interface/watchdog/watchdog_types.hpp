/******************************************************************************
 *  File Name:
 *    watchdog_types.hpp
 *
 *  Description:
 *    LLD types for the WATCHDOG drivers
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef LLD_WATCHDOG_TYPES_HPP
#define LLD_WATCHDOG_TYPES_HPP

namespace Thor::LLD::Watchdog
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class IndependentDriver;
  struct IRegisterMap;

  class WindowDriver;
  struct WRegisterMap;


  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using IndependentDriver_rPtr = IndependentDriver *;
  using WindowDriver_rPtr = WindowDriver *;

}    // namespace Thor::LLD::WATCHDOG

#endif /* !LLD_WATCHDOG_TYPES_HPP */
