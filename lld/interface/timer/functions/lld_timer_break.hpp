/******************************************************************************
 *  File Name:
 *    lld_timer_break.hpp
 *
 *  Description:
 *    Break and Dead Time register control for a Timer
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_BREAK_AND_DEAD_TIME_HPP
#define THOR_LLD_TIMER_BREAK_AND_DEAD_TIME_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Thor/lld/interface/timer/timer_types.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum BreakSource : uint32_t
  {
    BREAK_SOURCE_INTERNAL,
    BREAK_SOURCE_EXTERNAL
  };

  enum BreakChannel : uint32_t
  {
    BREAK_INPUT_1,
    BREAK_INPUT_2
  };

  enum BreakPolarity : uint32_t
  {
    BREAK_ACTIVE_LOW,
    BREAK_ACTIVE_HIGH
  };

  enum LockoutLevel : uint32_t
  {
    LOCK_LEVEL_OFF,
    LOCK_LEVEL_1,
    LOCK_LEVEL_2,
    LOCK_LEVEL_3
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void generateBreakEvent( Handle_rPtr timer, const BreakChannel channel );

  void breakEnable( Handle_rPtr timer, const BreakSource src, const BreakChannel channel );

  void setBreakPolarity( Handle_rPtr timer, const BreakSource src, const BreakChannel channel, const BreakPolarity polarity );

  void lockoutTimer( Handle_rPtr timer, const LockoutLevel level );

  bool isLockedOut(Handle_rPtr timer );

  bool setDeadTime( Handle_rPtr timer, const float dt_ns );

}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_BREAK_AND_DEAD_TIME_HPP */
