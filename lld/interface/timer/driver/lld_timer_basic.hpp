/******************************************************************************
 *  File Name:
 *    lld_timer_basic.hpp
 *
 *  Description:
 *    Basic timer driver interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_BASIC_HPP
#define THOR_LLD_TIMER_BASIC_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <Thor/lld/interface/timer/functions/lld_timer_common.hpp>
#include <Thor/lld/interface/timer/functions/lld_timer_control.hpp>
#include <Thor/lld/interface/timer/functions/lld_timer_time_base.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Get the General Driver object
   *
   * @param instance   Peripheral hardware instance
   * @return GeneralDriver_rPtr
   */
  BasicDriver_rPtr getBasicDriver( const Chimera::Timer::Instance instance );

}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_BASIC_HPP */
