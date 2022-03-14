/******************************************************************************
 *  File Name:
 *    lld_timer_control.hpp
 *
 *  Description:
 *    Common driver for Advanced, Basic, General timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_CONTROL_UNIT_DRIVER_HPP
#define THOR_LLD_TIMER_CONTROL_UNIT_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>

namespace Thor::LLD::TIMER
{
  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /**
   * @brief Shared functionality among the General/Basic/Advanced drivers
   */
  template<class Derived>
  class ControlUnit
  {
  public:
    ControlUnit()
    {
    }

    ~ControlUnit()
    {
    }

    inline void enableCounter()
    {
      CEN::set( static_cast<Derived *>( this )->mPeriph, CR1_CEN );
    }

    inline void disableCounter()
    {
      CEN::clear( static_cast<Derived *>( this )->mPeriph, CR1_CEN );
    }

    inline void ctlBufferARREnable()
    {
      ARPE::set( static_cast<Derived *>( this )->mPeriph, CR1_ARPE );
    }
  };

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_CONTROL_UNIT_DRIVER_HPP */
