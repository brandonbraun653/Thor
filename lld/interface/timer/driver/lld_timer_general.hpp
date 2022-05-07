/******************************************************************************
 *  File Name:
 *    lld_timer_general.hpp
 *
 *  Description:
 *    General timer driver interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_GENERAL_HPP
#define THOR_LLD_TIMER_GENERAL_HPP

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
  GeneralDriver_rPtr getGeneralDriver( const Chimera::Timer::Instance instance );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class GeneralDriver : public ModuleDriver<GeneralDriver>, public ControlUnit<GeneralDriver>, public TimeBase<GeneralDriver>
  {
  public:
    GeneralDriver();
    ~GeneralDriver();

    constexpr HardwareType timerType()
    {
      return HardwareType::TIMER_HW_GENERAL;
    }

  protected:
    friend ModuleDriver<GeneralDriver>;

  private:

  };
}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_GENERAL_HPP */
