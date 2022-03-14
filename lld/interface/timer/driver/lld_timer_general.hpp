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
#include <Thor/lld/interface/timer/driver/lld_timer_common.hpp>
#include <Thor/lld/interface/timer/driver/lld_timer_control.hpp>
#include <Thor/lld/interface/timer/driver/lld_timer_time_base.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the general driver module
   */
  void initGeneralDriver();

  /**
   * @brief Get the General Driver object
   *
   * @param typeIndex   Peripheral type resource index
   * @return GeneralDriver_rPtr
   */
  GeneralDriver_rPtr getGeneralDriver( const RIndex_t typeIndex );

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
      return HardwareType::GENERAL;
    }

  protected:
    friend ModuleDriver<GeneralDriver>;

  private:

  };
}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_GENERAL_HPP */
