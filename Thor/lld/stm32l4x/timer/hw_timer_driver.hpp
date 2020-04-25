/********************************************************************************
 *  File Name:
 *    hw_timer_driver.hpp
 *
 *  Description:
 *    Declares the LLD interface to the STM32L4 series TIMER hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_TIMER_DRIVER_STM32L4_HPP
#define THOR_HW_TIMER_DRIVER_STM32L4_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/stm32l4x/timer/hw_timer_types.hpp>
#include <Thor/lld/stm32l4x/timer/hw_timer_mapping.hpp>
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>

namespace Thor::LLD::TIMER
{
  class AdvancedDriver : public virtual IAdvancedDriver
  {
  public:
    AdvancedDriver();
    ~AdvancedDriver();

    Chimera::Status_t attach( RegisterMap *const peripheral ) final override;

  private:
    RegisterMap *periph;
  };


  class BasicDriver : public virtual IBasicDriver
  {
  public:
    BasicDriver();
    ~BasicDriver();

    Chimera::Status_t attach( RegisterMap *const peripheral ) final override;

  private:
    RegisterMap *periph;
  };


  class GeneralDriver : public virtual IGeneralDriver
  {
  public:
    GeneralDriver();
    ~GeneralDriver();

    Chimera::Status_t attach( RegisterMap *const peripheral ) final override;

  private:
    RegisterMap *periph;
  };


  class LowPowerDriver : public virtual ILowPowerDriver
  {
  public:
    LowPowerDriver();
    ~LowPowerDriver();

    Chimera::Status_t attach( LPRegisterMap *const peripheral ) final override;

  private:
    LPRegisterMap *periph;
  };
}    // namespace Thor::LLD::TIMER

#endif /* !THOR_HW_TIMER_DRIVER_STM32L4_HPP */
