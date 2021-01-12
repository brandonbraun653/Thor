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


  class GeneralDriverImpl
  {
  public:
    GeneralDriverImpl();
    ~GeneralDriverImpl();

    /**
     *  Resets the hardware registers back to boot-up values
     *
     *  @return Chimera::Status_t
     */
    Chimera::Status_t reset();

    /**
     *  Enables the peripheral clock
     *
     *  @return void
     */
    void clockEnable();

    /**
     *  Disables the peripheral clock
     *
     *  @return void
     */
    void clockDisable();

    void toggleCounter( const bool state );

    void toggleChannel( const Chimera::Timer::Channel channel, const bool state );

    Chimera::Status_t attach( RegisterMap *const peripheral );

    Chimera::Status_t initBaseTimer( const Chimera::Timer::DriverConfig &cfg );

    Chimera::Status_t initPWM( const Chimera::Timer::PWM::Config &cfg );


    void setCaptureComparePolarity( const Chimera::Timer::Channel channel, const Reg32_t val );
    void setCaptureCompareMatch( const Chimera::Timer::Channel channel, const Reg32_t val );
    void setCaptureCompareDirection( const Chimera::Timer::Channel channel, const Reg32_t val );

    void setOutputCompareMode( const Chimera::Timer::Channel channel, const Reg32_t val );
    void setOutputComparePreload( const Chimera::Timer::Channel channel, const Reg32_t val );

  private:
    RegisterMap *mpPeriph;
    //RIndex mRIndex;
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
