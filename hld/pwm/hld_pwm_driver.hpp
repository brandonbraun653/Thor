/********************************************************************************
 *  File Name:
 *    hld_pwm_driver.hpp
 *
 *  Description:
 *    Thor PWM high level driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_PWM_HPP
#define THOR_PWM_HPP

/* C/C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/pwm>
#include <Chimera/thread>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/gpio>
#include <Thor/timer>

namespace Thor::PWM
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Chimera::PWM::IPWM_sPtr getDriver( const size_t channel );
  size_t numSupportedChannels();


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  #if 0
  class Driver : public Chimera::Thread::Lockable<Driver>
  {
  public:
    /*------------------------------------------------
    Class Specific Functions
    ------------------------------------------------*/
    Driver();
    ~Driver();

    /*------------------------------------------------
    HW Interface
    ------------------------------------------------*/
    Chimera::Status_t init( const Chimera::PWM::DriverConfig &cfg ) final override;
    Chimera::Status_t toggleOutput( const bool state ) final override;
    Chimera::Status_t setFrequency( const size_t freq ) final override;
    Chimera::Status_t setDutyCyle( const size_t dutyCycle ) final override;
    Chimera::Status_t setPolarity( const Chimera::Timer::PWM::Polarity polarity ) final override;

  private:
    friend Chimera::Thread::Lockable<Driver>;


    bool mInitialized;
    Chimera::Timer::Instance mPeripheral;
    Chimera::Timer::PWM::Config mPWMConfig;

    Thor::GPIO::Driver_rPtr mpOutputPin;
    Chimera::Timer::Driver_rPtr mpTimerDriver;

    Chimera::Status_t applyConfig( const size_t freq, const size_t dutyCycle, const Chimera::Timer::PWM::Polarity polarity );
  };
  #endif
}    // namespace Thor::PWM

#endif /* !THOR_PWM_HPP*/
