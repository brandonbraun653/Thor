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
#include <Chimera/pwm>
#include <Chimera/thread>

namespace Thor::PWM
{
  Chimera::Status_t initialize();

  class Driver : virtual public Chimera::PWM::IPWM, public Chimera::Threading::Lockable
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
    void enableOutput() final override;
    void disableOutput() final override;
    Chimera::Status_t setFrequency( const size_t freq ) final override;
    Chimera::Status_t setDutyCyle( const size_t dutyCycle ) final override;

  private:
  };

}    // namespace Thor::PWM

#endif /* PWM_H_*/
