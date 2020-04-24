/********************************************************************************
 *  File Name:
 *    hld_timer_driver.hpp
 *
 *  Description:
 *    Thor high level driver for Timer
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_TIMER_DRIVER_HPP 
#define THOR_HLD_TIMER_DRIVER_HPP 

/* STL Includes */
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/event>
#include <Chimera/timer>
#include <Chimera/thread>

namespace Thor::Timer
{
  Chimera::Status_t initialize();

  Chimera::Status_t reset();

  void incrementSystemTick();

  size_t millis();

  void delayMilliseconds( const size_t val );

  void delayMicroseconds( const size_t val );


  class Driver : public virtual Chimera::Timer::ITimer, public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Class Functions
    ------------------------------------------------*/
    Driver();
    ~Driver();

    /*------------------------------------------------
    HW Interface
    ------------------------------------------------*/
    void enable() final override;
    void disable() final override;
    Chimera::Status_t initPeripheral( const Chimera::Timer::DriverConfig &cfg ) final override;
    Chimera::Status_t enableISR( const Chimera::Timer::ISREvent type ) final override;
    Chimera::Status_t disableISR( const Chimera::Timer::ISREvent type ) final override;
    Chimera::Status_t setupInputCapture( const Chimera::Timer::InputCapture::Config &cfg ) final override;
    Chimera::Status_t setupOutputCompare( const Chimera::Timer::OutputCompare::Config &cfg ) final override;
    Chimera::Status_t setupPWMGeneration( const Chimera::Timer::PWMGeneration::Config &cfg ) final override;
    Chimera::Status_t setupOnePulse( const Chimera::Timer::OnePulse::Config &cfg ) final override;
    size_t counterBitWidth() final override;
    size_t tickRate( const Chimera::Units::Time units ) final override;
    size_t maxPeriod( const Chimera::Units::Time units ) final override;
    size_t minPeriod( const Chimera::Units::Time units ) final override;
    Chimera::Timer::Mode currentMode() final override;

    /*------------------------------------------------
    Listener Interface
    ------------------------------------------------*/
    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                        size_t &registrationID ) final override;
    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout ) final override;

  private:
    Chimera::Event::ActionableList eventListeners;
  };
}    // namespace Thor::Timer

#endif  /* !THOR_HLD_TIMER_DRIVER_HPP */