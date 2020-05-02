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

/* Thor Includes */
#include <Thor/hld/timer/hld_timer_types.hpp>

namespace Thor::TIMER
{
  /*-------------------------------------------------------------------------------
  Chimera Based Free Function Declarations (see Chimera::Timer for documentation)
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize();

  Chimera::Status_t reset();

  void incrementSystemTick();

  size_t millis();

  size_t micros();

  void delayMilliseconds( const size_t val );

  void delayMicroseconds( const size_t val );

  /*-------------------------------------------------------------------------------
  High Level Driver Declaration
  -------------------------------------------------------------------------------*/
  class AdvancedDriver : virtual public Chimera::Timer::ITimer,
                         public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Advanced Driver Interface
    ------------------------------------------------*/
    AdvancedDriver();
    ~AdvancedDriver();

    /*-------------------------------------------------
    Chimera ITimer Interface
    -------------------------------------------------*/
    Chimera::Status_t initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init ) final override;
    Chimera::Status_t invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize ) final override;
    Chimera::Status_t setState( const Chimera::Timer::Switchable device, const Chimera::Timer::SwitchableState state ) final override;
    Chimera::Status_t requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize ) final override;
    const Chimera::Timer::Descriptor * getDeviceInfo() final override;

  private:
    friend AdvancedDriver_sPtr getAdvancedDriver_sPtr( const Chimera::Timer::Peripheral, const bool );
    size_t mResourceIndex;
  };

  class GeneralDriver : virtual public Chimera::Timer::ITimer,
                        public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    General Driver Interface
    ------------------------------------------------*/
    GeneralDriver();
    ~GeneralDriver();

    /*-------------------------------------------------
    Chimera ITimer Interface
    -------------------------------------------------*/
    Chimera::Status_t initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init ) final override;
    Chimera::Status_t invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize ) final override;
    Chimera::Status_t setState( const Chimera::Timer::Switchable device, const Chimera::Timer::SwitchableState state ) final override;
    Chimera::Status_t requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize ) final override;
    const Chimera::Timer::Descriptor * getDeviceInfo() final override;

  private:
    friend GeneralDriver_sPtr getGeneralDriver_sPtr( const Chimera::Timer::Peripheral, const bool );
    size_t mResourceIndex;
  };

  class BasicDriver : virtual public Chimera::Timer::ITimer,
                      public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Basic Driver Interface
    ------------------------------------------------*/
    BasicDriver();
    ~BasicDriver();

    /*-------------------------------------------------
    Chimera ITimer Interface
    -------------------------------------------------*/
    Chimera::Status_t initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init ) final override;
    Chimera::Status_t invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize ) final override;
    Chimera::Status_t setState( const Chimera::Timer::Switchable device, const Chimera::Timer::SwitchableState state ) final override;
    Chimera::Status_t requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize ) final override;
    const Chimera::Timer::Descriptor * getDeviceInfo() final override;

  private:
    friend BasicDriver_sPtr getBasicDriver_sPtr( const Chimera::Timer::Peripheral, const bool );
    size_t mResourceIndex;
  };

  class LowPowerDriver : virtual public Chimera::Timer::ITimer,
                         public Chimera::Threading::Lockable
  {
  public:
    /*------------------------------------------------
    Low Power Driver Interface
    ------------------------------------------------*/
    LowPowerDriver();
    ~LowPowerDriver();

    /*-------------------------------------------------
    Chimera ITimer Interface
    -------------------------------------------------*/
    Chimera::Status_t initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init ) final override;
    Chimera::Status_t invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize ) final override;
    Chimera::Status_t setState( const Chimera::Timer::Switchable device, const Chimera::Timer::SwitchableState state ) final override;
    Chimera::Status_t requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize ) final override;
    const Chimera::Timer::Descriptor * getDeviceInfo() final override;

  private:
    friend LowPowerDriver_sPtr getLowPowerDriver_sPtr( const Chimera::Timer::Peripheral, const bool );
    size_t mResourceIndex;
  };
}    // namespace Thor::TIMER

#endif /* !THOR_HLD_TIMER_DRIVER_HPP */