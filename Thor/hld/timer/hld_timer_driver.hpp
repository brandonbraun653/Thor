/********************************************************************************
 *  File Name:
 *    hld_timer_driver.hpp
 *
 *  Description:
 *    Thor HLD for Timer perihperals. ST seems to have decided that timers can
 *    be divided into four main categories: Advanced, Basic, General, and Low Power.
 *    The is module provides support for all of these types while keeping a common
 *    interface to handle injection with Chimera based software.
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
#include <Thor/hld/common/types.hpp>
#include <Thor/hld/timer/hld_timer_types.hpp>
#include <Thor/lld/common/types.hpp>

namespace Thor::TIMER
{
  /*-------------------------------------------------------------------------------
  Chimera Based Free Function Declarations (see Chimera::Timer for documentation)
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initializeModule();
  Chimera::Status_t resetModule();
  void incrementSystemTick();
  size_t millis();
  size_t micros();
  void delayMilliseconds( const size_t val );
  void delayMicroseconds( const size_t val );

  /*-------------------------------------------------------------------------------
  High Level Driver Declaration
  -------------------------------------------------------------------------------*/
  // class AdvancedDriver : virtual public Chimera::Timer::HWInterface,
  //                        public Chimera::Threading::Lockable
  // {
  // public:
  //   /*------------------------------------------------
  //   Advanced Driver Interface
  //   ------------------------------------------------*/
  //   AdvancedDriver();
  //   ~AdvancedDriver();

  //   /*-------------------------------------------------
  //   Chimera ITimer Interface
  //   -------------------------------------------------*/
  //   Chimera::Status_t initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init ) final override;
  //   Chimera::Status_t invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize ) final override;
  //   Chimera::Status_t setState( const Chimera::Timer::Switchable device, const Chimera::Timer::SwitchableState state ) final override;
  //   Chimera::Status_t requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize ) final override;
  //   const Chimera::Timer::Descriptor * getDeviceInfo() final override;

  // private:
  //   friend Chimera::Status_t initAdvancedDriverObject( const Thor::HLD::RIndex );
  //   Thor::HLD::RIndex mIndexHLD;
  //   Thor::LLD::RIndex mIndexLLD;
  // };

  // class GeneralDriver : virtual public Chimera::Timer::HWInterface,
  //                       public Chimera::Threading::Lockable
  // {
  // public:
  //   /*-------------------------------------------------
  //   Chimera ITimer Interface
  //   -------------------------------------------------*/
  //   Chimera::Status_t initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init ) final override;
  //   Chimera::Status_t invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize ) final override;
  //   Chimera::Status_t setState( const Chimera::Timer::Switchable device, const Chimera::Timer::SwitchableState state ) final override;
  //   Chimera::Status_t requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize ) final override;
  //   const Chimera::Timer::Descriptor * getDeviceInfo() final override;

  //   /*------------------------------------------------
  //   General Driver Interface
  //   ------------------------------------------------*/
  //   GeneralDriver();
  //   ~GeneralDriver();

  //   Chimera::Status_t initCoreTimer( const Chimera::Timer::DriverConfig &cfg );

  //   /*------------------------------------------------
  //   PWM Functionality
  //   ------------------------------------------------*/
  //   Chimera::Status_t initPWM( const Chimera::Timer::PWM::Config &cfg );
  //   Chimera::Status_t toggleOutput( const Chimera::Timer::Channel channel, const bool state );


  // private:
  //   friend Chimera::Status_t initGeneralDriverObject( const Thor::HLD::RIndex );
  //   Thor::HLD::RIndex mIndexHLD;
  //   Thor::LLD::RIndex mIndexLLD;
  // };

  // class BasicDriver : virtual public Chimera::Timer::HWInterface,
  //                     public Chimera::Threading::Lockable
  // {
  // public:
  //   /*------------------------------------------------
  //   Basic Driver Interface
  //   ------------------------------------------------*/
  //   BasicDriver();
  //   ~BasicDriver();

  //   /*-------------------------------------------------
  //   Chimera ITimer Interface
  //   -------------------------------------------------*/
  //   Chimera::Status_t initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init ) final override;
  //   Chimera::Status_t invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize ) final override;
  //   Chimera::Status_t setState( const Chimera::Timer::Switchable device, const Chimera::Timer::SwitchableState state ) final override;
  //   Chimera::Status_t requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize ) final override;
  //   const Chimera::Timer::Descriptor * getDeviceInfo() final override;

  // private:
  //   friend Chimera::Status_t initBasicDriverObject( const Thor::HLD::RIndex );
  //   Thor::HLD::RIndex mIndexHLD;
  //   Thor::LLD::RIndex mIndexLLD;
  // };

  // class LowPowerDriver : virtual public Chimera::Timer::HWInterface,
  //                        public Chimera::Threading::Lockable
  // {
  // public:
  //   /*------------------------------------------------
  //   Low Power Driver Interface
  //   ------------------------------------------------*/
  //   LowPowerDriver();
  //   ~LowPowerDriver();

  //   /*-------------------------------------------------
  //   Chimera ITimer Interface
  //   -------------------------------------------------*/
  //   Chimera::Status_t initializeCoreFeature( const Chimera::Timer::CoreFeature feature, Chimera::Timer::CoreFeatureInit &init ) final override;
  //   Chimera::Status_t invokeAction( const Chimera::Timer::DriverAction action, void *arg, const size_t argSize ) final override;
  //   Chimera::Status_t setState( const Chimera::Timer::Switchable device, const Chimera::Timer::SwitchableState state ) final override;
  //   Chimera::Status_t requestData( const Chimera::Timer::DriverData data, void *arg, const size_t argSize ) final override;
  //   const Chimera::Timer::Descriptor * getDeviceInfo() final override;

  // private:
  //   friend Chimera::Status_t initLowPowerDriverObject( const Thor::HLD::RIndex );
  //   Thor::HLD::RIndex mIndexHLD;
  //   Thor::LLD::RIndex mIndexLLD;
  // };
}    // namespace Thor::TIMER

#endif /* !THOR_HLD_TIMER_DRIVER_HPP */