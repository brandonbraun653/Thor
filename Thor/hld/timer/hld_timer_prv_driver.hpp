/********************************************************************************
 *  File Name:
 *    hld_timer_prv_driver.hpp
 *
 *  Description:
 *    Internal interface for the HLD timer driver
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HLD_TIMER_PRIVATE_DRIVER_HPP
#define THOR_HLD_TIMER_PRIVATE_DRIVER_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/hld/common/types.hpp>
#include <Thor/hld/timer/hld_timer_types.hpp>
#include <Thor/lld/interface/timer/timer_detail.hpp>

namespace Thor::TIMER
{
  /*-------------------------------------------------
  External memory used to hold instances of the HLDs
  -------------------------------------------------*/
  extern std::array<AdvancedDriver_sPtr, Thor::LLD::TIMER::NUM_ADVANCED_PERIPHS> hld_advanced_drivers;
  extern std::array<BasicDriver_sPtr, Thor::LLD::TIMER::NUM_BASIC_PERIPHS> hld_basic_drivers;
  extern std::array<GeneralDriver_sPtr, Thor::LLD::TIMER::NUM_GENERAL_PERIPHS> hld_general_drivers;
  extern std::array<LowPowerDriver_sPtr, Thor::LLD::TIMER::NUM_LOW_POWER_PERIPHS> hld_low_power_drivers;

  /*-------------------------------------------------
  Private Free Functions
  -------------------------------------------------*/
  bool isInitialized();
  Chimera::Status_t initAdvancedDriverModule();
  Chimera::Status_t initAdvancedDriverObject( const Thor::HLD::RIndex hld_index );
  
  Chimera::Status_t initBasicDriverModule();
  Chimera::Status_t initBasicDriverObject( const Thor::HLD::RIndex hld_index );

  Chimera::Status_t initGeneralDriverModule();
  Chimera::Status_t initGeneralDriverObject( const Thor::HLD::RIndex hld_index );

  Chimera::Status_t initLowPowerDriverModule();
  Chimera::Status_t initLowPowerDriverObject( const Thor::HLD::RIndex hld_index );

  /*-------------------------------------------------
  Helper functions for Chimera hooks
  -------------------------------------------------*/
  Chimera::Timer::ITimer_rPtr lookUpRawPointer( const Chimera::Timer::Peripheral peripheral );
  Chimera::Timer::ITimer_sPtr lookUpSharedPointer( const Chimera::Timer::Peripheral peripheral );
}    // namespace Thor::TIMER

#endif /* !THOR_HLD_TIMER_PRIVATE_DRIVER_HPP */
