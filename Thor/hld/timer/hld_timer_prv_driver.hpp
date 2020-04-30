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

/* Thor Includes */
/**
 *  It's ok to include the LLD details here as this private header will
 *  only be seen in source files for this driver. It should never be
 *  included into public headers.
 */
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
  Chimera::Status_t initializeAdvanced();
  Chimera::Status_t initializeBasic();
  Chimera::Status_t initializeGeneral();
  Chimera::Status_t initializeLowPower();

  void *getDriverAddress( const Chimera::Timer::Peripheral periph, const size_t hld_resource_index );

  /*------------------------------------------------
  hld_timer_chimera.cpp
  ------------------------------------------------*/
  Chimera::Timer::TimerBaseImpl *getBaseImplDriver( const size_t resourceIndex );
  Chimera::Timer::TimerEncoderImpl *getEncoderImplDriver( const size_t resourceIndex );
  Chimera::Timer::TimerInputCaptureImpl *getInputCaptureImplDriver( const size_t resourceIndex );
  Chimera::Timer::TimerOnePulseImpl *getOnePulseImplDriver( const size_t resourceIndex );
  Chimera::Timer::TimerOutputCompareImpl *getOutputCompareImplDriver( const size_t resourceIndex );

}    // namespace Thor::TIMER

#endif /* !THOR_HLD_TIMER_PRIVATE_DRIVER_HPP */
