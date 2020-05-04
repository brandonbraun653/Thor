/********************************************************************************
 *  File Name:
 *    timer_types.hpp
 *
 *  Description:
 *    LLD Timer Interface Types
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef LLD_TIMER_INTERFACE_TYPES
#define LLD_TIMER_INTERFACE_TYPES

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/algorithm>
#include <Chimera/container>
#include <Chimera/timer>

/* Thor Includes */
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/common/types.hpp>

namespace Thor::LLD::TIMER
{

  /**
   *  The category of timer peripheral that can exist. Not all
   *  timers have the same capabilities.
   */
  enum class Type : uint8_t
  {
    INVALID,
    ADVANCED_TIMER,
    BASIC_TIMER,
    GENERAL_PURPOSE_TIMER,
    LOW_POWER_TIMER,

    NUM_OPTIONS
  };

  /**
   *  Possible actions that a timer can do or behave as
   */
  enum class Functionality : uint8_t
  {
    PWM_GENERATION,

  };

  /**
   *  Forward declaration to ease compilation
   */
  struct RegisterMap;
  struct LPRegisterMap;

  struct DeviceDescription
  {
    uint8_t counterWidth;
    uint8_t numChannels;
    Type timerType;

    void *registerMap;

    const Chimera::Algorithm::OptionsList* supportedEvents;
    const Chimera::Algorithm::OptionsList* supportedModes;
  };

  class IAdvancedDriver;
  using IAdvancedDriver_sPtr = IAdvancedDriver*;
  using IAdvancedDriver_uPtr = IAdvancedDriver*;

  class IBasicDriver;
  using IBasicDriver_sPtr = IBasicDriver*;
  using IBasicDriver_uPtr = IBasicDriver*;


  class ILowPowerDriver;
  using ILowPowerDriver_sPtr = ILowPowerDriver*;
  using ILowPowerDriver_uPtr = ILowPowerDriver*;


  class GeneralDriverImpl;
  using GeneralDriver_rPtr = GeneralDriverImpl*;

  /*------------------------------------------------
  Flat Map Data Types
  ------------------------------------------------*/
  // Instance To Resource Index Map
  using ITRIMap = Chimera::Container::LightFlatMap<std::uintptr_t, size_t>;

  // Peripheral To Resource Index Map
  using PTRIMapLLD = Chimera::Container::LightFlatMap<Chimera::Timer::Peripheral, Thor::LLD::RIndex>;
  using PTRIMapHLD = Chimera::Container::LightFlatMap<Chimera::Timer::Peripheral, Thor::HLD::RIndex>;
  
  /*------------------------------------------------
  Look Up Table Types
  ------------------------------------------------*/
  using DirectionConverter = std::array<Reg32_t, static_cast<size_t>( Chimera::Timer::Direction::NUM_OPTIONS )>;

}    // namespace Thor::LLD::Timer

#endif  /* !LLD_TIMER_INTERFACE_TYPES */
