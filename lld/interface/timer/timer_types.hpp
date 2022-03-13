/********************************************************************************
 *  File Name:
 *    timer_types.hpp
 *
 *  Description:
 *    LLD Timer Interface Types
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef LLD_TIMER_INTERFACE_TYPES
#define LLD_TIMER_INTERFACE_TYPES

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/algorithm>
#include <Chimera/container>
#include <Chimera/timer>
#include <Thor/hld/common/types.hpp>
#include <Thor/lld/common/types.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class AdvancedDriver;
  class BasicDriver;
  class GeneralDriver;

  struct RegisterMap;
  struct LPRegisterMap;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using AdvancedDriver_rPtr = AdvancedDriver *;
  using BasicDriver_rPtr    = BasicDriver *;
  using GeneralDriver_rPtr   = GeneralDriver *;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
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

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct DeviceDescription
  {
    uint8_t counterWidth;
    uint8_t numChannels;
    Type timerType;

    void *registerMap;

    const Chimera::Algorithm::OptionsList *supportedEvents;
    const Chimera::Algorithm::OptionsList *supportedModes;
  };

}    // namespace Thor::LLD::TIMER

#endif /* !LLD_TIMER_INTERFACE_TYPES */
