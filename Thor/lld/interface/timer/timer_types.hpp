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

namespace Thor::LLD::Timer
{
  /** 
   *  The category of timer peripheral that can exist. Not all
   *  timers have the same capabilities.
   */
  enum class Type
  {
    ADVANCED_TIMER,
    BASIC_TIMER,
    GENERAL_PURPOSE_TIMER,
    LOW_POWER_TIMER,

    NUM_OPTIONS
  };

  /**
   *  Possible actions that a timer can do or behave as
   */
  enum class Functionality
  {
    PWM_GENERATION,

  };


  class IAdvancedDriver;
  using IAdvancedDriver_sPtr = std::shared_ptr<IAdvancedDriver>;
  using IAdvancedDriver_uPtr = std::unique_ptr<IAdvancedDriver>;

  class IBasicDriver;
  using IBasicDriver_sPtr = std::shared_ptr<IBasicDriver>;
  using IBasicDriver_uPtr = std::unique_ptr<IBasicDriver>;

  class IGeneralDriver;
  using IGeneralDriver_sPtr = std::shared_ptr<IGeneralDriver>;
  using IGeneralDriver_uPtr = std::unique_ptr<IGeneralDriver>;

  class ILowPowerDriver;
  using ILowPowerDriver_sPtr = std::shared_ptr<ILowPowerDriver>;
  using ILowPowerDriver_uPtr = std::unique_ptr<ILowPowerDriver>;

}    // namespace Thor::LLD::Timer

#endif  /* !LLD_TIMER_INTERFACE_TYPES */
