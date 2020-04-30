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


namespace Thor::LLD::TIMER
{

  /**
   *  The category of timer peripheral that can exist. Not all
   *  timers have the same capabilities.
   */
  enum class Type : uint8_t
  {
    ADVANCED_TIMER,
    BASIC_TIMER,
    GENERAL_PURPOSE_TIMER,
    LOW_POWER_TIMER,

    NUM_OPTIONS,
    INVALID
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

//  class IAdvancedDriver;
//  using IAdvancedDriver_sPtr = std::shared_ptr<IAdvancedDriver>;
//  using IAdvancedDriver_uPtr = std::unique_ptr<IAdvancedDriver>;
//
//  class IBasicDriver;
//  using IBasicDriver_sPtr = std::shared_ptr<IBasicDriver>;
//  using IBasicDriver_uPtr = std::unique_ptr<IBasicDriver>;
//
//  class IGeneralDriver;
//  using IGeneralDriver_sPtr = std::shared_ptr<IGeneralDriver>;
//  using IGeneralDriver_uPtr = std::unique_ptr<IGeneralDriver>;
//
//  class ILowPowerDriver;
//  using ILowPowerDriver_sPtr = std::shared_ptr<ILowPowerDriver>;
//  using ILowPowerDriver_uPtr = std::unique_ptr<ILowPowerDriver>;

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
  #if defined( VIRTUAL_FUNC )
  class IGeneralDriver;
  using IGeneralDriver_sPtr = IGeneralDriver*;
  using IGeneralDriver_uPtr = IGeneralDriver*;

  #else
  template<class T>
  class IGeneralDriver;

  using IGeneralDriver_sPtr = IGeneralDriver<GeneralDriverImpl>*;
  using IGeneralDriver_uPtr = IGeneralDriver<GeneralDriverImpl>*;
  #endif 

  /*-------------------------------------------------
  General Driver Types
  -------------------------------------------------*/
  // // Interface defined in timer_intf.hpp
  // template<class T>
  // class IGeneralDriver;

  // template<class T>
  // using GeneralDriver = IGeneralDriver<T>;

  // template<class T>
  // using GeneralDriver_sPtr = std::shared_ptr<GeneralDriver<T>>;

  // template<class T>
  // using GeneralDriver_uPtr = std::unique_ptr<GeneralDriver<T>>;

  /*------------------------------------------------
  Flat Map Data Types
  ------------------------------------------------*/
  // Instance To Resource Index Map
  using ITRIMap = Chimera::Container::LightFlatMap<std::uintptr_t, size_t>;

  // Peripheral To Resource Index Map
  using PTRIMap = Chimera::Container::LightFlatMap<Chimera::Timer::Peripheral, size_t>;

  /*------------------------------------------------
  Look Up Table Types
  ------------------------------------------------*/


}    // namespace Thor::LLD::Timer

#endif  /* !LLD_TIMER_INTERFACE_TYPES */
