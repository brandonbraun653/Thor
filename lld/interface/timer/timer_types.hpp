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
  struct UnifiedDriver;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using AdvancedDriver_rPtr = AdvancedDriver *;
  using BasicDriver_rPtr    = BasicDriver *;
  using GeneralDriver_rPtr  = GeneralDriver *;
  using UnifiedDriver_rPtr  = UnifiedDriver *;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   * @brief Enumerates available hardware driver types
   */
  enum HardwareType : size_t
  {
    TIMER_HW_INVALID  = 0,
    TIMER_HW_ADVANCED = ( 1u << 0 ),
    TIMER_HW_BASIC    = ( 1u << 1 ),
    TIMER_HW_GENERAL  = ( 1u << 2 ),

    NUM_OPTIONS = 3
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Single object to pass around one of the 3 supported timer drivers
   *
   */
  struct UnifiedDriver
  {
    union _XDriver
    {
      AdvancedDriver_rPtr advanced;
      BasicDriver_rPtr    basic;
      GeneralDriver_rPtr  general;
    } driver;          /**< Driver instance */
    HardwareType type; /**< Type of driver */

    UnifiedDriver() : type( TIMER_HW_INVALID )
    {
    }
  };

}    // namespace Thor::LLD::TIMER

#endif /* !LLD_TIMER_INTERFACE_TYPES */
