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
  using GeneralDriver_rPtr  = GeneralDriver *;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   * @brief Enumerates available hardware driver types
   */
  enum class HardwareType : uint8_t
  {
    ADVANCED,
    BASIC,
    GENERAL,

    NUM_OPTIONS,
    INVALID
  };

}    // namespace Thor::LLD::TIMER

#endif /* !LLD_TIMER_INTERFACE_TYPES */
