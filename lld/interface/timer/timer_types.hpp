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
  class Handle;
  struct RegisterMap;
  struct LPRegisterMap;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Handle_rPtr  = Handle *;

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
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Represents state of the three supported timer variants
   *
   * Supports: Basic, Advanced, General timers
   */
  class Handle
  {
  public:
    Chimera::Timer::Instance instance;    /**< Which timer instance this is */
    RegisterMap             *registers;   /**< Memory mapped registers for the instance */
    HardwareType             type;        /**< Type of driver */
    RIndex_t                 globalIndex; /**< Global resource index for all timer peripherals */
    RIndex_t                 typeIndex;   /**< Resource index for specific timer type (General, Advanced, etc.) */

    Handle() : registers( nullptr ), type( TIMER_HW_INVALID )
    {
    }
  private:
  };

}    // namespace Thor::LLD::TIMER

#endif /* !LLD_TIMER_INTERFACE_TYPES */
