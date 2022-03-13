/******************************************************************************
 *  File Name:
 *    lld_timer_driver_intf.cpp
 *
 *  Description:
 *    Interface implementation details for the timer module
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstddef>
#include <Thor/lld/interface/inc/timer>

namespace Thor::LLD::TIMER
{
  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    return 0;
  }
}  // namespace Thor::LLD::TIMER
