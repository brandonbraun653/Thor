/******************************************************************************
 *  File Name:
 *    hw_power_driver_STM32L4.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series POWER hardware.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_prj.hpp>
#include <Thor/lld/stm32l4x/power/hw_power_types.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_PWR )

namespace Thor::LLD::PWR
{
  /*-------------------------------------------------
  LLD->HLD Interface Implementation
  -------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Chimera::Status::OK;
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    if ( address == PWR_BASE_ADDR )
    {
      return PWR_RESOURCE_INDEX;
    }
    else
    {
      return INVALID_RESOURCE_INDEX;
    }
  }

}    // namespace Thor::LLD::POWER

#endif /* TARGET_STM32L4 && THOR_DRIVER_POWER */
