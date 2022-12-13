/******************************************************************************
 *  File Name:
 *    hw_flash_driver_STM32L4.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series FLASH hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/interface/flash/flash_prv_data.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_types.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_FLASH )

namespace Thor::LLD::FLASH
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Chimera::Status::OK;
  }

  size_t availableChannels()
  {
    return NUM_FLASH_PERIPHS;
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    if ( address == FLASH_BASE_ADDR )
    {
      return FLASH_RESOURCE_INDEX;
    }
    else
    {
      return INVALID_RESOURCE_INDEX;
    }
  }

}    // namespace Thor::LLD::FLASH

#endif /* TARGET_STM32L4 && THOR_DRIVER_FLASH */
