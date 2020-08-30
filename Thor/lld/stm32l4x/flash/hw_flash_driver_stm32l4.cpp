/********************************************************************************
 *  File Name:
 *    hw_flash_driver_STM32L4.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series FLASH hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/flash/hw_flash_driver.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_mapping.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_prj.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_driver.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_FLASH )

namespace Thor::LLD::FLASH
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    initializeRegisters();
    initializeMapping();

    return Chimera::Status::OK;
  }

  size_t availableChannels()
  {
    return NUM_FLASH_PERIPHS;
  }


  RIndexType getResourceIndex( const std::uintptr_t address )
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

  /*-------------------------------------------------
  Private LLD Function Implementation
  -------------------------------------------------*/
  bool isFLASH( const std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
      }
    }

    return result;
  }

  /*-----------------------------------------------------
  Low Level Driver Implementation
  -----------------------------------------------------*/

}    // namespace Thor::LLD::FLASH

#endif /* TARGET_STM32L4 && THOR_DRIVER_FLASH */
