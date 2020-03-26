/********************************************************************************
 *  File Name:
 *    hw_flash_mapping.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/flash/hw_flash_mapping.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_FLASH )

namespace Thor::LLD::FLASH
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;

  /*-------------------------------------------------
  Module Functions 
  -------------------------------------------------*/
  void initializeMapping()
  {
    PeripheralList[ FLASH_RESOURCE_INDEX ] = FLASH_PERIPH;
  }

  /*-------------------------------------------------
  Initialize the Chimera Option to Register Config Mappings
  -------------------------------------------------*/
  /* clang-format off */
  // const std::array<uint32_t, static_cast<size_t>( Chimera::GPIO::Pull::NUM_OPTIONS )> PullMap = {};
  /* clang-format on */

}    // namespace Thor::LLD::FLASH

#endif /* TARGET_STM32L4 && THOR_LLD_FLASH */
