/******************************************************************************
 *  File Name:
 *    hw_crs_register_stm32l432kc.cpp
 *
 *  Description:
 *    CRS register definitions for the STM32L432KC series chips.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/crs/hw_crs_prj.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_prv_data.hpp>
#include <Thor/lld/stm32l4x/crs/hw_crs_types.hpp>
#include <Thor/lld/stm32l4x/crs/variant/hw_crs_register_stm32l4xxxx.hpp>


namespace Thor::LLD::CRS
{
  static RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_CRS1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( CRS1_PERIPH ) )
    {
      return CRS1_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }

}    // namespace Thor::LLD::CRS
