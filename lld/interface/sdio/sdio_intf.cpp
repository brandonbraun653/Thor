/******************************************************************************
 *  File Name:
 *    sdio_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/sdio>

#if defined( THOR_SDIO )
namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( SDIO1_PERIPH ) )
    {
      return SDIO1_RESOURCE_INDEX;
    }
#endif
    
    return INVALID_RESOURCE_INDEX;
  }
}  // namespace Thor::LLD::SDIO
#endif  /* THOR_SDIO */
