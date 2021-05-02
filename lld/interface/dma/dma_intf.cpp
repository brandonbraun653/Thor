/********************************************************************************
 *  File Name:
 *    dma_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/


/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/dma>

#if defined( THOR_LLD_DMA )
namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_DMA1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( DMA1_PERIPH ) )
    {
      return DMA1_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_DMA2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( DMA2_PERIPH ) )
    {
      return DMA2_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }
}  // namespace Thor::LLD::DMA
#endif  /* THOR_LLD_DMA */
