/******************************************************************************
 *  File Name:
 *    hw_sdio_data.cpp
 *
 *  Description:
 *    SDIO LLD data for the STM32F4 series chips
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/sdio>
#include <Thor/lld/interface/inc/sdio>
#include <Thor/lld/interface/inc/dma>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Configuration Maps
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  {
  }

  /*---------------------------------------------------------------------------
  Peripheral Resources
  ---------------------------------------------------------------------------*/
  namespace Resource
  {
    LLD_CONST Thor::LLD::DMA::Source RXDMASignals[ NUM_SDIO_PERIPHS ] = {
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SDIO,
#endif
    };

    LLD_CONST Thor::LLD::DMA::Source TXDMASignals[ NUM_SDIO_PERIPHS ] = {
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::SDIO,
#endif
    };

    LLD_CONST IRQn_Type IRQSignals[ NUM_SDIO_PERIPHS ] = {  
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
      SDIO_IRQn,
#endif
    };
  }    // namespace Resource
}    // namespace Thor::LLD::SDIO
