/******************************************************************************
 *  File Name:
 *    sdio_prv_data.hpp
 *
 *  Description:
 *    Declaration of data the must either be defined by the LLD implementation
 *    or is shared among all possible drivers.
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_SDIO_DATA_HPP
#define THOR_LLD_SDIO_DATA_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/cfg>
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/interface/dma/dma_types.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>
#include <Thor/lld/interface/sdio/sdio_detail.hpp>
#include <Thor/lld/interface/sdio/sdio_types.hpp>

#if defined( THOR_SDIO )
namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Peripheral Instances
  ---------------------------------------------------------------------------*/
#if defined( STM32_SDIO1_PERIPH_AVAILABLE )
  extern RegisterMap *SDIO1_PERIPH;
#endif

  /*---------------------------------------------------------------------------
  Configuration Maps
  ---------------------------------------------------------------------------*/
  namespace ConfigMap
  {

  }    // namespace ConfigMap

  /*---------------------------------------------------------------------------
  Peripheral Resources
  ---------------------------------------------------------------------------*/
  namespace Resource
  {
    extern LLD_CONST Thor::LLD::DMA::Source RXDMASignals[ NUM_SDIO_PERIPHS ];
    extern LLD_CONST Thor::LLD::DMA::Source TXDMASignals[ NUM_SDIO_PERIPHS ];
    extern LLD_CONST IRQn_Type              IRQSignals[ NUM_SDIO_PERIPHS ];
  }    // namespace Resource
}    // namespace Thor::LLD::SDIO
#endif /* THOR_SDIO */

#endif /* !THOR_LLD_SDIO_DATA_HPP */
