/********************************************************************************
 *  File Name:
 *    hld_dma_driver.hpp
 *
 *  Description:
 *    Thor implementation of the DMA driver
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_HPP
#define THOR_DMA_HPP

/* Chimera Includes */
#include <Chimera/thread>
#include <Chimera/dma>

/* Thor Includes */
#include <Thor/hld/dma/hld_dma_intf.hpp>

namespace Thor::DMA
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Chimera Interface Requirements
  -------------------------------------------------*/
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  Chimera::DMA::RequestId constructPipe( const Chimera::DMA::PipeConfig &config );
  Chimera::DMA::RequestId transfer( const Chimera::DMA::MemTransfer &transfer );
  Chimera::DMA::RequestId transfer( const Chimera::DMA::PipeTransfer &transfer );

  /*-------------------------------------------------
  Thor Interface (Additional)
  -------------------------------------------------*/
  /**
   * @brief Get the Pipe Config object mapped to the request ID
   *
   * @param id        Which ID to look up
   * @param output    Structure to load the config into
   * @return bool     True, the config exists. False, it doesn't exist.
   */
  bool getPipeConfig( const Chimera::DMA::RequestId id, Chimera::DMA::PipeConfig &output );

}    // namespace Thor::DMA

#endif /* !THOR_DMA_HPP */
