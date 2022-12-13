/******************************************************************************
 *  File Name:
 *    hld_dma_driver.hpp
 *
 *  Description:
 *    Thor implementation of the DMA driver
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_DMA_HPP
#define THOR_DMA_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/dma>

namespace Thor::DMA
{
  /*-----------------------------------------------------------------------------
  Public Functions
  -----------------------------------------------------------------------------*/
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
