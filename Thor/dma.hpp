/********************************************************************************
 *  File Name:
 *    dma.hpp
 *
 *  Description:
 *    Thor implementation of the DMA driver
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_HPP
#define THOR_DMA_HPP

/* Chimera Includes */
#include <Chimera/interface/dma_intf.hpp>
#include <Chimera/types/dma_types.hpp>

/* Thor Includes */
#include <Thor/definitions/dma_definitions.hpp>

namespace Thor::DMA
{
  class DMAClass : public Chimera::DMA::Interface
  {
  public:
    DMAClass();
    ~DMAClass();

    Chimera::Status_t init( const Chimera::DMA::Init &config, const size_t timeout,
                            Chimera::DMA::TransferHandle_t handle ) final override;

    Chimera::Status_t start( Chimera::DMA::TransferHandle_t handle, const Chimera::DMA::TCB &transfer,
                             const size_t timeout ) final override;

    Chimera::Status_t abort( Chimera::DMA::TransferHandle_t handle, const size_t timeout ) final override;

    Chimera::Status_t status( Chimera::DMA::TransferHandle_t handle, const size_t timeout ) final override;
  };
}    // namespace Thor::DMA

#endif /* !THOR_DMA_HPP */
