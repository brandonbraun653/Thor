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
#include <Chimera/threading.hpp>
#include <Chimera/interface/dma_intf.hpp>
#include <Chimera/types/dma_types.hpp>

/* Thor Includes */
#include <Thor/definitions/dma_definitions.hpp>

namespace Thor::DMA
{
  class DMAClass : public Chimera::DMA::Interface, public Chimera::Threading::Lockable
  {
  public:
    ~DMAClass();

    /**
     *
     */
    static std::shared_ptr<DMAClass> get();

    Chimera::Status_t init() final override;

    Chimera::Status_t reset() final override;

    Chimera::Status_t start() final override;

    Chimera::Status_t configure( const Chimera::DMA::Init &config, const Chimera::DMA::TCB &transfer, const size_t timeout,
                                 Chimera::DMA::TransferHandle_t *const handle ) final override;

    Chimera::Status_t abort( Chimera::DMA::TransferHandle_t handle, const size_t timeout ) final override;

    Chimera::Status_t status( Chimera::DMA::TransferHandle_t handle, const size_t timeout ) final override;

    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout, size_t &registrationID ) final override;

    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout ) final override;

  private:
    DMAClass();

    size_t listenerIDCount;
    std::vector<Chimera::Event::Actionable> eventListeners;
  };
}    // namespace Thor::DMA

#endif /* !THOR_DMA_HPP */
