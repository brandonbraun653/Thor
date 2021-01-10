/********************************************************************************
 *  File Name:
 *    hld_dma_driver.hpp
 *
 *  Description:
 *    Thor implementation of the DMA driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_HPP
#define THOR_DMA_HPP

/* Chimera Includes */
#include <Chimera/thread>
#include <Chimera/dma>

/* Thor Includes */
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/hld/dma/hld_dma_types.hpp>

namespace Thor::DMA
{
  Chimera::Status_t initialize();

#if 0
  class DMAClass : public Chimera::Threading::LockableCRTP<DMAClass>
  {
  public:
    ~DMAClass();

    static std::shared_ptr<DMAClass> get();

    Chimera::Status_t init() final override;

    Chimera::Status_t reset() final override;

    Chimera::Status_t start() final override;

    Chimera::Status_t configure( const Chimera::DMA::Init &config, const Chimera::DMA::TCB &transfer, const size_t timeout,
                                 Chimera::DMA::TransferHandle_t *const handle ) final override;

    Chimera::Status_t abort( Chimera::DMA::TransferHandle_t handle, const size_t timeout ) final override;

    Chimera::Status_t status( Chimera::DMA::TransferHandle_t handle, const size_t timeout ) final override;

    /**
     *  Registers a listener to a specific DMA stream
     *
     *  @param[in]  stream            The stream to register the listener against (zero indexed)
     *  @param[in]  listener          The listener to be registered
     *  @param[in]  timeout           How long to wait for the registration sink to become available
     *  @param[out] registrationID    Returned ID that uniquely identifies the registrated listener
     *
     *  @return Chimera::Status_t
     */
    Chimera::Status_t registerListener( const size_t stream, Chimera::Event::Actionable &listener,
                                        const size_t timeout, size_t &registrationID );

    /**
     *  Removes a previously registered listener on a specific DMA stream
     *
     *  @param[in]  stream            The stream to remove the listener from (zero indexed)
     *  @param[in]  registrationID    ID returned when the listener was registered
     *  @param[in]  timeout           How long to wait for the registration sink to become available
     *
     *  @return Chimera::Status_t
     */
    Chimera::Status_t removeListener( const size_t stream, const size_t registrationID, const size_t timeout );

  private:
    friend Chimera::Threading::LockableCRTP<DMAClass>;
    Chimera::Threading::RecursiveTimedMutex mClsMutex;

    DMAClass();

    size_t listenerIDCount;
    std::vector<Chimera::Event::Actionable> eventListeners;
  };

#endif

}    // namespace Thor::DMA

#endif /* !THOR_DMA_HPP */
