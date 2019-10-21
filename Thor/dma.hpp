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
#include <Thor/drivers/common/types/dma_types.hpp>

namespace Thor::DMA
{
  /**
   *  Initialize the DMA driver
   *  
   *  @return Chimera::Status_t 
   */
  Chimera::Status_t initialize();

  class DMAClass : public Chimera::DMA::HWInterface, public Chimera::Threading::Lockable
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

    /**
     *  Registers a listener to a specific DMA stream
     *
     *  @param[in]  stream            The stream to register the listener against
     *  @param[in]  listener          The listener to be registered
     *  @param[in]  timeout           How long to wait for the registration sink to become available
     *  @param[out] registrationID    Returned ID that uniquely identifies the registrated listener
     *
     *  @return Chimera::Status_t
     */
    Chimera::Status_t registerListener( Driver::DMA::StreamX *const stream, Chimera::Event::Actionable &listener,
                                        const size_t timeout, size_t &registrationID );

    /**
     *  Removes a previously registered listener on a specific DMA stream
     *
     *  @param[in]  stream            The stream to remove the listener from
     *  @param[in]  registrationID    ID returned when the listener was registered
     *  @param[in]  timeout           How long to wait for the registration sink to become available
     *
     *  @return Chimera::Status_t
     */
    Chimera::Status_t removeListener( Driver::DMA::StreamX *const stream, const size_t registrationID, const size_t timeout );

  private:
    DMAClass();

    Thor::Driver::DMA::StreamResources lastLookup;

    size_t listenerIDCount;
    std::vector<Chimera::Event::Actionable> eventListeners;
  };
}    // namespace Thor::DMA

#endif /* !THOR_DMA_HPP */
