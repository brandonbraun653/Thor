/********************************************************************************
 *  File Name:
 *    dma_model.hpp
 *
 *  Description:
 *    STM32 Driver DMA Model
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_DMA_HPP
#define THOR_DRIVER_MODEL_DMA_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/event>
#include <Chimera/thread>
#include <Chimera/common>
#include <Chimera/event>
#include <Chimera/dma>

/* Thor Includes */
#include <Thor/lld/interface/dma/dma_types.hpp>

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Initialize the driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /**
   * Models the interface to a DMA controller peripheral
   */
  class IPeripheral
  {
  public:
    virtual ~IPeripheral() = default;

    /**
     *  Attaches an instance of a DMA peripheral for the class to control
     *
     *  @param[in]  peripheral        The peripheral to be attached to the driver
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t attach( RegisterMap *const peripheral ) = 0;

    /**
     *  Enables the DMA peripheral clock
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t clockEnable() = 0;

    /**
     *  Disables the DMA peripheral clock
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t clockDisable() = 0;

    /**
     *  Completely resets the entire driver, including all instance resources.
     *
     *  @note init() must be called again before the driver can be reused
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset() = 0;

    /**
     *  Performs low level driver initialization functionality
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t init() = 0;

    /**
     *  Reconfigures a stream for a new transfer
     *
     *  @param[in]  config            The stream's transfer configuration settings
     *  @return Chimera::Status_t
     *
     *  |  Return Value |               Explanation               |
     *  |:-------------:|:---------------------------------------:|
     *  |            OK | The stream was configured               |
     *  |        LOCKED | The stream is busy                      |
     *  | NOT_SUPPORTED | A configuration option wasn't supported |
     */
    virtual Chimera::Status_t configure( StreamX *const stream, StreamConfig *const config, TCB *const controlBlock ) = 0;

    /**
     *  Starts the transfer on the given stream
     *
     *  @param[in]  stream            The stream to act upon
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t start( StreamX *const stream ) = 0;

    /**
     *  Stops the transfer on the given stream
     *
     *  @param[in]  stream            The stream to act upon
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t abort( StreamX *const stream ) = 0;

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
    virtual Chimera::Status_t registerListener( StreamX *const stream, Chimera::Event::Actionable &listener,
                                                const size_t timeout, size_t &registrationID ) = 0;

    /**
     *  Removes a previously registered listener on a specific DMA stream
     *
     *  @param[in]  stream            The stream to remove the listener from
     *  @param[in]  registrationID    ID returned when the listener was registered
     *  @param[in]  timeout           How long to wait for the registration sink to become available
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t removeListener( StreamX *const stream, const size_t registrationID, const size_t timeout ) = 0;
  };

  /**
   *  Models the interface to a DMA controller stream
   */
  class IStream : virtual public Chimera::Event::ListenerInterface
  {
  public:
    virtual ~IStream() = default;

    virtual Chimera::Status_t attach( StreamX *const peripheral, RegisterMap *const parent ) = 0;

    /**
     *  Attaches a semaphore that will be given to in the ISR handler
     *  when any event completes. This could be transfer complete, transfer
     *  half complete, etc.
     *
     *  @param[in]  wakeup            Signal to be given to upon ISR events
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup ) = 0;

    /**
     *  Reconfigures a stream for a new transfer
     *
     *  @param[in]  config            The stream's transfer configuration settings
     *  @param[in]  controlBlcok      Control block for the transfer, describing behavior
     *  @return Chimera::Status_t
     *
     *  |  Return Value |               Explanation               |
     *  |:-------------:|:---------------------------------------:|
     *  |            OK | The stream was configured               |
     *  |        LOCKED | The stream is busy                      |
     *  | NOT_SUPPORTED | A configuration option wasn't supported |
     */
    virtual Chimera::Status_t configure( StreamConfig *const config, TCB *const controlBlock ) = 0;

    /**
     *  Starts the transfer on this stream
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t start() = 0;

    /**
     *  Stops the transfer on this stream
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t abort() = 0;
  };

}    // namespace Thor::LLD::DMA

#endif /* !THOR_DRIVER_MODEL_DMA_HPP */
