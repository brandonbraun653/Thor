/********************************************************************************
 *  File Name:
 *    hw_dma_driver.hpp
 *
 *  Description:
 *    STM32 Driver for the DMA Peripheral
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_DRIVER_HPP
#define THOR_HW_DMA_DRIVER_HPP

/* Chimera Includes */
#include <Chimera/event>
#include <Chimera/thread>
#include <Chimera/dma>

/* Driver Includes */
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/common/interrupts/dma_interrupt_vectors.hpp>
#include <Thor/lld/interface/dma/dma_intf.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_types.hpp>
#include <Thor/lld/stm32f4x/interrupt/hw_it_prj.hpp>

namespace Thor::LLD::DMA
{
  class StreamController;

  /**
   *  Checks if a stream is on a given DMA controller
   *
   *  @param[in]  controller    The DMA instance to check against
   *  @param[in]  stream        The stream to check if on the controller
   *  @return bool
   */
  bool streamIsOnController( RegisterMap *const controller, StreamX *const stream );

  /**
   *  Gets the current stream controller instance via lookup
   *
   *  @param[in]  resourceIndex   LLD defined resource lookup index for a stream
   *  @return StreamController *  The instance of the controller
   */
  StreamController * getStreamController( const uint8_t resourceIndex );

  /**
   *  Models a stream within a DMA controller peripheral (channel)
   */
  class StreamController : virtual public IStream, public Chimera::Thread::Lockable<StreamController>
  {
  public:
    StreamController();
    ~StreamController();

    Chimera::Status_t attach( StreamX *const peripheral, RegisterMap *const parent ) final override;

    Chimera::Status_t attachISRWakeup( Chimera::Thread::BinarySemaphore *const wakeup ) final override;

    Chimera::Status_t configure( StreamConfig *const config, TCB *const controlBlock ) final override;

    Chimera::Status_t start() final override;

    Chimera::Status_t abort() final override;

    Chimera::Status_t registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                        size_t &registrationID ) final override;

    Chimera::Status_t removeListener( const size_t registrationID, const size_t timeout ) final override;

    /**
     *  Non-ISR post processing of an interrupt event. Allows the user to have more
     *  complex functionality that executes in response to an ISR signal at the cost
     *  of a little bit of latency.
     *
     *  @return Chimera::Status_t
     */
    Chimera::Status_t postISRProcessing();

  protected:
    friend void( ::DMA1_Stream0_IRQHandler )();
    friend void( ::DMA1_Stream1_IRQHandler )();
    friend void( ::DMA1_Stream2_IRQHandler )();
    friend void( ::DMA1_Stream3_IRQHandler )();
    friend void( ::DMA1_Stream4_IRQHandler )();
    friend void( ::DMA1_Stream5_IRQHandler )();
    friend void( ::DMA1_Stream6_IRQHandler )();
    friend void( ::DMA1_Stream7_IRQHandler )();
    friend void( ::DMA2_Stream0_IRQHandler )();
    friend void( ::DMA2_Stream1_IRQHandler )();
    friend void( ::DMA2_Stream2_IRQHandler )();
    friend void( ::DMA2_Stream3_IRQHandler )();
    friend void( ::DMA2_Stream4_IRQHandler )();
    friend void( ::DMA2_Stream5_IRQHandler )();
    friend void( ::DMA2_Stream6_IRQHandler )();
    friend void( ::DMA2_Stream7_IRQHandler )();

    /**
     *  Stream interrupt request handler
     *
     *  @param[in]  channel     The channel on the stream that generated the interrupt.
     *  @param[in]  request     The request generator peripheral ID
     *  @return void
     */
    void IRQHandler( const uint8_t channel );

  private:
    friend Chimera::Thread::Lockable<StreamController>;


    StreamX *stream;
    RegisterMap *parent;
    TCB controlBlock;
    size_t streamRegisterIndex;
    size_t streamResourceIndex;
    Chimera::Thread::BinarySemaphore *wakeupSignal;
    IRQn_Type streamIRQn;

    size_t listenerIDCount;
    Chimera::Event::ActionableList eventListeners;

    void enableTransferIRQ();
    void disableTransferIRQ();
    void enterCriticalSection();
    void exitCriticalSection();

    void processListeners( const Chimera::Event::Trigger event );
  };

  /**
   *  Models the interface to a full DMA controller, which is composed of many streams.
   *  For the STM32F4xxx chips, there is typically seven streams per channel.
   */
  class ChannelController : virtual public IPeripheral
  {
  public:
    ChannelController();
    ~ChannelController();

    Chimera::Status_t attach( RegisterMap *const peripheral ) final override;

    Chimera::Status_t clockEnable() final override;

    Chimera::Status_t clockDisable() final override;

    Chimera::Status_t reset() final override;

    Chimera::Status_t init() final override;

    Chimera::Status_t configure( StreamX *const stream, StreamConfig *const config, TCB *const controlBlock ) final override;

    Chimera::Status_t start( StreamX *const stream ) final override;

    Chimera::Status_t abort( StreamX *const stream ) final override;

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
    Chimera::Status_t registerListener( StreamX *const stream, Chimera::Event::Actionable &listener, const size_t timeout,
                                        size_t &registrationID );

    /**
     *  Removes a previously registered listener on a specific DMA stream
     *
     *  @param[in]  stream            The stream to remove the listener from
     *  @param[in]  registrationID    ID returned when the listener was registered
     *  @param[in]  timeout           How long to wait for the registration sink to become available
     *
     *  @return Chimera::Status_t
     */
    Chimera::Status_t removeListener( StreamX *const stream, const size_t registrationID, const size_t timeout );

  private:
    RegisterMap *periph; /**< Memory mapped struct to the DMA controller peripheral */
  };

}    // namespace Thor::LLD::DMA

#endif /* !THOR_HW_DMA_DRIVER_HPP */
