/********************************************************************************
 *  File Name:
 *    dma_intf.hpp
 *
 *  Description:
 *    STM32 Driver DMA Model
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_DMA_HPP
#define THOR_DRIVER_MODEL_DMA_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/dma>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/common/interrupts/dma_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/dma/dma_types.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Public Functions (Implemented by the driver)
  -------------------------------------------------------------------------------*/
  /**
   *  Initialize the driver
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Gets a shared pointer to the driver for a particular channel
   *
   *  @param[in] control        The channel to grab
   *  @return Driver_rPtr       Instance of the driver for the requested channel
   */
  Driver_rPtr getDriver( const Controller control );

  Stream_rPtr getStream( const Controller control, const Streamer stream );

  StreamMap *const streamView( RegisterMap *const periph, const size_t streamNum );

  /*-------------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  -------------------------------------------------------------------------------*/
  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );


  /*-------------------------------------------------------------------------------
  Driver Interface
  -------------------------------------------------------------------------------*/
  class IDriver
  {
  public:
    virtual ~IDriver() = default;

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
     */
    virtual Chimera::Status_t configure( StreamMap *const stream, StreamConfig *const config, TCB *const controlBlock ) = 0;

    /**
     *  Starts the transfer on the given stream
     *
     *  @param[in]  stream            The stream to act upon
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t start( StreamMap *const stream ) = 0;

    /**
     *  Stops the transfer on the given stream
     *
     *  @param[in]  stream            The stream to act upon
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t abort( StreamMap *const stream ) = 0;
  };


  class Driver
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attach( RegisterMap *const peripheral );
    Chimera::Status_t clockEnable();
    Chimera::Status_t clockDisable();
    Chimera::Status_t reset();
    Chimera::Status_t init();
    Chimera::Status_t configure( StreamMap *const stream, StreamConfig *const config, TCB *const controlBlock );
    Chimera::Status_t start( StreamMap *const stream );
    Chimera::Status_t abort( StreamMap *const stream );

  private:
    RegisterMap *periph;
  };


  /*-------------------------------------------------------------------------------
  Stream Interface
  -------------------------------------------------------------------------------*/
  /**
   *  Models the interface to a DMA controller stream
   */
  class IStream : virtual public Chimera::Event::ListenerInterface
  {
  public:
    virtual ~IStream() = default;

    virtual Chimera::Status_t attach( StreamMap *const peripheral, RegisterMap *const parent ) = 0;

    /**
     *  Reconfigures a stream for a new transfer
     *
     *  @param[in]  config            The stream's transfer configuration settings
     *  @param[in]  controlBlock      Control block for the transfer, describing behavior
     *  @return Chimera::Status_t
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

  /**
   *  Models a stream within a DMA controller peripheral (channel)
   */
  class Stream : public Chimera::Thread::Lockable<Stream>
  {
  public:
    Stream();
    ~Stream();

    Chimera::Status_t attach( StreamMap *const peripheral, RegisterMap *const parent );
    Chimera::Status_t configure( StreamConfig *const config, TCB *const controlBlock );
    Chimera::Status_t start();
    Chimera::Status_t abort();
    void enableTransferIRQ();
    void disableTransferIRQ();
    void enterCriticalSection();
    void exitCriticalSection();

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
    friend Chimera::Thread::Lockable<Stream>;

    StreamMap *stream;
    RegisterMap *parent;
    TCB controlBlock;
    size_t streamRegisterIndex;
    size_t streamResourceIndex;
    IRQn_Type streamIRQn;
  };

}    // namespace Thor::LLD::DMA

#endif /* !THOR_DRIVER_MODEL_DMA_HPP */
