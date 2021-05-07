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
   *  Gets a pointer to the driver for a particular dma controller
   *
   *  @param[in] control        The channel to grab
   *  @return Driver_rPtr       Instance of the driver for the requested channel
   */
  Driver_rPtr getDriver( const Controller control );

  /**
   * @brief Gets a pointer to the stream controller object
   *
   * @param control     Root controller instance id
   * @param stream      Stream controller instance id
   * @return Stream_rPtr
   */
  Stream_rPtr getStream( const Controller control, const Streamer stream );

  /**
   * @brief Gets the stream controller's memory mapped registers from the parent DMA instance
   *
   * @param periph      Parent DMA instance
   * @param streamNum   Which stream  to grab
   * @return StreamMap*
   */
  StreamMap *streamView( RegisterMap *const periph, const Streamer streamNum );


  /*-------------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  -------------------------------------------------------------------------------*/
  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return bool
   */
  bool isSupported( const Controller channel, const Streamer stream );

  /**
   *  Gets the resource index associated with a particular channel. If not
   *  supported, will return INVALID_RESOURCE_INDEX
   *
   *  @param[in]  channel       The channel number to be checked
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Controller channel, const Streamer stream );

  /**
   *  Looks up a resource index based on a raw peripheral instance
   *
   *  @param[in]  address       The peripheral address
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Gets the DMA controller associated with a peripheral address
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return Controller
   */
  Controller getController( const std::uintptr_t address );

  /**
   * @brief Gets the DMA stream associated with a peripheral address
   *
   * @param address             Memory address the peripheral is mapped to
   * @return Streamer
   */
  Streamer getStream( const std::uintptr_t address );

  /**
   *  Initializes the drivers by attaching the appropriate peripheral
   *
   *  @param[in]  driverList    List of driver objects to be initialized
   *  @param[in]  numDrivers    How many drivers are in driverList
   *  @return bool
   */
  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers );


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
     * @brief Initialize the DMA controller
     *
     * @return Chimera::Status_t
     */
    virtual Chimera::Status_t init() = 0;

    /**
     *  Enables the DMA peripheral clock
     *
     *  @return void
     */
    virtual void clockEnable() = 0;

    /**
     *  Disables the DMA peripheral clock
     *
     *  @return void
     */
    virtual void clockDisable() = 0;

    /**
     *  Completely resets the entire driver, including all instance resources.
     *
     *  @note init() must be called again before the driver can be reused
     *
     *  @return void
     */
    virtual void reset() = 0;
  };


  class Driver
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t attach( RegisterMap *const peripheral );
    Chimera::Status_t init();
    void clockEnable();
    void clockDisable();
    void reset();

  private:
    RegisterMap *mPeriph;
  };


  /*-------------------------------------------------------------------------------
  Stream Interface
  -------------------------------------------------------------------------------*/
  /**
   *  Models the interface to a DMA controller stream
   */
  class IStream
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

    /**
     * @brief Attaches a callback to execute on an ISR event
     *
     * This will run directly in the ISR
     *
     * @param irq           Signal to attach to
     * @param callback      Callback function
     * @return Chimera::Status_t
     */
    virtual Chimera::Status_t onIRQ( const Interrupt irq, ISRCallback &callback ) = 0;

    /**
     * @brief Enables interrupts for the configured stream
     */
    virtual void enableInterrupts() = 0;

    /**
     * @brief Disables interrupts for the configured stream
     */
    virtual void disableInterrupts() = 0;
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
    Chimera::Status_t onIRQ( const Interrupt irq, ISRCallback &callback );
    void enableInterrupts();
    void disableInterrupts();

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
     *  @param[in]  status      Status registers for the given interrupt
     *  @return void
     */
    void IRQHandler( const uint8_t channel, const uint8_t status );

  private:
    friend Chimera::Thread::Lockable<Stream>;

    StreamMap *mStream;    /**< Stream's memory mapped registers */
    RegisterMap *mPeriph;  /**< Core controller memory mapped registers */
    size_t mRegisterIndex; /**< Register offset for the stream */
    size_t mResourceIndex; /**< Resource index for the stream */
    IRQn_Type mIRQn;       /**< Stream's IRQ number */
    TCB mTCB;              /**< Control block for current transfer */

    /**
     * @brief Resets the LISR/HISR registers for the configured stream
     * Assumes interrupt safe context.
     */
    void reset_isr();
  };

}    // namespace Thor::LLD::DMA

#endif /* !THOR_DRIVER_MODEL_DMA_HPP */
