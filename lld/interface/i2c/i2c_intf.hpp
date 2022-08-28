/******************************************************************************
 *  File Name:
 *    i2c_intf.hpp
 *
 *  Description:
 *    STM32 LLD I2C Interface Spec
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_I2C_DRIVER_INTERFACE_HPP
#define THOR_LLD_I2C_DRIVER_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/container>
#include <Chimera/common>
#include <Chimera/dma>
#include <Chimera/gpio>
#include <Chimera/i2c>
#include <Chimera/thread>
#include <Thor/lld/common/interrupts/i2c_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/i2c/i2c_detail.hpp>
#include <Thor/lld/interface/i2c/i2c_types.hpp>
#include <limits>


#if defined( THOR_I2C )
namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Public Functions (Implemented by the project)
  ---------------------------------------------------------------------------*/
  /**
   *  Initializes the low level driver in the integrating project
   *
   *  @return Chimera::Status_t
   */
  Chimera::Status_t init_prj_driver();

  /*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initialize the I2C interface layer
   *
   * @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   *  Gets a raw pointer to the I2C driver for a particular channel
   *
   *  @param[in]  channel     The I2C channel to get
   *  @return IDriver_rPtr    Instance of the I2C driver for the requested channel
   */
  Driver_rPtr getLLDriver( const Chimera::I2C::Channel channel );

  /**
   *  Checks if the given hardware channel is supported on this device.
   *
   *  @param[in]  port        The I2C port to grab
   *  @param[in]  pin         Which pin on the given port
   *  @return bool
   */
  bool isSupported( const Chimera::I2C::Channel channel );

  /**
   *  Looks up an index value that can be used to access distributed resources
   *  associated with a peripheral instance. If the address is invalid, this will
   *  return INVALID_RESOURCE_INDEX.
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   *  Looks up an index value that can be used to access distributed resources
   *  associated with a peripheral instance. If the port is invalid, this will
   *  return INVALID_RESOURCE_INDEX.
   *
   *  @param[in]  port          Which port to get the resource index for
   *  @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::I2C::Channel channel );

  /**
   *  Gets the I2C port associated with a peripheral address
   *
   *  @param[in]  address       Memory address the peripheral is mapped to
   *  @return Chimera::I2C::Channel
   */
  Chimera::I2C::Channel getChannel( const std::uintptr_t address );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class Driver : public Chimera::Thread::Lockable<Driver>
  {
  public:
    Driver();
    ~Driver();

    /*-------------------------------------------------------------------------
    Configuration
    -------------------------------------------------------------------------*/
    /**
     * @brief Attaches a peripheral instance to the interaction model
     *
     * @param[in]  peripheral    Memory mapped struct of the desired peripheral
     * @return Chimera::Status_t
     */
    Chimera::Status_t attach( RegisterMap *const peripheral );

    /**
     * @brief Enables the peripheral clock for the configured instance
     * @return void
     */
    void enableClock();

    /**
     * @brief Disables the peripheral clock for the configured instance
     * @return void
     */
    void disableClock();

    /**
     * @brief Turns on interrupts for the given signal type, if supported.
     *
     * @param[in]  signal        The ISR event to enable
     * @return void
     */
    void enableISRSignal( const InterruptBits signal );

    /**
     * @brief Turns off interrupts for the given signal type, if supported.
     *
     * @param[in]  signal        The ISR event to enable
     * @return void
     */
    void disableISRSignal( const InterruptBits signal );

    /**
     * @brief Configure the driver with the appropriate settings.
     *
     * These are more generic settings that apply across all implementations.
     *
     * @param[in]  cfg           The configuration info
     * @return Chimera::Status_t
     */
    Chimera::Status_t configure( const Chimera::I2C::DriverConfig &cfg );

    /*-------------------------------------------------------------------------
    Transmit and Receive Operations
    -------------------------------------------------------------------------*/
    /**
     * @brief Reads data off the bus
     *
     * @param address   Address to read from
     * @param data      Data buffer to read into
     * @param length    Number of bytes to read
     * @return Chimera::Status_t
     */
    Chimera::Status_t read( const uint16_t address, void *const data, const size_t length );

    /**
     * @brief Writes data onto the bus
     *
     * @param address   Address to write to
     * @param data      Data buffer to write from
     * @param length    Number of bytes to write
     * @return Chimera::Status_t
     */
    Chimera::Status_t write( const uint16_t address, const void *const data, const size_t length );

    /**
     * @brief Perform a full-duplex transfer on the bus
     *
     * @param address   Address to read/write from
     * @param tx_data   Data buffer to write from
     * @param rx_data   Data buffer to read into
     * @param length    Number of bytes to transfer
     * @return Chimera::Status_t
     */
    Chimera::Status_t transfer( const uint16_t address, const void *const tx_data, void *const rx_data, const size_t length );

    /**
     * @brief Gets the state of the last transfer
     *
     * @return TxfrCB
     */
    TxfrCB whatHappened();

    /*-------------------------------------------------------------------------
    ISR Protection Mechanisms
    -------------------------------------------------------------------------*/
    /**
     * Disables peripheral specific interrupts. This prevents the hardware from
     * listening to ISR events when those events would interfere with a complex
     * operation.
     *
     * These events will be pending in the NVIC hardware and fire as soon as
     * exitCriticalSection() is called.
     */
    void enterCriticalSection();

    /**
     * Re-enables peripheral specific interrupts. If an event was generated
     * while inside a critical context, it will fire immediately upon calling
     * this function.
     */
    void exitCriticalSection();

  protected:
    /**
     * @brief Perform a full-duplex transfer on the bus using interrupts
     *
     * @param address   Address to read/write from
     * @param tx_data   Data buffer to write from
     * @param rx_data   Data buffer to read into
     * @param length    Number of bytes to transfer
     * @return Chimera::Status_t
     */
    Chimera::Status_t transferIT( const uint16_t address, const void *const tx_data, void *const rx_data, const size_t length );

    /**
     * @brief Perform a full-duplex transfer on the bus using DMA
     *
     * @param address   Address to read/write from
     * @param tx_data   Data buffer to write from
     * @param rx_data   Data buffer to read into
     * @param length    Number of bytes to transfer
     * @return Chimera::Status_t
     */
    Chimera::Status_t transferDMA( const uint16_t address, const void *const tx_data, void *const rx_data,
                                   const size_t length );

    /**
     * @brief Handler for error events
     */
    void IRQErrorHandler();

    /**
     * @brief Handler for generic events
     */
    void IRQEventHandler();

    /**
     * @brief DMA callback when a transfer finishes or is aborted
     *
     * @param stats   Information about the transfer
     */
    void onDMAComplete( const Chimera::DMA::TransferStats &stats );

  private:
    friend Chimera::Thread::Lockable<Driver>;

#if defined( STM32_I2C1_PERIPH_AVAILABLE )
    friend void( ::I2C1_EV_IRQHandler )();
    friend void( ::I2C1_ER_IRQHandler )();
#endif
#if defined( STM32_I2C2_PERIPH_AVAILABLE )
    friend void( ::I2C2_EV_IRQHandler )();
    friend void( ::I2C2_ER_IRQHandler )();
#endif
#if defined( STM32_I2C3_PERIPH_AVAILABLE )
    friend void( ::I2C3_EV_IRQHandler )();
    friend void( ::I2C3_ER_IRQHandler )();
#endif

    /*-------------------------------------------------------------------------
    Driver Data
    -------------------------------------------------------------------------*/
    RegisterMap *mPeriph;                    /**< Memory mapped peripheral */
    uint32_t mResourceIndex;                 /**< Derived lookup table index for resource access */
    uint32_t mISRBits;                       /**< Masks which ISR signals the software will listen to */
    Chimera::DMA::RequestId mTXDMARequestId; /**< Request id of the TX DMA pipe for the driver */
    Chimera::DMA::RequestId mRXDMARequestId; /**< Request id of the RX DMA pipe for the driver */
    TxfrCB mTransfer;                        /**< Current transfer */
    TxfrCB mTransferCache;                   /**< Cache of last transfer */
  };
}    // namespace Thor::LLD::I2C

#endif /* THOR_LLD_I2C */
#endif /* !THOR_LLD_I2C_DRIVER_INTERFACE_HPP */
