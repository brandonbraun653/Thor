/********************************************************************************
 * File Name:
 *   can_intf.hpp
 *
 * Description:
 *   STM32 LLD CAN Interface Spec
 *
 * 2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_CAN_DRIVER_INTERFACE_HPP
#define THOR_LLD_CAN_DRIVER_INTERFACE_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Aurora/container>
#include <Chimera/can>
#include <Chimera/common>
#include <Chimera/gpio>
#include <Thor/lld/common/interrupts/can_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/can/can_detail.hpp>
#include <Thor/lld/interface/can/can_types.hpp>
#include <limits>

#if defined( THOR_CAN )
namespace Thor::LLD::CAN
{
  /*---------------------------------------------------------------------------
  Public Functions (Implemented by the project or common driver)
  ---------------------------------------------------------------------------*/
  /**
   * @brief Initializes the low level driver
   * @return Chimera::Status_t
   */
  Chimera::Status_t initialize();

  /**
   * @brief Gets a raw pointer to the CAN driver for a particular channel
   *
   * @param channel   The CAN channel to get
   * @return IDriver_rPtr    Instance of the CAN driver for the requested channel
   */
  Driver_rPtr getDriver( const Chimera::CAN::Channel channel );

  /*---------------------------------------------------------------------------
  Public Functions (Implemented at the interface layer)
  ---------------------------------------------------------------------------*/
  /**
   * @brief Checks if the given hardware channel is supported on this device.
   *
   * @param port  The CAN port to grab
   * @param pin   Which pin on the given port
   * @return bool
   */
  bool isSupported( const Chimera::CAN::Channel channel );

  /**
   * @brief Converts a hardware address into a resource index
   *
   * Looks up an index value that can be used to access distributed resources
   * associated with a peripheral instance. If the address is invalid, this will
   * return INVALID_RESOURCE_INDEX.
   *
   * @param address   Memory address the peripheral is mapped to
   * @return RIndex_t
   */
  RIndex_t getResourceIndex( const std::uintptr_t address );

  /**
   * @brief Converts a CAN channel into a resource index
   *
   * Looks up an index value that can be used to access distributed resources
   * associated with a peripheral instance. If the port is invalid, this will
   * return INVALID_RESOURCE_INDEX.
   *
   * @param port  Which port to get the resource index for
   * @return RIndex_t
   */
  RIndex_t getResourceIndex( const Chimera::CAN::Channel channel );

  /**
   * @brief Gets the CAN port associated with a peripheral address
   *
   * @param address   Memory address the peripheral is mapped to
   * @return Chimera::CAN::Channel
   */
  Chimera::CAN::Channel getChannel( const std::uintptr_t address );

  /**
   * @brief Initializes the CAN drivers by attaching the appropriate peripheral
   *
   * @param driverList  List of driver objects to be initialized
   * @param numDrivers  How many drivers are in driverList
   * @return bool
   */
  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers );

  /**
   * @brief Takes the given filter list and organizes them by size, from largest to smallest.
   *
   * Size is determined by the number of bytes each filter takes up in a bank. Generally this
   * is configurable from 2-8 bytes.
   *
   * @param filterList  List of configured filters to be sorted
   * @param listSize    Number of filters the list can hold. All lists must match this length.
   * @param indexList   On successful sort, will contain indices of the sorted filterList
   * @return bool       True if the list was filtered, false if there was some problem
   */
  bool sortFiltersBySize( const MessageFilter *const filterList, const uint8_t listSize, uint8_t *const indexList );

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  class Driver
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
     * @param peripheral  Memory mapped struct of the desired CAN peripheral
     * @return void
     */
    void attach( RegisterMap *const peripheral );

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
     * @brief Configure the CAN bus driver with the appropriate settings.
     *
     * These are more generic settings that apply across all CAN implementations.
     *
     * @param cfg   The configuration info
     * @return Chimera::Status_t
     */
    Chimera::Status_t configure( const Chimera::CAN::DriverConfig &cfg );

    /**
     * @brief Applies the given filter list onto the hardware filter banks.
     *
     * The list objects will be updated to include its assigned match index
     * (FMI), assuming the filter was successfully configured. This operation
     * will destroy any previous filter configuration and replace it with the
     * one given.
     *
     * @param filterList  List of filters that should be configured
     * @param filterSize  How many filters are in the list
     * @return Chimera::Status_t
     */
    Chimera::Status_t applyFilters( MessageFilter *const filterList, const size_t filterSize );

    /**
     * @brief Turns on interrupts for the given signal type, if supported.
     *
     * @param signal  The ISR event to enable
     * @return Chimera::Status_t
     */
    Chimera::Status_t enableISRSignal( const Chimera::CAN::InterruptType signal );

    /**
     * @brief Turns off interrupts for the given signal type, if supported.
     *
     * @param signal  The ISR event to enable
     * @return void
     */
    void disableISRSignal( const Chimera::CAN::InterruptType signal );

    /**
     * @brief Places the hardware into a debug mode for testing purposes
     *
     * @param mode  Which mode to be placed into
     * @return void
     */
    void enterDebugMode( const Chimera::CAN::DebugMode mode );

    /**
     * @brief Leaves any debug mode and returns to normal mode
     * @return void
     */
    void exitDebugMode();

    /*-------------------------------------------------------------------------
    Control
    -------------------------------------------------------------------------*/
    /**
     * @brief Flushes the TX buffer
     * @return void
     */
    void flushTX();

    /**
     * @brief Flushes the RX buffer
     * @return void
     */
    void flushRX();

    /*-------------------------------------------------------------------------
    Transmit & Receive Operations
    -------------------------------------------------------------------------*/
    /**
     * @brief Places the given frame into the TX circular buffer for transmission.
     *
     * @param frame   The frame to be transmitted
     * @return Chimera::Status_t
     */
    Chimera::Status_t send( const Chimera::CAN::BasicFrame &frame );

    /**
     * @brief Reads a frame off the internal RX circular buffer.
     *
     * This buffer is populated by the ISR when new data arrives.
     *
     * @param frame   The frame to place the received message into
     * @return Chimera::Status_t
     */
    Chimera::Status_t receive( Chimera::CAN::BasicFrame &frame );

    /**
     * @brief Checks how many frames are available for reception
     * @return size_t
     */
    size_t available();

    /*-------------------------------------------------------------------------
    Asynchronous Operations
    -------------------------------------------------------------------------*/
    /**
     * @brief Gets any information that was posted in relation to the last ISR event.
     *
     * This is limited to read-only because of the single producer many consumer
     * data model. The ISR produces, you consume. Copy out the data inside of a
     * critical section to guarantee the ISR won't modify the structure while reading.
     *
     * @param isr   The ISR event type to get the context for
     * @return const ISREventContext *const
     */
    const ISREventContext *const getISRContext( const Chimera::CAN::InterruptType isr );

    /**
     * Callback for the HLD to inform the LLD that the associated ISR event
     * has been properly handled and it's ok to re-enable the signal.
     *
     * @param isr   The ISR signal to indicate was handled
     * @return void
     */
    void setISRHandled( const Chimera::CAN::InterruptType isr );

    /*-------------------------------------------------------------------------
    ISR Protection Mechanisms
    -------------------------------------------------------------------------*/
    /**
     * @brief Disables peripheral specific CAN interrupts.
     *
     * This prevents the hardware from generating events when those events would
     * interfere with a complex operation. These events will be pending in the
     * NVIC hardware and fire as soon as exitCriticalSection() is called.
     *
     * @return void
     */
    void enterCriticalSection();

    /**
     * @brief Re-enables peripheral specific CAN interrupts.
     *
     * If an event was generated while inside a critical context, it will fire
     * immediately upon calling this function.
     *
     * @return void
     */
    void exitCriticalSection();

  protected:
    void CAN_TX_IRQHandler();
    void CAN_FIFO0_IRQHandler();
    void CAN_FIFO1_IRQHandler();
    void CAN_ERR_STS_CHG_IRQHandler();

  private:
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    friend void( ::CAN1_TX_IRQHandler )();
    friend void( ::CAN1_RX0_IRQHandler )();
    friend void( ::CAN1_RX1_IRQHandler )();
    friend void( ::CAN1_SCE_IRQHandler )();
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
    friend void( ::CAN2_TX_IRQHandler )();
    friend void( ::CAN2_RX0_IRQHandler )();
    friend void( ::CAN2_RX1_IRQHandler )();
    friend void( ::CAN2_SCE_IRQHandler )();
#endif

    /*-------------------------------------------------
    Peripheral descriptive information
    -------------------------------------------------*/
    RegisterMap *mPeriph;
    size_t       mResourceIndex;

    /*-------------------------------------------------
    Transmit/Receive Buffers
    -------------------------------------------------*/
    Aurora::Container::CircularBuffer<Chimera::CAN::BasicFrame> mTXBuffer;
    Aurora::Container::CircularBuffer<Chimera::CAN::BasicFrame> mRXBuffer;

    /*-------------------------------------------------
    ISR signaling and context buffers
    -------------------------------------------------*/
    ISREventContext mISREventContext[ NUM_CAN_IRQ_HANDLERS ];
  };
}    // namespace Thor::LLD::CAN

#endif /* THOR_LLD_CAN */
#endif /* !THOR_LLD_CAN_DRIVER_INTERFACE_HPP */
