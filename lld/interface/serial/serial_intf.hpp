/********************************************************************************
 *  File Name:
 *    serial_intf.hpp
 *
 *  Description:
 *    STM32 Driver Model for Serial Communication
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_MODEL_SERIAL_HPP
#define THOR_DRIVER_MODEL_SERIAL_HPP

/* C++ Includes */
#include <cstdint>
#include <cstdlib>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/serial>

/* Driver Includes */
#include <Thor/lld/interface/serial/serial_types.hpp>


namespace Thor::LLD::Serial
{
  /**
   *  Describes the bare minimum functionality that should be present across
   *  all STM32 families for a UART/USART driver.
   */
  class Basic
  {
  public:
    virtual ~Basic() = default;

    /**
     *  Performs the low level driver register initialization to
     *  get the peripheral into a basic functional state for blocking transfers.
     *
     *  @param[in]  cfg     Hardware configuration options
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t init( const Thor::LLD::Serial::Config &cfg ) = 0;

    /**
     *  Deinitializes the low level driver and peripheral
     *
     *  @note init() must be called again before the driver can be reused
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t deinit() = 0;

    /**
     *  Completely resets the entire driver, including all instance resources.
     *
     *  @note init() must be called again before the driver can be reused
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t reset() = 0;

    /**
     *  Transmit data using blocking mode. The function will not return in the given
     *  thread of execution until the transfer is complete.
     *
     *  @warning Not intended for production designs due to its blocking nature.
     *
     *  @param[in]  data      The buffer to transmit from
     *  @param[in]  size      How much data to transfer in bytes
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t transmit( const void *const data, const size_t size ) = 0;

    /**
     *  Receive data using blocking mode. The function will not return in the given
     *  thread of execution until the transfer is complete.
     *
     *  @warning Not intended for production designs due to its blocking nature.
     *
     *  @param[in]  data      The buffer to read into
     *  @param[in]  size      How much data to receive in bytes
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t receive( void *const data, const size_t size ) = 0;

    /**
     *  Gets the current state of the TX transfer
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t txTransferStatus() = 0;

    /**
     *  Gets the current state of the RX transfer
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t rxTransferStatus() = 0;

    /**
     *  Gets some signaling flags that describe specific behaviors in the serial driver
     *
     *  @return uint32_t
     */
    virtual uint32_t getFlags() = 0;

    /**
     *  Clears specific bits in the signal flags
     *
     *  @param[in]  flagbits    Bits that are set here will be cleared internally
     *  @return void
     */
    virtual void clearFlags( const uint32_t flagBits ) = 0;
  };

  /**
   *  Extended fucntionality that allows for interrupt and DMA based transfers.
   *  Most chips have this functionality as well, though some may not.
   */
  class Extended
  {
  public:
    virtual ~Extended() = default;

    /**
     *  Enable the interrupts needed for a particular peripheral to
     *  perform a transfer in interrupt mode.
     *
     *  @param[in]  periph    The peripheral to enable interrupts for
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableIT( const Chimera::Hardware::SubPeripheral periph ) = 0;

    /**
     *  Disable the interrupts needed for a particular peripheral to
     *  perform a transfer in interrupt mode.
     *
     *  @param[in]  periph    The peripheral to disable interrupts for
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disableIT( const Chimera::Hardware::SubPeripheral periph ) = 0;

    /**
     *  Transmit data using interrupt mode. Assumes that the memory pointed to
     *  by 'data' will exist for as long as the transfer takes to execute.
     *
     *  @param[in]  data      The data to be transmitted
     *  @param[in]  size      How long the data is in bytes
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t transmitIT( const void *const data, const size_t size ) = 0;

    /**
     *  Receive data using interrupt mode. If none arrives immediately, the driver should
     *  listen indefinitely using a line idle interrupt or something similar. Assumes that
     *  the memory pointed to by 'data' will exist for as long as the transfer takes to execute.
     *
     *  @param[in]  data      The data to be transmitted
     *  @param[in]  size      How long the data is in bytes
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t receiveIT( void *const data, const size_t size ) = 0;

    /**
     *  Initializes the necessary DMA hardware
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t initDMA() = 0;

    /**
     *  Deinitializes the DMA hardware only used for this peripheral, if any exists. It absolutely
     *  is not allowed to touch other configuration settings for fear of breaking other modules.
     *
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t deinitDMA() = 0;

    /**
     *  Enable the interrupts needed for a particular peripheral to
     *  perform a transfer in DMA mode.
     *
     *  @param[in]  periph    The peripheral to enable interrupts for
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t enableDMA_IT( const Chimera::Hardware::SubPeripheral periph ) = 0;

    /**
     *  Disable the interrupts needed for a particular peripheral to
     *  perform a transfer in DMA mode.
     *
     *  @param[in]  periph    The peripheral to disable interrupts for
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t disableDMA_IT( const Chimera::Hardware::SubPeripheral periph ) = 0;

    /**
     *  Transmit data using DMA mode. Assumes that the memory pointed to
     *  by 'data' will exist for as long as the transfer takes to execute.
     *
     *  @param[in]  data      The data to be transmitted
     *  @param[in]  size      How long the data is in bytes
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t transmitDMA( const void *const data, const size_t size ) = 0;

    /**
     *  Receive data using interrupt mode. If none arrives immediately, the driver should
     *  listen indefinitely using a line idle interrupt or something similar. Assumes that
     *  the memory pointed to by 'data' will exist for as long as the transfer takes to execute.
     *
     *  @param[in]  data      The data to be transmitted
     *  @param[in]  size      How long the data is in bytes
     *  @return Chimera::Status_t
     */
    virtual Chimera::Status_t receiveDMA( void *const data, const size_t size ) = 0;

    /**
     *  Stops an ongoing transmission, if in progress
     *
     *  @return void
     */
    virtual void killTransmit() = 0;

    /**
     *  Stops an ongoing reception, if in progress. Any associated data stored with the
     *  reception is assumed incomplete and will be lost.
     *
     *  @return void
     */
    virtual void killReceive() = 0;


    virtual void attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup ) = 0;
    virtual CDTCB getTCB_TX() = 0;
    virtual MDTCB getTCB_RX() = 0;
    virtual Config getConfiguration() = 0;
  };

}    // namespace Thor::LLD::Serial


#endif /* !THOR_DRIVER_MODEL_SERIAL_HPP */