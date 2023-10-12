/******************************************************************************
 *  File Name:
 *    sdio_driver.hpp
 *
 *  Description:
 *    Driver interface for the SDIO peripheral
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_INTF_SDIO_DRIVER_HPP
#define THOR_LLD_INTF_SDIO_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/dma>
#include <Chimera/sdio>
#include <Chimera/thread>
#include <Thor/lld/common/interrupts/sdio_interrupt_vectors.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/sdio/sdio_detail.hpp>
#include <Thor/lld/interface/sdio/sdio_types.hpp>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Register level driver for the SDIO peripheral
   *
   * Generally speaking, the driver focuses on SD Memory Card support since
   * that is the most common use case.
   */
  class Driver : public Chimera::Thread::AsyncIO<Driver>
  {
  public:
    Driver();
    ~Driver();

    /*-----------------------------------------------------------------------------
    Peripheral Control Functions
    -----------------------------------------------------------------------------*/
    /**
     * @brief Attaches a peripheral for the driver to control
     *
     * @param peripheral  Pointer to the peripheral register block
     * @return Chimera::Status_t
     */
    Chimera::Status_t attach( RegisterMap *const peripheral );

    /**
     * @brief Resets the peripheral to a known state
     * @return Chimera::Status_t
     */
    Chimera::Status_t reset();

    /**
     * @brief Enable the peripheral clock
     * @return void
     */
    void clockEnable();

    /**
     * @brief Disable the peripheral clock
     * @return void
     */
    void clockDisable();

    /**
     * @brief Enables the IO bus clock
     * @return void
     */
    void busClockEnable();

    /**
     * @brief Disables the IO bus clock
     * @return void
     */
    void busClockDisable();

    /**
     * @brief Disables the peripheral core ISR
     * @return void
     */
    void enterCriticalSection();

    /**
     * @brief Enables the peripheral core ISR
     * @return void
     */
    void exitCriticalSection();

    /**
     * @brief Gets the current bus frequency for the given channel
     * @return uint32_t Frequency in Hz
     */
    uint32_t getBusFrequency();

    /**
     * @brief Sets the bus clock to a desired frequency
     *
     * @param freq    Desired frequency in Hz
     * @return ErrorType
     */
    ErrorType setBusFrequency( const uint32_t freq );

    /**
     * @brief Initializes the peripheral for SD card operation
     * @return Chimera::Status_t
     */
    Chimera::Status_t init();

    /**
     * @brief Deinitializes the peripheral and returns it to a reset state
     * @return Chimera::Status_t
     */
    Chimera::Status_t deinit();

    /**
     * @brief Transition the peripheral to the power on state
     * @return Chimera::Status_t
     */
    Chimera::Status_t setPowerStateOn();

    /**
     * @brief Transition the peripheral to the power off state
     * @return Chimera::Status_t
     */
    Chimera::Status_t setPowerStateOff();

    /**
     * @brief Sets the read wait mode for the peripheral
     *
     * @param mode
     * @return Chimera::Status_t
     */
    Chimera::Status_t setSDMMCReadWaitMode( uint32_t mode );

    /**
     * @brief Gets the command response register
     * @return uint8_t
     */
    uint8_t cpsmGetCmdResponse();

    /**
     * @brief Gets the response for a given response register
     *
     * @param which       Which response register to read
     * @return uint32_t   The response value
     */
    uint32_t cpsmGetRespX( const ResponseMailbox which ) const;

    /*-------------------------------------------------------------------------
    Command Management
    -------------------------------------------------------------------------*/
    /**
     * @brief Indicate an application specific command is to be sent
     *
     * @param rca   Address of the card to send the command to
     * @return ErrorType
     */
    ErrorType cmdAppCommand( const uint16_t rca );

    /**
     * @brief Send ACMD41 to the SD card
     * Sends host capacity support information and activates the card's initialization process.
     *
     * @param Argument  Host information about supported voltage and capacities
     * @return ErrorType
     */
    ErrorType cmdAppOperCommand( const uint32_t Argument );

    /**
     * @brief Set the data block length for the SD card
     *
     * @param blockSize   Block size in bytes
     * @return ErrorType
     */
    ErrorType cmdBlockLength( const uint32_t blockSize );

    /**
     * @brief Set the bus width for the SD card
     *
     * @param width   Bus width to use
     * @return ErrorType
     */
    ErrorType cmdBusWidth( const Chimera::SDIO::BusWidth width );

    /**
     * @brief Starts erase procedure set up by cmdEraseStartAdd() and cmdEraseEndAdd()
     * @return ErrorType
     */
    ErrorType cmdErase();

    /**
     * @brief Sets the end address for the erase procedure
     *
     * @param address  Address to end erasing at
     * @return ErrorType
     */
    ErrorType cmdEraseEndAdd( const uint32_t address );

    /**
     * @brief Sets the start address for the erase procedure
     *
     * @param address  Address to start erasing at
     * @return ErrorType
     */
    ErrorType cmdEraseStartAdd( const uint32_t address );

    /**
     * @brief Sends CMD0 to the SD card to reset it to Idle state
     * @return ErrorType
     */
    ErrorType cmdGoIdleState();

    /**
     * @brief Sends CMD8 to the SD card
     *
     * Sends SD Memory Card interface condition, which includes host supply
     * voltage information and asks whether card supports this voltage.
     *
     * @return ErrorType
     */
    ErrorType cmdOperCond();

    /**
     * @brief Sends CMD18 to the SD card to read multiple blocks
     *
     * @param address    Address of the block to start reading from
     * @return ErrorType
     */
    ErrorType cmdReadMultiBlock( const uint32_t address );

    /**
     * @brief Sends CMD17 to the SD card to read a single block
     *
     * @param address    Address of the block to start reading from
     * @return ErrorType
     */
    ErrorType cmdReadSingleBlock( const uint32_t address );

    /**
     * @brief Selects or deselects the card
     *
     * @param rca  Address of the card to select or deselect
     * @return ErrorType
     */
    ErrorType cmdSelDesel( const uint16_t rca );

    /**
     * @brief Request the CID from the SD card
     * @return ErrorType
     */
    ErrorType cmdSendCID();

    /**
     * @brief Sends CMD9 to the SD card to get the card's CSD register
     *
     * @param Argumnent  Address of the card to get the CSD from
     * @return ErrorType
     */
    ErrorType cmdSendCSD( const uint16_t rca );

    /**
     * @brief Sends CMD51 to the SD card to read the SCR register
     * @return ErrorType
     */
    ErrorType cmdSendSCR();

    /**
     * @brief Sends CMD13 to the card to get the card's status register
     *
     * @param rca   Address of the card to get the status from
     * @return ErrorType
     */
    ErrorType cmdSendStatus( const uint16_t rca );

    /**
     * @brief Sends CMD3 to the SD card to get the card's relative address
     *
     * @param pRCA  Output parameter to store the card's relative address
     * @return ErrorType
     */
    ErrorType cmdSetRelAdd( uint16_t *const pRCA );

    /**
     * @brief Requests the SD card status register
     * @return ErrorType
     */
    ErrorType cmdStatusRegister();

    /**
     * @brief Sends CMD12 to the SD card to stop a transmission
     * @return ErrorType
     */
    ErrorType cmdStopTransfer();

    /**
     * @brief Sends CMD25 to the SD card to write multiple blocks
     *
     * @param address   Address to write to
     * @return ErrorType
     */
    ErrorType cmdWriteMultiBlock( const uint32_t address );

    /**
     * @brief Sends CMD24 to the SD card to write a single block
     *
     * @param address   Address to write to
     * @return ErrorType
     */
    ErrorType cmdWriteSingleBlock( const uint32_t address );

    /*-------------------------------------------------------------------------
    Response Management
    -------------------------------------------------------------------------*/
    /**
     * @brief Checks for error conditions on the R1 response
     *
     * @param command   Expected command index
     * @param timeout   Timeout in ms
     * @return ErrorType
     */
    ErrorType getCmdResp1( const uint8_t command, const uint32_t timeout ) const;

    /**
     * @brief Checks for error conditions on the R2 response
     * @return ErrorType
     */
    ErrorType getCmdResp2() const;

    /**
     * @brief Checks for error conditions on the R3 response
     * @return ErrorType
     */
    ErrorType getCmdResp3() const;

    /**
     * @brief Checks for error conditions on the R6 response
     *
     * @param command Expected command index
     * @param pRCA    Output parameter to store the card's relative address
     * @return ErrorType
     */
    ErrorType getCmdResp6( const uint8_t command, uint16_t *const pRCA ) const;

    /**
     * @brief Checks for error conditions on the R7 response
     * @return ErrorType
     */
    ErrorType getCmdResp7() const;

    /*-------------------------------------------------------------------------
    Data Management
    -------------------------------------------------------------------------*/
    /**
     * @brief Read out the Stream Control Register (SCR) from the card
     *
     * @param rca   Address of the card to read the SCR from
     * @param pSCR  Pointer to the SCR register copy output location
     * @return LLD::ErrorType
     */
    ErrorType getStreamControlRegister( const uint16_t rca, uint32_t *const pSCR );

    /**
     * @brief Sends ACMD13 to get the SD card status
     *
     * @param rca       Address of the card to get the status from
     * @param pSDstatus Output parameter to store the card's status
     * @return ErrorType
     */
    ErrorType getSDStatus( const uint16_t rca, uint32_t *const pSDstatus );

    /**
     * @brief Sends CMD13 to get the card's status register
     *
     * @param rca         Address of the card to get the status from
     * @param pCardStatus Output parameter to store the card's status
     * @return ErrorType
     */
    ErrorType getCardStatus( const uint16_t rca, uint32_t *const pCardStatus );

    /**
     * @brief Reads a single 512 byte block from the SD card
     *
     * @param address   Starting address to read from (aligned to block size)
     * @param count     Number of blocks to read
     * @param buffer    Buffer to read the data into
     * @return ErrorType
     */
    ErrorType asyncReadBlock( const uint32_t address, const uint32_t count, void *const buffer );

    /**
     * @brief Writes a single 512 byte block to the SD card
     *
     * @param address   Starting address to write into (aligned to block size)
     * @param count     Number of blocks to write
     * @param buffer    Buffer to write the data from
     * @return ErrorType
     */
    ErrorType asyncWriteBlock( const uint32_t address, const uint32_t count, const void *const buffer );

  protected:
    void IRQHandler();
    bool readyForNextTransfer();
    bool waitUntilReady( const uint32_t timeout );

  private:
    friend Chimera::Thread::AsyncIO<Driver>;
    friend void( ::SDIO_IRQHandler )();

    RegisterMap            *mPeriph;        /**< Mapped hardware peripheral */
    RIndex_t                mResourceIndex; /**< Lookup index for mPeriph */
    uint16_t                mRCA;           /**< Relative card address */
    Chimera::DMA::RequestId mTXDMAPipeId;   /**< Request id of the TX DMA pipe for the driver */
    Chimera::DMA::RequestId mRXDMAPipeId;   /**< Request id of the RX DMA pipe for the driver */
  };
}    // namespace Thor::LLD::SDIO

#endif /* !THOR_LLD_INTF_SDIO_DRIVER_HPP */
