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
  class Driver
  {
  public:
    Driver();
    ~Driver();

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

    /*-----------------------------------------------------------------------------
    Peripheral Control Functions
    -----------------------------------------------------------------------------*/
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
     * @brief Puts a CPSM command on the bus
     *
     * @param cmd    Command to send
     * @return void
     */
    void cpsmPutCmd( const CPSMCommand &cmd );

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
    uint32_t cpsmGetResponse( const ResponseMailbox which );

    /**
     * @brief Configures the Data Path State Machine
     *
     * @param config  Configuration parameters
     * @return void
     */
    void dpsmConfigure( const DPSMConfig &config );

    uint32_t dpsmGetDataCounter();
    uint32_t dpsmGetFIFOCount();

    /*-------------------------------------------------------------------------
    Command Management
    -------------------------------------------------------------------------*/
    /**
     * @brief Indicate an application specific command is to be sent
     *
     * @param Argument  Command argument
     * @return ErrorType
     */
    ErrorType cmdAppCommand( const uint32_t Argument );

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

    ErrorType cmdErase();
    ErrorType cmdEraseEndAdd( const uint32_t EndAdd );
    ErrorType cmdEraseStartAdd( const uint32_t StartAdd );

    /**
     * @brief Sends CMD0 to the SD card to reset it to Idle state
     * @return ErrorType
     */
    ErrorType cmdGoIdleState();

    ErrorType cmdOpCondition( const uint32_t Argument );

    /**
     * @brief Sends CMD8 to the SD card
     *
     * Sends SD Memory Card interface condition, which includes host supply
     * voltage information and asks whether card supports this voltage.
     *
     * @return ErrorType
     */
    ErrorType cmdOperCond();

    ErrorType cmdReadMultiBlock( const uint32_t ReadAdd );
    ErrorType cmdReadSingleBlock( const uint32_t ReadAdd );
    ErrorType cmdSDEraseEndAdd( const uint32_t EndAdd );
    ErrorType cmdSDEraseStartAdd( const uint32_t StartAdd );

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

    ErrorType cmdSendEXTCSD( const uint32_t Argument );

    /**
     * @brief Sends CMD51 to the SD card to read the SCR register
     * @return ErrorType
     */
    ErrorType cmdSendSCR();

    ErrorType cmdSendStatus( const uint32_t Argument );

    /**
     * @brief Sends CMD3 to the SD card to get the card's relative address
     *
     * @param pRCA  Output parameter to store the card's relative address
     * @return ErrorType
     */
    ErrorType cmdSetRelAdd( uint16_t *const pRCA );

    ErrorType cmdStatusRegister();
    ErrorType cmdStopTransfer();
    ErrorType cmdSwitch( const uint32_t Argument );
    ErrorType cmdWriteMultiBlock( const uint32_t WriteAdd );
    ErrorType cmdWriteSingleBlock( const uint32_t WriteAdd );

    /*-------------------------------------------------------------------------
    Response Management
    -------------------------------------------------------------------------*/

    ErrorType getCmdResp1( uint8_t SD_CMD, uint32_t Timeout );

    /**
     * @brief Gets the response2
     * @return ErrorType
     */
    ErrorType getCmdResp2();

    /**
     * @brief Checks for error conditions on the R3 response
     * @return ErrorType
     */
    ErrorType getCmdResp3();

    ErrorType getCmdResp6( uint8_t SD_CMD, uint16_t *pRCA );

    /**
     * @brief Checks for error conditions on the R7 response
     * @return ErrorType
     */
    ErrorType getCmdResp7();

    /*-------------------------------------------------------------------------
    Data Management
    -------------------------------------------------------------------------*/
    /**
     * @brief Read out the Stream Control Register (SCR) from the card
     *
     * @param pSCR  Pointer to the SCR register copy output location
     * @return LLD::ErrorType
     */
    ErrorType getStreamControlRegister( uint32_t *const pSCR );

    /**
     * @brief Reads a single 512 byte block from the SD card
     * @note Will kick off a DMA transfer and immediately return
     *
     * @param address   Starting address to read from (aligned to block size)
     * @param buffer    Buffer to read the data into
     * @return ErrorType
     */
    ErrorType asyncReadBlock( const uint32_t address, void *const buffer );

  protected:
    void IRQHandler();
    void onDMARXComplete( const Chimera::DMA::TransferStats &stats );

  private:
    friend void( ::SDIO_IRQHandler )();

    RegisterMap            *mPeriph;         /**< Mapped hardware peripheral */
    RIndex_t                mResourceIndex;  /**< Lookup index for mPeriph */
    Chimera::DMA::RequestId mTXDMARequestId; /**< Request id of the TX DMA pipe for the driver */
    Chimera::DMA::RequestId mRXDMARequestId; /**< Request id of the RX DMA pipe for the driver */
  };
}    // namespace Thor::LLD::SDIO

#endif /* !THOR_LLD_INTF_SDIO_DRIVER_HPP */
