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

    Chimera::Status_t dpsmConfigure( const DPSMConfig &config );
    uint32_t          dpsmGetDataCounter();
    uint32_t          dpsmGetFIFOCount();

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
    ErrorType cmdAppOperCommand( const uint32_t Argument);

    ErrorType cmdBlockLength( const uint32_t BlockSize);
    ErrorType cmdBusWidth( const uint32_t BusWidth);
    ErrorType cmdErase();
    ErrorType cmdEraseEndAdd( const uint32_t EndAdd);
    ErrorType cmdEraseStartAdd( const uint32_t StartAdd);
    ErrorType cmdGoIdleState();
    ErrorType cmdOpCondition( const uint32_t Argument);

    /**
     * @brief Sends CMD8 to the SD card
     *
     * Sends SD Memory Card interface condition, which includes host supply
     * voltage information and asks whether card supports this voltage.
     *
     * @return ErrorType
     */
    ErrorType cmdOperCond();

    ErrorType cmdReadMultiBlock( const uint32_t ReadAdd);
    ErrorType cmdReadSingleBlock( const uint32_t ReadAdd);
    ErrorType cmdSDEraseEndAdd( const uint32_t EndAdd);
    ErrorType cmdSDEraseStartAdd( const uint32_t StartAdd);
    ErrorType cmdSelDesel( const uint64_t Addr);
    ErrorType cmdSendCID();
    ErrorType cmdSendCSD( const uint32_t Argument);
    ErrorType cmdSendEXTCSD( const uint32_t Argument);
    ErrorType cmdSendSCR();
    ErrorType cmdSendStatus( const uint32_t Argument);
    ErrorType cmdSetRelAdd( uint16_t *pRCA);
    ErrorType cmdSetRelAddMmc( const uint16_t RCA);
    ErrorType cmdStatusRegister();
    ErrorType cmdStopTransfer();
    ErrorType cmdSwitch( const uint32_t Argument);
    ErrorType cmdWriteMultiBlock( const uint32_t WriteAdd);
    ErrorType cmdWriteSingleBlock( const uint32_t WriteAdd);

    /*-------------------------------------------------------------------------
    Response Management
    -------------------------------------------------------------------------*/

    ErrorType getCmdResp1( uint8_t SD_CMD, uint32_t Timeout );
    ErrorType getCmdResp2();

    /**
     * @brief Checks for error conditions on the R3 response
     * @return ErrorType
     */
    ErrorType getCmdResp3();

    ErrorType getCmdResp6(uint8_t SD_CMD, uint16_t *pRCA);

    /**
     * @brief Checks for error conditions on the R7 response
     * @return ErrorType
     */
    ErrorType getCmdResp7();

  protected:
    void IRQHandler();

  private:
    friend void( ::SDIO_IRQHandler )();

    RegisterMap *mPeriph;      /**< Mapped hardware peripheral */
    RIndex_t     mResourceIdx; /**< Lookup index for mPeriph */
  };
}    // namespace Thor::LLD::SDIO

#endif /* !THOR_LLD_INTF_SDIO_DRIVER_HPP */
