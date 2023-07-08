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
     * @return Chimera::Status_t
     */
    Chimera::Status_t cpsmPutCmd( const CPSMCommand &cmd );

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
    uint32_t cpsmGetResponse( const uint8_t which );

    Chimera::Status_t dpsmConfigure( const DPSMConfig &config );
    uint32_t          dpsmGetDataCounter();
    uint32_t          dpsmGetFIFOCount();

    /*-------------------------------------------------------------------------
    Command Management
    -------------------------------------------------------------------------*/
    uint32_t cmdAppCommand( const uint32_t Argument);
    uint32_t cmdAppOperCommand( const uint32_t Argument);
    uint32_t cmdBlockLength( const uint32_t BlockSize);
    uint32_t cmdBusWidth( const uint32_t BusWidth);
    uint32_t cmdErase();
    uint32_t cmdEraseEndAdd( const uint32_t EndAdd);
    uint32_t cmdEraseStartAdd( const uint32_t StartAdd);
    uint32_t cmdGoIdleState();
    uint32_t cmdOpCondition( const uint32_t Argument);
    uint32_t cmdOperCond();
    uint32_t cmdReadMultiBlock( const uint32_t ReadAdd);
    uint32_t cmdReadSingleBlock( const uint32_t ReadAdd);
    uint32_t cmdSDEraseEndAdd( const uint32_t EndAdd);
    uint32_t cmdSDEraseStartAdd( const uint32_t StartAdd);
    uint32_t cmdSelDesel( const uint64_t Addr);
    uint32_t cmdSendCID();
    uint32_t cmdSendCSD( const uint32_t Argument);
    uint32_t cmdSendEXTCSD( const uint32_t Argument);
    uint32_t cmdSendSCR();
    uint32_t cmdSendStatus( const uint32_t Argument);
    uint32_t cmdSetRelAdd( uint16_t *pRCA);
    uint32_t cmdSetRelAddMmc( const uint16_t RCA);
    uint32_t cmdStatusRegister();
    uint32_t cmdStopTransfer();
    uint32_t cmdSwitch( const uint32_t Argument);
    uint32_t cmdWriteMultiBlock( const uint32_t WriteAdd);
    uint32_t cmdWriteSingleBlock( const uint32_t WriteAdd);

    /*-------------------------------------------------------------------------
    Response Management
    -------------------------------------------------------------------------*/
    uint32_t getCmdResp1(uint8_t SD_CMD, uint32_t Timeout);
    uint32_t getCmdResp2();
    uint32_t getCmdResp3();
    uint32_t getCmdResp6(uint8_t SD_CMD, uint16_t *pRCA);
    uint32_t getCmdResp7();

  protected:
    void IRQHandler();

  private:
    friend void( ::SDIO_IRQHandler )();

    RegisterMap *mPeriph;      /**< Mapped hardware peripheral */
    RIndex_t     mResourceIdx; /**< Lookup index for mPeriph */
  };
}    // namespace Thor::LLD::SDIO

#endif /* !THOR_LLD_INTF_SDIO_DRIVER_HPP */
