/******************************************************************************
 *  File Name:
 *    sdio_types.hpp
 *
 *  Description:
 *    Types for the SDIO low level driver
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_INTF_SDIO_HPP
#define THOR_INTF_SDIO_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>

namespace Thor::LLD::SDIO
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum ErrorType : uint32_t
  {
    ERROR_NONE                   = 0x00000000U, /**< No error                                                      */
    ERROR_CMD_CRC_FAIL           = 0x00000001U, /**< Command response received (but CRC check failed)              */
    ERROR_DATA_CRC_FAIL          = 0x00000002U, /**< Data block sent/received (CRC check failed)                   */
    ERROR_CMD_RSP_TIMEOUT        = 0x00000004U, /**< Command response timeout                                      */
    ERROR_DATA_TIMEOUT           = 0x00000008U, /**< Data timeout                                                  */
    ERROR_TX_UNDERRUN            = 0x00000010U, /**< Transmit FIFO underrun                                        */
    ERROR_RX_OVERRUN             = 0x00000020U, /**< Receive FIFO overrun                                          */
    ERROR_ADDR_MISALIGNED        = 0x00000040U, /**< Misaligned address                                            */
    ERROR_BLOCK_LEN_ERR          = 0x00000080U, /**< Txfr block length is not allowed or doesn't match block length */
    ERROR_ERASE_SEQ_ERR          = 0x00000100U, /**< An error in the sequence of erase command occurs              */
    ERROR_BAD_ERASE_PARAM        = 0x00000200U, /**< An invalid selection for erase groups                         */
    ERROR_WRITE_PROT_VIOLATION   = 0x00000400U, /**< Attempt to program a write protect block                      */
    ERROR_LOCK_UNLOCK_FAILED     = 0x00000800U, /**< Sequence or password error, or an attempt to access a locked card */
    ERROR_COM_CRC_FAILED         = 0x00001000U, /**< CRC check of the previous command failed                      */
    ERROR_ILLEGAL_CMD            = 0x00002000U, /**< Command is not legal for the card state                       */
    ERROR_CARD_ECC_FAILED        = 0x00004000U, /**< Card internal ECC was applied but failed to correct the data  */
    ERROR_CC_ERR                 = 0x00008000U, /**< Internal card controller error                                */
    ERROR_GENERAL_UNKNOWN_ERR    = 0x00010000U, /**< General or unknown error                                      */
    ERROR_STREAM_READ_UNDERRUN   = 0x00020000U, /**< The card could not sustain data reading in stream rmode       */
    ERROR_STREAM_WRITE_OVERRUN   = 0x00040000U, /**< The card could not sustain data programming in stream mode    */
    ERROR_CID_CSD_OVERWRITE      = 0x00080000U, /**< CID/CSD overwrite error                                       */
    ERROR_WP_ERASE_SKIP          = 0x00100000U, /**< Only partial address space was erased                         */
    ERROR_CARD_ECC_DISABLED      = 0x00200000U, /**< Command has been executed without using internal ECC          */
    ERROR_ERASE_RESET            = 0x00400000U, /**< Erase sequence cleared. Out of erase sequence command was received */
    ERROR_AKE_SEQ_ERR            = 0x00800000U, /**< Error in sequence of authentication                           */
    ERROR_INVALID_VOLTRANGE      = 0x01000000U, /**< Error in case of invalid voltage range                        */
    ERROR_ADDR_OUT_OF_RANGE      = 0x02000000U, /**< Error when addressed block is out of range                    */
    ERROR_REQUEST_NOT_APPLICABLE = 0x04000000U, /**< Error when command request is not applicable                  */
    ERROR_INVALID_PARAMETER      = 0x08000000U, /**< the used parameter is not valid                               */
    ERROR_UNSUPPORTED_FEATURE    = 0x10000000U, /**< Error when feature is not insupported                         */
    ERROR_BUSY                   = 0x20000000U, /**< Error when transfer process is busy                           */
    ERROR_DMA                    = 0x40000000U, /**< Error while DMA transfer                                      */
    ERROR_TIMEOUT                = 0x80000000U, /**< Timeout error                                                 */
  };

  enum CommandType : uint8_t
  {
    CMD_GO_IDLE_STATE        = 0U,  /**< Resets the SD memory card. */
    CMD_SEND_OP_COND         = 1U,  /**< Sends host capacity support information and activates the card's initialization process. */
    CMD_ALL_SEND_CID         = 2U,  /**< Asks any card connected to the host to send the CID numbers on the CMD line. */
    CMD_SET_REL_ADDR         = 3U,  /**< Asks the card to publish a new relative address (RCA). */
    CMD_SET_DSR              = 4U,  /**< Programs the DSR of all cards. */
    CMD_SDMMC_SEN_OP_COND    = 5U,  /**< Sends host capacity support information (HCS) and asks the accessed card to send its operating condition register (OCR) content in the response on the CMD line. */
    CMD_HS_SWITCH            = 6U,  /**< Checks switchable function (mode 0) and switch card function (mode 1). */
    CMD_SEL_DESEL_CARD       = 7U,  /**< Selects the card by its own relative address and gets deselected by any other address. */
    CMD_HS_SEND_EXT_CSD      = 8U,  /**< Sends SD Memory Card interface condition, which includes host supply voltage information and asks the card whether card supports voltage. */
    CMD_SEND_CSD             = 9U,  /**< Addressed card sends its card specific data (CSD) on the CMD line. */
    CMD_SEND_CID             = 10U, /**< Addressed card sends its card identification (CID) on the CMD line. */
    CMD_READ_DAT_UNTIL_STOP  = 11U, /**< SD card doesn't support it. */
    CMD_STOP_TRANSMISSION    = 12U, /**< Forces the card to stop transmission. */
    CMD_SEND_STATUS          = 13U, /**< Addressed card sends its status register. */
    CMD_HS_BUSTEST_READ      = 14U, /**< Reserved. */
    CMD_GO_INACTIVE_STATE    = 15U, /**< Sends an addressed card into the inactive state. */
    CMD_SET_BLOCKLEN         = 16U, /**< Sets the block length (in bytes for SDSC) for all following block commands (read, write, lock). Default block length is fixed to 512 Bytes. Not effective for SDHS and SDXC. Fixed 512 bytes in case of SDHC and SDXC. */
    CMD_READ_MULT_BLOCK      = 18U, /**< Continuously transfers data blocks from card to host until interrupted by STOP_TRANSMISSION command. */
    CMD_HS_BUSTEST_WRITE     = 19U, /**< 64 bytes tuning pattern is sent for SDR50 and SDR104. */
    CMD_WRITE_DAT_UNTIL_STOP = 20U, /**< Speed class control command. */
    CMD_SET_BLOCK_COUNT      = 23U, /**< Specify block count for CMD18 and CMD25. */
    CMD_WRITE_SINGLE_BLOCK   = 24U, /**< Writes single block of size selected by SET_BLOCKLEN in case of SDSC, and a block of fixed 512 bytes in case of SDHC and SDXC. */
    CMD_WRITE_MULT_BLOCK     = 25U, /**< Continuously writes blocks of data until a STOP_TRANSMISSION follows. */
    CMD_PROG_CID             = 26U, /**< Reserved for manufacturers. */
    CMD_PROG_CSD             = 27U, /**< Programming of the programmable bits of the CSD. */
    CMD_SET_WRITE_PROT       = 28U, /**< Sets the write protection bit of the addressed group. */
    CMD_CLR_WRITE_PROT       = 29U, /**< Clears the write protection bit of the addressed group. */
    CMD_SEND_WRITE_PROT      = 30U, /**< Asks the card to send the status of the write protection bits. */
    CMD_SD_ERASE_GRP_START   = 32U, /**< Sets the address of the first write block to be erased. (For SD card only). */
    CMD_SD_ERASE_GRP_END     = 33U, /**< Sets the address of the last write block of the continuous range to be erased. */
    CMD_ERASE_GRP_START      = 35U, /**< Sets the address of the first write block to be erased. Reserved for each command system set by switch function command (CMD6). */
    CMD_ERASE_GRP_END        = 36U, /**< Sets the address of the last write block of the continuous range to be erased. Reserved for each command system set by switch function command (CMD6). */
    CMD_ERASE                = 38U, /**< Reserved for SD security applications. */
    CMD_FAST_IO              = 39U, /**< SD card doesn't support it (Reserved). */
    CMD_GO_IRQ_STATE         = 40U, /**< SD card doesn't support it (Reserved). */
    CMD_SD_APP_OP_COND       = 41U, /**< (ACMD41) Sends host capacity support information and activates the card's initialization process. */
    CMD_LOCK_UNLOCK          = 42U, /**< Sets/resets the password or lock/unlock the card. The size of the data block is set by the SET_BLOCK_LEN command. */
    CMD_APP_CMD              = 55U, /**< Indicates to the card that the next command is an application specific command rather than a standard command. */
    CMD_GEN_CMD              = 56U, /**< Used either to transfer a data block to the card or to get a data block from the card for general purpose/application specific commands. */
    CMD_NO_CMD               = 64U, /**< No command. */
  };


  enum ResponseMailbox : uint8_t
  {
    RESPONSE_1,
    RESPONSE_2,
    RESPONSE_3,
    RESPONSE_4,
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   * @brief Command Path State Machine (CPSM) Command Settings
   */
  struct CPSMCommand
  {
    uint32_t Argument;         /*!< Specifies the SDMMC command argument which is sent
                                   to a card as part of a command message. If a command
                                   contains an argument, it must be loaded into this register
                                   before writing the command to the command register.              */

    uint32_t CmdIndex;         /*!< Specifies the SDMMC command index. It must be Min_Data = 0 and
                                   Max_Data = 64                                                    */

    uint32_t Response;         /*!< Specifies the SDMMC response type.
                                   This parameter can be a value of @ref SDMMC_LL_Response_Type         */

    uint32_t WaitForInterrupt; /*!< Specifies whether SDMMC wait for interrupt request is
                                   enabled or disabled.
                                   This parameter can be a value of @ref SDMMC_LL_Wait_Interrupt_State  */

    uint32_t CPSM;             /*!< Specifies whether SDMMC Command path state machine (CPSM)
                                   is enabled or disabled.
                                   This parameter can be a value of @ref SDMMC_LL_CPSM_State            */
  };

  struct DPSMConfig
  {
    uint32_t DataTimeOut;   /*!< Specifies the data timeout period in card bus clock periods.  */

    uint32_t DataLength;    /*!< Specifies the number of data bytes to be transferred.         */

    uint32_t DataBlockSize; /*!< Specifies the data block size for block transfer.
                                 This parameter can be a value of @ref SDMMC_LL_Data_Block_Size    */

    uint32_t TransferDir;   /*!< Specifies the data transfer direction, whether the transfer
                                 is a read or write.
                                 This parameter can be a value of @ref SDMMC_LL_Transfer_Direction */

    uint32_t TransferMode;  /*!< Specifies whether data transfer is in stream or block mode.
                                 This parameter can be a value of @ref SDMMC_LL_Transfer_Type      */

    uint32_t DPSM;          /*!< Specifies whether SDMMC Data path state machine (DPSM)
                                 is enabled or disabled.
                                 This parameter can be a value of @ref SDMMC_LL_DPSM_State         */
  };

}    // namespace Thor::LLD::SDIO

#endif /* !THOR_INTF_SDIO_HPP */
