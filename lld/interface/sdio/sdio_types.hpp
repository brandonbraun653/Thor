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
