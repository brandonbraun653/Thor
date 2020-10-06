/********************************************************************************
 *  File Name:
 *    hw_can_types.hpp
 *
 *  Description:
 *    STM32L4 types for the CAN peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_HW_CAN_TYPES_HPP
#define THOR_HW_CAN_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/can/hw_can_prj.hpp>

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct TxMailbox
  {
    uint32_t TIR;  /**< CAN TX mailbox identifier register */
    uint32_t TDTR; /**< CAN mailbox data length control and time stamp register */
    uint32_t TDLR; /**< CAN mailbox data low register */
    uint32_t TDHR; /**< CAN mailbox data high register */
  };

  struct RxMailbox
  {
    uint32_t RIR;  /**< CAN receive FIFO mailbox identifier register */
    uint32_t RDTR; /**< CAN receive FIFO mailbox data length control and time stamp register */
    uint32_t RDLR; /**< CAN receive FIFO mailbox data low register */
    uint32_t RDHR; /**< CAN receive FIFO mailbox data high register */
  };

  struct FilterReg
  {
    uint32_t FR1; /**< CAN Filter bank register 1 */
    uint32_t FR2; /**< CAN Filter bank register 1 */
  };

  struct RegisterMap
  {
    volatile uint32_t MCR;                    /**< CAN master control register,         Address offset: 0x00          */
    volatile uint32_t MSR;                    /**< CAN master status register,          Address offset: 0x04          */
    volatile uint32_t TSR;                    /**< CAN transmit status register,        Address offset: 0x08          */
    volatile uint32_t RF0R;                   /**< CAN receive FIFO 0 register,         Address offset: 0x0C          */
    volatile uint32_t RF1R;                   /**< CAN receive FIFO 1 register,         Address offset: 0x10          */
    volatile uint32_t IER;                    /**< CAN interrupt enable register,       Address offset: 0x14          */
    volatile uint32_t ESR;                    /**< CAN error status register,           Address offset: 0x18          */
    volatile uint32_t BTR;                    /**< CAN bit timing register,             Address offset: 0x1C          */
    uint32_t RESERVED0[ 88 ];                 /**< Reserved, 0x020 - 0x17F                                            */
    volatile TxMailbox sTxMailBox[ 3 ];       /**< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
    volatile RxMailbox sFIFOMailBox[ 2 ];     /**< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
    uint32_t RESERVED1[ 12 ];                 /**< Reserved, 0x1D0 - 0x1FF                                            */
    volatile uint32_t FMR;                    /**< CAN filter master register,          Address offset: 0x200         */
    volatile uint32_t FM1R;                   /**< CAN filter mode register,            Address offset: 0x204         */
    uint32_t RESERVED2;                       /**< Reserved, 0x208                                                    */
    volatile uint32_t FS1R;                   /**< CAN filter scale register,           Address offset: 0x20C         */
    uint32_t RESERVED3;                       /**< Reserved, 0x210                                                    */
    volatile uint32_t FFA1R;                  /**< CAN filter FIFO assignment register, Address offset: 0x214         */
    uint32_t RESERVED4;                       /**< Reserved, 0x218                                                    */
    volatile uint32_t FA1R;                   /**< CAN filter activation register,      Address offset: 0x21C         */
    uint32_t RESERVED5[ 8 ];                  /**< Reserved, 0x220-0x23F                                              */
    volatile FilterReg sFilterRegister[ 28 ]; /**< CAN Filter Register,                 Address offset: 0x240-0x31C   */
  };

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
}  // namespace Thor::LLD::CAN

#endif  /* !THOR_HW_CAN_TYPES_HPP */
