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

  static_assert( NUM_CAN_TX_MAILBOX == ARRAY_COUNT( RegisterMap::sTxMailBox ) );

  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/

  /*------------------------------------------------
  Master Control Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, MCR, MCR_Msk, MCR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MCR, MCR_INRQ_Msk, INRQ, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MCR, MCR_SLEEP_Msk, SLEEP, BIT_ACCESS_RW );

  /*------------------------------------------------
  Master Status Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, MSR, MSR_Msk, MSR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, MSR, MSR_INAK_Msk, INAK, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, MSR, MSR_SLAK_Msk, SLAK, BIT_ACCESS_R );

  /*------------------------------------------------
  Transmit Status Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, TSR, TSR_Msk, TSR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TME2_Msk, TME2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TME1_Msk, TME1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TME0_Msk, TME0, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TSR, TSR_RQCP0_Msk, RQCP0, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TERR0_Msk, TERR0, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, TSR, TSR_ALST0_Msk, ALST0, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TXOK0_Msk, TXOK0, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, TSR, TSR_RQCP1_Msk, RQCP1, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TERR1_Msk, TERR1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, TSR, TSR_ALST1_Msk, ALST1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TXOK1_Msk, TXOK1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, TSR, TSR_RQCP2_Msk, RQCP2, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TERR2_Msk, TERR2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, TSR, TSR_ALST2_Msk, ALST2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, TSR, TSR_TXOK2_Msk, TXOK2, BIT_ACCESS_R );

  /*------------------------------------------------
  Receive FIFO 0 Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RF0R, RF0R_Msk, RF0R_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, RF0R, RF0R_FMP0_Msk, FMP0, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, RF0R, RF0R_RFOM0_Msk, RFOM0, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, RF0R, RF0R_FOVR0_Msk, FOVR0, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, RF0R, RF0R_FULL0_Msk, FULL0, BIT_ACCESS_RCW1 );

  /*------------------------------------------------
  Receive FIFO 1 Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RF1R, RF1R_Msk, RF1R_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, RF1R, RF1R_FMP1_Msk, FMP1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, RF1R, RF1R_RFOM1_Msk, RFOM1, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, RF1R, RF1R_FOVR1_Msk, FOVR1, BIT_ACCESS_RCW1 );
  REG_ACCESSOR( RegisterMap, RF1R, RF1R_FULL1_Msk, FULL1, BIT_ACCESS_RCW1 );

  /*------------------------------------------------
  Interrupt Enable Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, IER, IER_Msk, IER_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_TMEIE_Msk, TMEIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_FMPIE0_Msk, FMPIE0, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_FMPIE1_Msk, FMPIE1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_FFIE0_Msk, FFIE0, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_FFIE1_Msk, FFIE1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_FOVIE0_Msk, FOVIE0, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_FOVIE1_Msk, FOVIE1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_EWGIE_Msk, EWGIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_EPVIE_Msk, EPVIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_BOFIE_Msk, BOFIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_LECIE_Msk, LECIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_ERRIE_Msk, ERRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_WKUIE_Msk, WKUIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, IER, IER_SLKIE_Msk, SLKIE, BIT_ACCESS_RW );

  /*------------------------------------------------
  Error Status Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ESR, ESR_Msk, ESR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ESR, ESR_REC_Msk, REC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ESR, ESR_TEC_Msk, TEC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ESR, ESR_LEC_Msk, LEC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ESR, ESR_BOFF_Msk, BOFF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ESR, ESR_EPVF_Msk, EPVF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, ESR, ESR_EWGF_Msk, EWGF, BIT_ACCESS_RW );

  /*------------------------------------------------
  Bit Timing Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, BTR, BTR_Msk, BTR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BTR, BTR_BRP_Msk, BRP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BTR, BTR_TS1_Msk, TS1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BTR, BTR_TS2_Msk, TS2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BTR, BTR_SJW_Msk, SJW, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BTR, BTR_SILM_Msk, SLIM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, BTR, BTR_LBKM_Msk, LBKM, BIT_ACCESS_RW );


}  // namespace Thor::LLD::CAN

#endif  /* !THOR_HW_CAN_TYPES_HPP */
