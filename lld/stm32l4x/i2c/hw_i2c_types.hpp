/******************************************************************************
 *  File Name:
 *    hw_i2c_types.hpp
 *
 *  Description:
 *    LLD types for the I2C peripheral
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_I2C_TYPES_HPP
#define THOR_HW_I2C_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/i2c/hw_i2c_prj.hpp>

namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  /**
   * Software representation of the interrupt bits that hardware can handle.
   * These provide fine-grain control of which interrupts the software will
   * listen to. Hardware ORs most of these signals together, which removes
   * most of the control.
   */
  enum InterruptBits : uint32_t
  {
    bRXNE     = ( 1u << 0 ),  /**< Receive buffer not empty */
    bTXIS     = ( 1u << 1 ),  /**< Transmit buffer interrupt status */
    bSTOPF    = ( 1u << 2 ),  /**< Stop received (slave) */
    bTCR      = ( 1u << 3 ),  /**< Transfer complete reload */
    bTC       = ( 1u << 4 ),  /**< Transfer complete */
    bADDR     = ( 1u << 5 ),  /**< Address Matched (slave) */
    bNACKF    = ( 1u << 6 ),  /**< NACK reception */
    bBERR     = ( 1u << 7 ),  /**< Bus error */
    bARLO     = ( 1u << 8 ),  /**< Arbitration lost */
    bOVR      = ( 1u << 9 ),  /**< Overrun/underrun */
    bPECERR   = ( 1u << 10 ), /**< PEC Error */
    bTIMEOUT  = ( 1u << 11 ), /**< Timeout/Tlow error */
    bALERT    = ( 1u << 12 )  /**< SMBus Alert */
  };

  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t CR1;      /**< I2C Control register 1,            Address offset: 0x00 */
    volatile uint32_t CR2;      /**< I2C Control register 2,            Address offset: 0x04 */
    volatile uint32_t OAR1;     /**< I2C Own address 1 register,        Address offset: 0x08 */
    volatile uint32_t OAR2;     /**< I2C Own address 2 register,        Address offset: 0x0C */
    volatile uint32_t TIMINGR;  /**< I2C Timing register,               Address offset: 0x10 */
    volatile uint32_t TIMEOUTR; /**< I2C Timeout register,              Address offset: 0x14 */
    volatile uint32_t ISR;      /**< I2C Interrupt and status register, Address offset: 0x18 */
    volatile uint32_t ICR;      /**< I2C Interrupt clear register,      Address offset: 0x1C */
    volatile uint32_t PECR;     /**< I2C PEC register,                  Address offset: 0x20 */
    volatile uint32_t RXDR;     /**< I2C Receive data register,         Address offset: 0x24 */
    volatile uint32_t TXDR;     /**< I2C Transmit data register,        Address offset: 0x28 */
  };

  /*---------------------------------------------------------------------------
  Register Classes
  ---------------------------------------------------------------------------*/
  /* Control Register 1 */
  REG_ACCESSOR( RegisterMap, CR1, CR1_ALL_Msk, CR1_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_PE_Msk, PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_TXIE_Msk, TXIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_RXIE_Msk, RXIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ADDRIE_Msk, ADDRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_NACKIE_Msk, NACKIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_STOPIE_Msk, STOPIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_TCIE_Msk, TCIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ERRIE_Msk, ERRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_DNF_Msk, DNF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ANFOFF_Msk, ANFOFF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_TXDMAEN_Msk, TXDMAEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_RXDMAEN_Msk, RXDMAEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SBC_Msk, SBC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_NOSTRETCH_Msk, NOSTRETCH, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_WUPEN_Msk, WUPEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_GCEN_Msk, GCEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SMBHEN_Msk, SMBHEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SMBDEN_Msk, SMBDEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ALERTEN_Msk, ALERTEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_PECEN_Msk, PECEN, BIT_ACCESS_RW );

  /* Control Register 2 */
  REG_ACCESSOR( RegisterMap, CR2, CR2_SADD_Msk, SADD, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_RD_WRN_Msk, RDWRN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_ADD10_Msk, ADD10, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_HEAD10R_Msk, HEAD10R, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_START_Msk, START, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR2, CR2_STOP_Msk, STOP, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR2, CR2_NACK_Msk, NACK, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, CR2, CR2_NBYTES_Msk, NBYTES, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_RELOAD_Msk, RELOAD, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_AUTOEND_Msk, AUTOEND, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_PECBYTE_Msk, PECBYTE, BIT_ACCESS_RW );

  /* Own Address Register 1 */
  REG_ACCESSOR( RegisterMap, OAR1, OAR1_OA1_Msk, OA1, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OAR1, OAR1_OA1MODE_Msk, OA1MODE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OAR1, OAR1_OA1EN_Msk, OA1EN, BIT_ACCESS_RW );

  /* Own Address Register 2 */
  REG_ACCESSOR( RegisterMap, OAR2, OAR2_OA2_Msk, OA2, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OAR2, OAR2_OA2MSK_Msk, OA2MSK, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OAR2, OAR2_OA2EN_Msk, OA2EN, BIT_ACCESS_RW );

  /* Timing Register */
  REG_ACCESSOR( RegisterMap, TIMINGR, TIMINGR_ALL_Msk, TIMINGR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMINGR, TIMINGR_SCLL_Msk, SCLL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMINGR, TIMINGR_SCLH_Msk, SCLH, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMINGR, TIMINGR_SDADEL_Msk, SDADEL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMINGR, TIMINGR_SCLDEL_Msk, SCLDEL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMINGR, TIMINGR_PRESC_Msk, PRESC, BIT_ACCESS_RW );

  /* Timeout Register */
  REG_ACCESSOR( RegisterMap, TIMEOUTR, TIMEOUTR_TIMEOUTA_Msk, TIMEOUTA, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMEOUTR, TIMEOUTR_TIDLE_Msk, TIDLE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMEOUTR, TIMEOUTR_TIMOUTEN_Msk, TIMOUTEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMEOUTR, TIMEOUTR_TIMEOUTB_Msk, TIMEOUTB, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, TIMEOUTR, TIMEOUTR_TEXTEN_Msk, TEXTEN, BIT_ACCESS_RW );

  /* Interrupt and Status Register */
  REG_ACCESSOR( RegisterMap, ISR, ISR_ALL_Msk, ISR_ALL, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TXE_Msk, TXE, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TXIS_Msk, TXIS, BIT_ACCESS_RS );
  REG_ACCESSOR( RegisterMap, ISR, ISR_RXNE_Msk, RXNE, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_ADDR_Msk, ADDR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_NACKF_Msk, NACKF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_STOPF_Msk, STOPF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TC_Msk, TC, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TCR_Msk, TCR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_BERR_Msk, BERR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_ARLO_Msk, ARLO, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_OVR_Msk, OVR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_PECERR_Msk, PECERR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TIMEOUT_Msk, TIMEOUT, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_ALERT_Msk, ALERT, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_BUSY_Msk, BUSY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_DIR_Msk, DIR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_ADDCODE_Msk, ADDCODE, BIT_ACCESS_R );

  /* Interrupt Clear Register */
  REG_ACCESSOR( RegisterMap, ICR, ICR_ALL_Msk, ICR_ALL, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_ADDRCF_Msk, ADDRCF, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_NACKCF_Msk, NACKCF, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_STOPCF_Msk, STOPCF, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_BERRCF_Msk, BERRCF, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_ARLOCF_Msk, ARLOCF, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_OVRCF_Msk, OVRCF, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_PECCF_Msk, PECCF, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_TIMOUTCF_Msk, TIMOUTCF, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, ICR, ICR_ALERTCF_Msk, ALERTCF, BIT_ACCESS_W );

  /* Packet Error Check Register */
  REG_ACCESSOR( RegisterMap, PECR, PECR_PEC_Msk, PEC, BIT_ACCESS_R );

  /* Receive Data Register */
  REG_ACCESSOR( RegisterMap, RXDR, RXDR_RXDATA_Msk, RXDATA, BIT_ACCESS_RW );

  /* Transmit Data Register */
  REG_ACCESSOR( RegisterMap, TXDR, TXDR_TXDATA_Msk, TXDATA, BIT_ACCESS_RW );

}    // namespace Thor::LLD::I2C

#endif /* !THOR_HW_I2C_TYPES_HPP */
