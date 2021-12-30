/******************************************************************************
 *  File Name:
 *    hw_i2c_types.hpp
 *
 *  Description:
 *    LLD types for the I2C peripheral
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_I2C_TYPES_HPP
#define THOR_HW_I2C_TYPES_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32f4x/i2c/hw_i2c_prj.hpp>

namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile uint32_t CR1;   /**< I2C Control register 1,     Address offset: 0x00 */
    volatile uint32_t CR2;   /**< I2C Control register 2,     Address offset: 0x04 */
    volatile uint32_t OAR1;  /**< I2C Own address register 1, Address offset: 0x08 */
    volatile uint32_t OAR2;  /**< I2C Own address register 2, Address offset: 0x0C */
    volatile uint32_t DR;    /**< I2C Data register,          Address offset: 0x10 */
    volatile uint32_t SR1;   /**< I2C Status register 1,      Address offset: 0x14 */
    volatile uint32_t SR2;   /**< I2C Status register 2,      Address offset: 0x18 */
    volatile uint32_t CCR;   /**< I2C Clock control register, Address offset: 0x1C */
    volatile uint32_t TRISE; /**< I2C TRISE register,         Address offset: 0x20 */
    volatile uint32_t FLTR;  /**< I2C FLTR register,          Address offset: 0x24 */
  };

  /*---------------------------------------------------------------------------
  Register Classes
  ---------------------------------------------------------------------------*/
  /* Control Register 1 */
  REG_ACCESSOR( RegisterMap, CR1, CR1_PE_Msk, PE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SMBUS_Msk, SMBUS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SMBTYPE_Msk, SMBTYPE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ENARP_Msk, ENARP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ENPEC_Msk, ENPEC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ENGC_Msk, ENGC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_NOSTRETCH_Msk, NOSTRETCH, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_START_Msk, START, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_STOP_Msk, STOP, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ACK_Msk, ACK, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_POS_Msk, POS, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_PEC_Msk, CR1_PEC, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_ALERT_Msk, ALERT, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SWRST_Msk, SWRST, BIT_ACCESS_RW );

  /* Control Register 2 */
  REG_ACCESSOR( RegisterMap, CR2, CR2_FREQ_Msk, FREQ, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_ITERREN_Msk, ITERREN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_ITEVTEN_Msk, ITEVTEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_ITBUFEN_Msk, ITBUFEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_DMAEN_Msk, DMAEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_LAST_Msk, LAST, BIT_ACCESS_RW );

  /* Own Address Register 1 */
  REG_ACCESSOR( RegisterMap, OAR1, OAR1_ADD_Msk, ADD, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OAR1, OAR1_ADDMODE_Msk, ADDMODE, BIT_ACCESS_RW );

  /* Own Address Register 2 */
  REG_ACCESSOR( RegisterMap, OAR2, OAR2_ENDUAL_Msk, ENDUAL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, OAR2, OAR2_ADD2_Msk, ADD2, BIT_ACCESS_RW );

  /* Data Register */
  REG_ACCESSOR( RegisterMap, DR, DR_DR_Msk, DR, BIT_ACCESS_RW );

  /* Status Register 1 */
  REG_ACCESSOR( RegisterMap, SR1, SR1_SB_Msk, SB, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR1, SR1_ADDR_Msk, ADDR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR1, SR1_BTF_Msk, BTF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR1, SR1_ADD10_Msk, ADD10, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR1, SR1_STOPF_Msk, STOPF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR1, SR1_RXNE_Msk, RXNE, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR1, SR1_TXE_Msk, TXE, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR1, SR1_BERR_Msk, BERR, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR1, SR1_ARLO_Msk, ARLO, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR1, SR1_AF_Msk, AF, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR1, SR1_OVR_Msk, OVR, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR1, SR1_PECERR_Msk, PECERR, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR1, SR1_TIMEOUT_Msk, TIMEOUT, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR1, SR1_SMBALERT_Msk, SMBALERT, BIT_ACCESS_RCW0 );

  /* Status Register 2 */
  REG_ACCESSOR( RegisterMap, SR2, SR2_MSL_Msk, MSL, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR2, SR2_BUSY_Msk, BUSY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR2, SR2_TRA_Msk, TRA, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR2, SR2_GENCALL_Msk, GENCALL, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR2, SR2_SMBDEFAULT_Msk, SMBDEFAULT, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR2, SR2_SMBHOST_Msk, SMBHOST, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR2, SR2_DUALF_Msk, DUALF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR2, SR2_PEC_Msk, SR2_PEC, BIT_ACCESS_R );

  /* Clock Control Register */
  REG_ACCESSOR( RegisterMap, CCR, CCR_CCR_Msk, CCR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR, CCR_DUTY_Msk, DUTY, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CCR, CCR_FS_Msk, FS, BIT_ACCESS_RW );

  /* TRISE Register */
  REG_ACCESSOR( RegisterMap, TRISE, TRISE_TRISE_Msk, TRISE, BIT_ACCESS_RW );

  /* FLTR Register */
  REG_ACCESSOR( RegisterMap, FLTR, FLTR_DNF_Msk, DNF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, FLTR, FLTR_ANOFF_Msk, ANOFF, BIT_ACCESS_RW );

}    // namespace Thor::LLD::I2C

#endif /* !THOR_HW_I2C_TYPES_HPP */
