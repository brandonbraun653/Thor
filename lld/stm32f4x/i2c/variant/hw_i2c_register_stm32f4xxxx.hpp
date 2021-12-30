/******************************************************************************
 *  File Name:
 *    hw_i2c_register_stm32f4xxxx.hpp
 *
 *  Description:
 *    I2C register definitions for the STM32F4xxxx series chips.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_I2C_REGISTER_STM32F4XXXX_HPP
#define THOR_HW_I2C_REGISTER_STM32F4XXXX_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Chimera/common>
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

namespace Thor::LLD::I2C
{
  /*---------------------------------------------------------------------------
  Peripheral Instance Memory Map Base
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t I2C1_BASE_ADDR = Thor::System::MemoryMap::I2C1_PERIPH_START_ADDRESS;
  static constexpr uint32_t I2C2_BASE_ADDR = Thor::System::MemoryMap::I2C2_PERIPH_START_ADDRESS;
  static constexpr uint32_t I2C3_BASE_ADDR = Thor::System::MemoryMap::I2C3_PERIPH_START_ADDRESS;

  /*---------------------------------------------------------------------------
  Peripheral Register Definitions
  ---------------------------------------------------------------------------*/
  /*******************  Bit definition for CR1 register  ********************/
  static constexpr uint32_t CR1_PE_Pos        = ( 0U );
  static constexpr uint32_t CR1_PE_Msk        = ( 0x1UL << CR1_PE_Pos );
  static constexpr uint32_t CR1_PE            = CR1_PE_Msk;
  static constexpr uint32_t CR1_SMBUS_Pos     = ( 1U );
  static constexpr uint32_t CR1_SMBUS_Msk     = ( 0x1UL << CR1_SMBUS_Pos );
  static constexpr uint32_t CR1_SMBUS         = CR1_SMBUS_Msk;
  static constexpr uint32_t CR1_SMBTYPE_Pos   = ( 3U );
  static constexpr uint32_t CR1_SMBTYPE_Msk   = ( 0x1UL << CR1_SMBTYPE_Pos );
  static constexpr uint32_t CR1_SMBTYPE       = CR1_SMBTYPE_Msk;
  static constexpr uint32_t CR1_ENARP_Pos     = ( 4U );
  static constexpr uint32_t CR1_ENARP_Msk     = ( 0x1UL << CR1_ENARP_Pos );
  static constexpr uint32_t CR1_ENARP         = CR1_ENARP_Msk;
  static constexpr uint32_t CR1_ENPEC_Pos     = ( 5U );
  static constexpr uint32_t CR1_ENPEC_Msk     = ( 0x1UL << CR1_ENPEC_Pos );
  static constexpr uint32_t CR1_ENPEC         = CR1_ENPEC_Msk;
  static constexpr uint32_t CR1_ENGC_Pos      = ( 6U );
  static constexpr uint32_t CR1_ENGC_Msk      = ( 0x1UL << CR1_ENGC_Pos );
  static constexpr uint32_t CR1_ENGC          = CR1_ENGC_Msk;
  static constexpr uint32_t CR1_NOSTRETCH_Pos = ( 7U );
  static constexpr uint32_t CR1_NOSTRETCH_Msk = ( 0x1UL << CR1_NOSTRETCH_Pos );
  static constexpr uint32_t CR1_NOSTRETCH     = CR1_NOSTRETCH_Msk;
  static constexpr uint32_t CR1_START_Pos     = ( 8U );
  static constexpr uint32_t CR1_START_Msk     = ( 0x1UL << CR1_START_Pos );
  static constexpr uint32_t CR1_START         = CR1_START_Msk;
  static constexpr uint32_t CR1_STOP_Pos      = ( 9U );
  static constexpr uint32_t CR1_STOP_Msk      = ( 0x1UL << CR1_STOP_Pos );
  static constexpr uint32_t CR1_STOP          = CR1_STOP_Msk;
  static constexpr uint32_t CR1_ACK_Pos       = ( 10U );
  static constexpr uint32_t CR1_ACK_Msk       = ( 0x1UL << CR1_ACK_Pos );
  static constexpr uint32_t CR1_ACK           = CR1_ACK_Msk;
  static constexpr uint32_t CR1_POS_Pos       = ( 11U );
  static constexpr uint32_t CR1_POS_Msk       = ( 0x1UL << CR1_POS_Pos );
  static constexpr uint32_t CR1_POS           = CR1_POS_Msk;
  static constexpr uint32_t CR1_PEC_Pos       = ( 12U );
  static constexpr uint32_t CR1_PEC_Msk       = ( 0x1UL << CR1_PEC_Pos );
  static constexpr uint32_t CR1_PEC           = CR1_PEC_Msk;
  static constexpr uint32_t CR1_ALERT_Pos     = ( 13U );
  static constexpr uint32_t CR1_ALERT_Msk     = ( 0x1UL << CR1_ALERT_Pos );
  static constexpr uint32_t CR1_ALERT         = CR1_ALERT_Msk;
  static constexpr uint32_t CR1_SWRST_Pos     = ( 15U );
  static constexpr uint32_t CR1_SWRST_Msk     = ( 0x1UL << CR1_SWRST_Pos );
  static constexpr uint32_t CR1_SWRST         = CR1_SWRST_Msk;

  /*******************  Bit definition for CR2 register  ********************/
  static constexpr uint32_t CR2_FREQ_Pos = ( 0U );
  static constexpr uint32_t CR2_FREQ_Msk = ( 0x3FUL << CR2_FREQ_Pos );
  static constexpr uint32_t CR2_FREQ     = CR2_FREQ_Msk;
  static constexpr uint32_t CR2_FREQ_0   = ( 0x01UL << CR2_FREQ_Pos );
  static constexpr uint32_t CR2_FREQ_1   = ( 0x02UL << CR2_FREQ_Pos );
  static constexpr uint32_t CR2_FREQ_2   = ( 0x04UL << CR2_FREQ_Pos );
  static constexpr uint32_t CR2_FREQ_3   = ( 0x08UL << CR2_FREQ_Pos );
  static constexpr uint32_t CR2_FREQ_4   = ( 0x10UL << CR2_FREQ_Pos );
  static constexpr uint32_t CR2_FREQ_5   = ( 0x20UL << CR2_FREQ_Pos );

  static constexpr uint32_t CR2_ITERREN_Pos = ( 8U );
  static constexpr uint32_t CR2_ITERREN_Msk = ( 0x1UL << CR2_ITERREN_Pos );
  static constexpr uint32_t CR2_ITERREN     = CR2_ITERREN_Msk;
  static constexpr uint32_t CR2_ITEVTEN_Pos = ( 9U );
  static constexpr uint32_t CR2_ITEVTEN_Msk = ( 0x1UL << CR2_ITEVTEN_Pos );
  static constexpr uint32_t CR2_ITEVTEN     = CR2_ITEVTEN_Msk;
  static constexpr uint32_t CR2_ITBUFEN_Pos = ( 10U );
  static constexpr uint32_t CR2_ITBUFEN_Msk = ( 0x1UL << CR2_ITBUFEN_Pos );
  static constexpr uint32_t CR2_ITBUFEN     = CR2_ITBUFEN_Msk;
  static constexpr uint32_t CR2_DMAEN_Pos   = ( 11U );
  static constexpr uint32_t CR2_DMAEN_Msk   = ( 0x1UL << CR2_DMAEN_Pos );
  static constexpr uint32_t CR2_DMAEN       = CR2_DMAEN_Msk;
  static constexpr uint32_t CR2_LAST_Pos    = ( 12U );
  static constexpr uint32_t CR2_LAST_Msk    = ( 0x1UL << CR2_LAST_Pos );
  static constexpr uint32_t CR2_LAST        = CR2_LAST_Msk;

  /*******************  Bit definition for OAR1 register  *******************/
  static constexpr uint32_t OAR1_ADD_Pos     = ( 0U );
  static constexpr uint32_t OAR1_ADD_Msk     = ( 0x3FFUL << OAR1_ADD_Pos );
  static constexpr uint32_t OAR1_ADD         = OAR1_ADD_Msk;
  static constexpr uint32_t OAR1_ADD0_Pos     = ( 0U );
  static constexpr uint32_t OAR1_ADD0_Msk     = ( 0x1UL << OAR1_ADD0_Pos );
  static constexpr uint32_t OAR1_ADD0         = OAR1_ADD0_Msk;
  static constexpr uint32_t OAR1_ADD1_Pos     = ( 1U );
  static constexpr uint32_t OAR1_ADD1_Msk     = ( 0x1UL << OAR1_ADD1_Pos );
  static constexpr uint32_t OAR1_ADD1         = OAR1_ADD1_Msk;
  static constexpr uint32_t OAR1_ADD2_Pos     = ( 2U );
  static constexpr uint32_t OAR1_ADD2_Msk     = ( 0x1UL << OAR1_ADD2_Pos );
  static constexpr uint32_t OAR1_ADD2         = OAR1_ADD2_Msk;
  static constexpr uint32_t OAR1_ADD3_Pos     = ( 3U );
  static constexpr uint32_t OAR1_ADD3_Msk     = ( 0x1UL << OAR1_ADD3_Pos );
  static constexpr uint32_t OAR1_ADD3         = OAR1_ADD3_Msk;
  static constexpr uint32_t OAR1_ADD4_Pos     = ( 4U );
  static constexpr uint32_t OAR1_ADD4_Msk     = ( 0x1UL << OAR1_ADD4_Pos );
  static constexpr uint32_t OAR1_ADD4         = OAR1_ADD4_Msk;
  static constexpr uint32_t OAR1_ADD5_Pos     = ( 5U );
  static constexpr uint32_t OAR1_ADD5_Msk     = ( 0x1UL << OAR1_ADD5_Pos );
  static constexpr uint32_t OAR1_ADD5         = OAR1_ADD5_Msk;
  static constexpr uint32_t OAR1_ADD6_Pos     = ( 6U );
  static constexpr uint32_t OAR1_ADD6_Msk     = ( 0x1UL << OAR1_ADD6_Pos );
  static constexpr uint32_t OAR1_ADD6         = OAR1_ADD6_Msk;
  static constexpr uint32_t OAR1_ADD7_Pos     = ( 7U );
  static constexpr uint32_t OAR1_ADD7_Msk     = ( 0x1UL << OAR1_ADD7_Pos );
  static constexpr uint32_t OAR1_ADD7         = OAR1_ADD7_Msk;
  static constexpr uint32_t OAR1_ADD8_Pos     = ( 8U );
  static constexpr uint32_t OAR1_ADD8_Msk     = ( 0x1UL << OAR1_ADD8_Pos );
  static constexpr uint32_t OAR1_ADD8         = OAR1_ADD8_Msk;
  static constexpr uint32_t OAR1_ADD9_Pos     = ( 9U );
  static constexpr uint32_t OAR1_ADD9_Msk     = ( 0x1UL << OAR1_ADD9_Pos );
  static constexpr uint32_t OAR1_ADD9         = OAR1_ADD9_Msk;

  static constexpr uint32_t OAR1_ADDMODE_Pos = ( 15U );
  static constexpr uint32_t OAR1_ADDMODE_Msk = ( 0x1UL << OAR1_ADDMODE_Pos );
  static constexpr uint32_t OAR1_ADDMODE     = OAR1_ADDMODE_Msk;

  /*******************  Bit definition for OAR2 register  *******************/
  static constexpr uint32_t OAR2_ENDUAL_Pos = ( 0U );
  static constexpr uint32_t OAR2_ENDUAL_Msk = ( 0x1UL << OAR2_ENDUAL_Pos );
  static constexpr uint32_t OAR2_ENDUAL     = OAR2_ENDUAL_Msk;
  static constexpr uint32_t OAR2_ADD2_Pos   = ( 1U );
  static constexpr uint32_t OAR2_ADD2_Msk   = ( 0x7FUL << OAR2_ADD2_Pos );
  static constexpr uint32_t OAR2_ADD2       = OAR2_ADD2_Msk;

  /********************  Bit definition for DR register  ********************/
  static constexpr uint32_t DR_DR_Pos = ( 0U );
  static constexpr uint32_t DR_DR_Msk = ( 0xFFUL << DR_DR_Pos );
  static constexpr uint32_t DR_DR     = DR_DR_Msk;

  /*******************  Bit definition for SR1 register  ********************/
  static constexpr uint32_t SR1_SB_Pos       = ( 0U );
  static constexpr uint32_t SR1_SB_Msk       = ( 0x1UL << SR1_SB_Pos );
  static constexpr uint32_t SR1_SB           = SR1_SB_Msk;
  static constexpr uint32_t SR1_ADDR_Pos     = ( 1U );
  static constexpr uint32_t SR1_ADDR_Msk     = ( 0x1UL << SR1_ADDR_Pos );
  static constexpr uint32_t SR1_ADDR         = SR1_ADDR_Msk;
  static constexpr uint32_t SR1_BTF_Pos      = ( 2U );
  static constexpr uint32_t SR1_BTF_Msk      = ( 0x1UL << SR1_BTF_Pos );
  static constexpr uint32_t SR1_BTF          = SR1_BTF_Msk;
  static constexpr uint32_t SR1_ADD10_Pos    = ( 3U );
  static constexpr uint32_t SR1_ADD10_Msk    = ( 0x1UL << SR1_ADD10_Pos );
  static constexpr uint32_t SR1_ADD10        = SR1_ADD10_Msk;
  static constexpr uint32_t SR1_STOPF_Pos    = ( 4U );
  static constexpr uint32_t SR1_STOPF_Msk    = ( 0x1UL << SR1_STOPF_Pos );
  static constexpr uint32_t SR1_STOPF        = SR1_STOPF_Msk;
  static constexpr uint32_t SR1_RXNE_Pos     = ( 6U );
  static constexpr uint32_t SR1_RXNE_Msk     = ( 0x1UL << SR1_RXNE_Pos );
  static constexpr uint32_t SR1_RXNE         = SR1_RXNE_Msk;
  static constexpr uint32_t SR1_TXE_Pos      = ( 7U );
  static constexpr uint32_t SR1_TXE_Msk      = ( 0x1UL << SR1_TXE_Pos );
  static constexpr uint32_t SR1_TXE          = SR1_TXE_Msk;
  static constexpr uint32_t SR1_BERR_Pos     = ( 8U );
  static constexpr uint32_t SR1_BERR_Msk     = ( 0x1UL << SR1_BERR_Pos );
  static constexpr uint32_t SR1_BERR         = SR1_BERR_Msk;
  static constexpr uint32_t SR1_ARLO_Pos     = ( 9U );
  static constexpr uint32_t SR1_ARLO_Msk     = ( 0x1UL << SR1_ARLO_Pos );
  static constexpr uint32_t SR1_ARLO         = SR1_ARLO_Msk;
  static constexpr uint32_t SR1_AF_Pos       = ( 10U );
  static constexpr uint32_t SR1_AF_Msk       = ( 0x1UL << SR1_AF_Pos );
  static constexpr uint32_t SR1_AF           = SR1_AF_Msk;
  static constexpr uint32_t SR1_OVR_Pos      = ( 11U );
  static constexpr uint32_t SR1_OVR_Msk      = ( 0x1UL << SR1_OVR_Pos );
  static constexpr uint32_t SR1_OVR          = SR1_OVR_Msk;
  static constexpr uint32_t SR1_PECERR_Pos   = ( 12U );
  static constexpr uint32_t SR1_PECERR_Msk   = ( 0x1UL << SR1_PECERR_Pos );
  static constexpr uint32_t SR1_PECERR       = SR1_PECERR_Msk;
  static constexpr uint32_t SR1_TIMEOUT_Pos  = ( 14U );
  static constexpr uint32_t SR1_TIMEOUT_Msk  = ( 0x1UL << SR1_TIMEOUT_Pos );
  static constexpr uint32_t SR1_TIMEOUT      = SR1_TIMEOUT_Msk;
  static constexpr uint32_t SR1_SMBALERT_Pos = ( 15U );
  static constexpr uint32_t SR1_SMBALERT_Msk = ( 0x1UL << SR1_SMBALERT_Pos );
  static constexpr uint32_t SR1_SMBALERT     = SR1_SMBALERT_Msk;

  /*******************  Bit definition for SR2 register  ********************/
  static constexpr uint32_t SR2_MSL_Pos        = ( 0U );
  static constexpr uint32_t SR2_MSL_Msk        = ( 0x1UL << SR2_MSL_Pos );
  static constexpr uint32_t SR2_MSL            = SR2_MSL_Msk;
  static constexpr uint32_t SR2_BUSY_Pos       = ( 1U );
  static constexpr uint32_t SR2_BUSY_Msk       = ( 0x1UL << SR2_BUSY_Pos );
  static constexpr uint32_t SR2_BUSY           = SR2_BUSY_Msk;
  static constexpr uint32_t SR2_TRA_Pos        = ( 2U );
  static constexpr uint32_t SR2_TRA_Msk        = ( 0x1UL << SR2_TRA_Pos );
  static constexpr uint32_t SR2_TRA            = SR2_TRA_Msk;
  static constexpr uint32_t SR2_GENCALL_Pos    = ( 4U );
  static constexpr uint32_t SR2_GENCALL_Msk    = ( 0x1UL << SR2_GENCALL_Pos );
  static constexpr uint32_t SR2_GENCALL        = SR2_GENCALL_Msk;
  static constexpr uint32_t SR2_SMBDEFAULT_Pos = ( 5U );
  static constexpr uint32_t SR2_SMBDEFAULT_Msk = ( 0x1UL << SR2_SMBDEFAULT_Pos );
  static constexpr uint32_t SR2_SMBDEFAULT     = SR2_SMBDEFAULT_Msk;
  static constexpr uint32_t SR2_SMBHOST_Pos    = ( 6U );
  static constexpr uint32_t SR2_SMBHOST_Msk    = ( 0x1UL << SR2_SMBHOST_Pos );
  static constexpr uint32_t SR2_SMBHOST        = SR2_SMBHOST_Msk;
  static constexpr uint32_t SR2_DUALF_Pos      = ( 7U );
  static constexpr uint32_t SR2_DUALF_Msk      = ( 0x1UL << SR2_DUALF_Pos );
  static constexpr uint32_t SR2_DUALF          = SR2_DUALF_Msk;
  static constexpr uint32_t SR2_PEC_Pos        = ( 8U );
  static constexpr uint32_t SR2_PEC_Msk        = ( 0xFFUL << SR2_PEC_Pos );
  static constexpr uint32_t SR2_PEC            = SR2_PEC_Msk;

  /*******************  Bit definition for CCR register  ********************/
  static constexpr uint32_t CCR_CCR_Pos  = ( 0U );
  static constexpr uint32_t CCR_CCR_Msk  = ( 0xFFFUL << CCR_CCR_Pos );
  static constexpr uint32_t CCR_CCR      = CCR_CCR_Msk;
  static constexpr uint32_t CCR_DUTY_Pos = ( 14U );
  static constexpr uint32_t CCR_DUTY_Msk = ( 0x1UL << CCR_DUTY_Pos );
  static constexpr uint32_t CCR_DUTY     = CCR_DUTY_Msk;
  static constexpr uint32_t CCR_FS_Pos   = ( 15U );
  static constexpr uint32_t CCR_FS_Msk   = ( 0x1UL << CCR_FS_Pos );
  static constexpr uint32_t CCR_FS       = CCR_FS_Msk;

  /******************  Bit definition for TRISE register  *******************/
  static constexpr uint32_t TRISE_TRISE_Pos = ( 0U );
  static constexpr uint32_t TRISE_TRISE_Msk = ( 0x3FUL << TRISE_TRISE_Pos );
  static constexpr uint32_t TRISE_TRISE     = TRISE_TRISE_Msk;

  /******************  Bit definition for FLTR register  *******************/
  static constexpr uint32_t FLTR_DNF_Pos   = ( 0U );
  static constexpr uint32_t FLTR_DNF_Msk   = ( 0xFUL << FLTR_DNF_Pos );
  static constexpr uint32_t FLTR_DNF       = FLTR_DNF_Msk;
  static constexpr uint32_t FLTR_ANOFF_Pos = ( 4U );
  static constexpr uint32_t FLTR_ANOFF_Msk = ( 0x1UL << FLTR_ANOFF_Pos );
  static constexpr uint32_t FLTR_ANOFF     = FLTR_ANOFF_Msk;

}    // namespace Thor::LLD::I2C

#endif /* !THOR_HW_I2C_REGISTER_STM32F4XXXX_HPP */
