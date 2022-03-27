/******************************************************************************
 *  File Name:
 *    hw_i2c_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    I2C register definitions for the STM32L4xxxx series chips.
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_I2C_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_I2C_REGISTER_STM32L4XXXX_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <cstdint>
#include <Chimera/common>
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>

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
  /*******************  Bit definition for CR1 register  *******************/
  static constexpr uint32_t CR1_ALL_Msk       = 0x00FFFFFF;
  static constexpr uint32_t CR1_PE_Pos        = ( 0U );
  static constexpr uint32_t CR1_PE_Msk        = ( 0x1UL << CR1_PE_Pos );
  static constexpr uint32_t CR1_PE            = CR1_PE_Msk;
  static constexpr uint32_t CR1_TXIE_Pos      = ( 1U );
  static constexpr uint32_t CR1_TXIE_Msk      = ( 0x1UL << CR1_TXIE_Pos );
  static constexpr uint32_t CR1_TXIE          = CR1_TXIE_Msk;
  static constexpr uint32_t CR1_RXIE_Pos      = ( 2U );
  static constexpr uint32_t CR1_RXIE_Msk      = ( 0x1UL << CR1_RXIE_Pos );
  static constexpr uint32_t CR1_RXIE          = CR1_RXIE_Msk;
  static constexpr uint32_t CR1_ADDRIE_Pos    = ( 3U );
  static constexpr uint32_t CR1_ADDRIE_Msk    = ( 0x1UL << CR1_ADDRIE_Pos );
  static constexpr uint32_t CR1_ADDRIE        = CR1_ADDRIE_Msk;
  static constexpr uint32_t CR1_NACKIE_Pos    = ( 4U );
  static constexpr uint32_t CR1_NACKIE_Msk    = ( 0x1UL << CR1_NACKIE_Pos );
  static constexpr uint32_t CR1_NACKIE        = CR1_NACKIE_Msk;
  static constexpr uint32_t CR1_STOPIE_Pos    = ( 5U );
  static constexpr uint32_t CR1_STOPIE_Msk    = ( 0x1UL << CR1_STOPIE_Pos );
  static constexpr uint32_t CR1_STOPIE        = CR1_STOPIE_Msk;
  static constexpr uint32_t CR1_TCIE_Pos      = ( 6U );
  static constexpr uint32_t CR1_TCIE_Msk      = ( 0x1UL << CR1_TCIE_Pos );
  static constexpr uint32_t CR1_TCIE          = CR1_TCIE_Msk;
  static constexpr uint32_t CR1_ERRIE_Pos     = ( 7U );
  static constexpr uint32_t CR1_ERRIE_Msk     = ( 0x1UL << CR1_ERRIE_Pos );
  static constexpr uint32_t CR1_ERRIE         = CR1_ERRIE_Msk;
  static constexpr uint32_t CR1_DNF_Pos       = ( 8U );
  static constexpr uint32_t CR1_DNF_Msk       = ( 0xFUL << CR1_DNF_Pos );
  static constexpr uint32_t CR1_DNF           = CR1_DNF_Msk;
  static constexpr uint32_t CR1_ANFOFF_Pos    = ( 12U );
  static constexpr uint32_t CR1_ANFOFF_Msk    = ( 0x1UL << CR1_ANFOFF_Pos );
  static constexpr uint32_t CR1_ANFOFF        = CR1_ANFOFF_Msk;
  static constexpr uint32_t CR1_SWRST_Pos     = ( 13U );
  static constexpr uint32_t CR1_SWRST_Msk     = ( 0x1UL << CR1_SWRST_Pos );
  static constexpr uint32_t CR1_SWRST         = CR1_SWRST_Msk;
  static constexpr uint32_t CR1_TXDMAEN_Pos   = ( 14U );
  static constexpr uint32_t CR1_TXDMAEN_Msk   = ( 0x1UL << CR1_TXDMAEN_Pos );
  static constexpr uint32_t CR1_TXDMAEN       = CR1_TXDMAEN_Msk;
  static constexpr uint32_t CR1_RXDMAEN_Pos   = ( 15U );
  static constexpr uint32_t CR1_RXDMAEN_Msk   = ( 0x1UL << CR1_RXDMAEN_Pos );
  static constexpr uint32_t CR1_RXDMAEN       = CR1_RXDMAEN_Msk;
  static constexpr uint32_t CR1_SBC_Pos       = ( 16U );
  static constexpr uint32_t CR1_SBC_Msk       = ( 0x1UL << CR1_SBC_Pos );
  static constexpr uint32_t CR1_SBC           = CR1_SBC_Msk;
  static constexpr uint32_t CR1_NOSTRETCH_Pos = ( 17U );
  static constexpr uint32_t CR1_NOSTRETCH_Msk = ( 0x1UL << CR1_NOSTRETCH_Pos );
  static constexpr uint32_t CR1_NOSTRETCH     = CR1_NOSTRETCH_Msk;
  static constexpr uint32_t CR1_WUPEN_Pos     = ( 18U );
  static constexpr uint32_t CR1_WUPEN_Msk     = ( 0x1UL << CR1_WUPEN_Pos );
  static constexpr uint32_t CR1_WUPEN         = CR1_WUPEN_Msk;
  static constexpr uint32_t CR1_GCEN_Pos      = ( 19U );
  static constexpr uint32_t CR1_GCEN_Msk      = ( 0x1UL << CR1_GCEN_Pos );
  static constexpr uint32_t CR1_GCEN          = CR1_GCEN_Msk;
  static constexpr uint32_t CR1_SMBHEN_Pos    = ( 20U );
  static constexpr uint32_t CR1_SMBHEN_Msk    = ( 0x1UL << CR1_SMBHEN_Pos );
  static constexpr uint32_t CR1_SMBHEN        = CR1_SMBHEN_Msk;
  static constexpr uint32_t CR1_SMBDEN_Pos    = ( 21U );
  static constexpr uint32_t CR1_SMBDEN_Msk    = ( 0x1UL << CR1_SMBDEN_Pos );
  static constexpr uint32_t CR1_SMBDEN        = CR1_SMBDEN_Msk;
  static constexpr uint32_t CR1_ALERTEN_Pos   = ( 22U );
  static constexpr uint32_t CR1_ALERTEN_Msk   = ( 0x1UL << CR1_ALERTEN_Pos );
  static constexpr uint32_t CR1_ALERTEN       = CR1_ALERTEN_Msk;
  static constexpr uint32_t CR1_PECEN_Pos     = ( 23U );
  static constexpr uint32_t CR1_PECEN_Msk     = ( 0x1UL << CR1_PECEN_Pos );
  static constexpr uint32_t CR1_PECEN         = CR1_PECEN_Msk;

  /******************  Bit definition for CR2 register  ********************/
  static constexpr uint32_t CR2_SADD_Pos    = ( 0U );
  static constexpr uint32_t CR2_SADD_Msk    = ( 0x3FFUL << CR2_SADD_Pos );
  static constexpr uint32_t CR2_SADD        = CR2_SADD_Msk;
  static constexpr uint32_t CR2_RD_WRN_Pos  = ( 10U );
  static constexpr uint32_t CR2_RD_WRN_Msk  = ( 0x1UL << CR2_RD_WRN_Pos );
  static constexpr uint32_t CR2_RD_WRN      = CR2_RD_WRN_Msk;
  static constexpr uint32_t CR2_ADD10_Pos   = ( 11U );
  static constexpr uint32_t CR2_ADD10_Msk   = ( 0x1UL << CR2_ADD10_Pos );
  static constexpr uint32_t CR2_ADD10       = CR2_ADD10_Msk;
  static constexpr uint32_t CR2_HEAD10R_Pos = ( 12U );
  static constexpr uint32_t CR2_HEAD10R_Msk = ( 0x1UL << CR2_HEAD10R_Pos );
  static constexpr uint32_t CR2_HEAD10R     = CR2_HEAD10R_Msk;
  static constexpr uint32_t CR2_START_Pos   = ( 13U );
  static constexpr uint32_t CR2_START_Msk   = ( 0x1UL << CR2_START_Pos );
  static constexpr uint32_t CR2_START       = CR2_START_Msk;
  static constexpr uint32_t CR2_STOP_Pos    = ( 14U );
  static constexpr uint32_t CR2_STOP_Msk    = ( 0x1UL << CR2_STOP_Pos );
  static constexpr uint32_t CR2_STOP        = CR2_STOP_Msk;
  static constexpr uint32_t CR2_NACK_Pos    = ( 15U );
  static constexpr uint32_t CR2_NACK_Msk    = ( 0x1UL << CR2_NACK_Pos );
  static constexpr uint32_t CR2_NACK        = CR2_NACK_Msk;
  static constexpr uint32_t CR2_NBYTES_Pos  = ( 16U );
  static constexpr uint32_t CR2_NBYTES_Msk  = ( 0xFFUL << CR2_NBYTES_Pos );
  static constexpr uint32_t CR2_NBYTES      = CR2_NBYTES_Msk;
  static constexpr uint32_t CR2_RELOAD_Pos  = ( 24U );
  static constexpr uint32_t CR2_RELOAD_Msk  = ( 0x1UL << CR2_RELOAD_Pos );
  static constexpr uint32_t CR2_RELOAD      = CR2_RELOAD_Msk;
  static constexpr uint32_t CR2_AUTOEND_Pos = ( 25U );
  static constexpr uint32_t CR2_AUTOEND_Msk = ( 0x1UL << CR2_AUTOEND_Pos );
  static constexpr uint32_t CR2_AUTOEND     = CR2_AUTOEND_Msk;
  static constexpr uint32_t CR2_PECBYTE_Pos = ( 26U );
  static constexpr uint32_t CR2_PECBYTE_Msk = ( 0x1UL << CR2_PECBYTE_Pos );
  static constexpr uint32_t CR2_PECBYTE     = CR2_PECBYTE_Msk;

  /*******************  Bit definition for OAR1 register  ******************/
  static constexpr uint32_t OAR1_OA1_Pos     = ( 0U );
  static constexpr uint32_t OAR1_OA1_Msk     = ( 0x3FFUL << OAR1_OA1_Pos );
  static constexpr uint32_t OAR1_OA1         = OAR1_OA1_Msk;
  static constexpr uint32_t OAR1_OA1MODE_Pos = ( 10U );
  static constexpr uint32_t OAR1_OA1MODE_Msk = ( 0x1UL << OAR1_OA1MODE_Pos );
  static constexpr uint32_t OAR1_OA1MODE     = OAR1_OA1MODE_Msk;
  static constexpr uint32_t OAR1_OA1EN_Pos   = ( 15U );
  static constexpr uint32_t OAR1_OA1EN_Msk   = ( 0x1UL << OAR1_OA1EN_Pos );
  static constexpr uint32_t OAR1_OA1EN       = OAR1_OA1EN_Msk;

  /*******************  Bit definition for OAR2 register  ******************/
  static constexpr uint32_t OAR2_OA2_Pos       = ( 1U );
  static constexpr uint32_t OAR2_OA2_Msk       = ( 0x7FUL << OAR2_OA2_Pos );
  static constexpr uint32_t OAR2_OA2           = OAR2_OA2_Msk;
  static constexpr uint32_t OAR2_OA2MSK_Pos    = ( 8U );
  static constexpr uint32_t OAR2_OA2MSK_Msk    = ( 0x7UL << OAR2_OA2MSK_Pos );
  static constexpr uint32_t OAR2_OA2MSK        = OAR2_OA2MSK_Msk;
  static constexpr uint32_t OAR2_OA2NOMASK     = ( 0x00000000UL );
  static constexpr uint32_t OAR2_OA2MASK01_Pos = ( 8U );
  static constexpr uint32_t OAR2_OA2MASK01_Msk = ( 0x1UL << OAR2_OA2MASK01_Pos );
  static constexpr uint32_t OAR2_OA2MASK01     = OAR2_OA2MASK01_Msk;
  static constexpr uint32_t OAR2_OA2MASK02_Pos = ( 9U );
  static constexpr uint32_t OAR2_OA2MASK02_Msk = ( 0x1UL << OAR2_OA2MASK02_Pos );
  static constexpr uint32_t OAR2_OA2MASK02     = OAR2_OA2MASK02_Msk;
  static constexpr uint32_t OAR2_OA2MASK03_Pos = ( 8U );
  static constexpr uint32_t OAR2_OA2MASK03_Msk = ( 0x3UL << OAR2_OA2MASK03_Pos );
  static constexpr uint32_t OAR2_OA2MASK03     = OAR2_OA2MASK03_Msk;
  static constexpr uint32_t OAR2_OA2MASK04_Pos = ( 10U );
  static constexpr uint32_t OAR2_OA2MASK04_Msk = ( 0x1UL << OAR2_OA2MASK04_Pos );
  static constexpr uint32_t OAR2_OA2MASK04     = OAR2_OA2MASK04_Msk;
  static constexpr uint32_t OAR2_OA2MASK05_Pos = ( 8U );
  static constexpr uint32_t OAR2_OA2MASK05_Msk = ( 0x5UL << OAR2_OA2MASK05_Pos );
  static constexpr uint32_t OAR2_OA2MASK05     = OAR2_OA2MASK05_Msk;
  static constexpr uint32_t OAR2_OA2MASK06_Pos = ( 9U );
  static constexpr uint32_t OAR2_OA2MASK06_Msk = ( 0x3UL << OAR2_OA2MASK06_Pos );
  static constexpr uint32_t OAR2_OA2MASK06     = OAR2_OA2MASK06_Msk;
  static constexpr uint32_t OAR2_OA2MASK07_Pos = ( 8U );
  static constexpr uint32_t OAR2_OA2MASK07_Msk = ( 0x7UL << OAR2_OA2MASK07_Pos );
  static constexpr uint32_t OAR2_OA2MASK07     = OAR2_OA2MASK07_Msk;
  static constexpr uint32_t OAR2_OA2EN_Pos     = ( 15U );
  static constexpr uint32_t OAR2_OA2EN_Msk     = ( 0x1UL << OAR2_OA2EN_Pos );
  static constexpr uint32_t OAR2_OA2EN         = OAR2_OA2EN_Msk;

  /*******************  Bit definition for TIMINGR register *******************/
  static constexpr uint32_t TIMINGR_ALL_Msk    = 0xF0FFFFFF;
  static constexpr uint32_t TIMINGR_SCLL_Pos   = ( 0U );
  static constexpr uint32_t TIMINGR_SCLL_Msk   = ( 0xFFUL << TIMINGR_SCLL_Pos );
  static constexpr uint32_t TIMINGR_SCLL       = TIMINGR_SCLL_Msk;
  static constexpr uint32_t TIMINGR_SCLH_Pos   = ( 8U );
  static constexpr uint32_t TIMINGR_SCLH_Msk   = ( 0xFFUL << TIMINGR_SCLH_Pos );
  static constexpr uint32_t TIMINGR_SCLH       = TIMINGR_SCLH_Msk;
  static constexpr uint32_t TIMINGR_SDADEL_Pos = ( 16U );
  static constexpr uint32_t TIMINGR_SDADEL_Msk = ( 0xFUL << TIMINGR_SDADEL_Pos );
  static constexpr uint32_t TIMINGR_SDADEL     = TIMINGR_SDADEL_Msk;
  static constexpr uint32_t TIMINGR_SCLDEL_Pos = ( 20U );
  static constexpr uint32_t TIMINGR_SCLDEL_Msk = ( 0xFUL << TIMINGR_SCLDEL_Pos );
  static constexpr uint32_t TIMINGR_SCLDEL     = TIMINGR_SCLDEL_Msk;
  static constexpr uint32_t TIMINGR_PRESC_Pos  = ( 28U );
  static constexpr uint32_t TIMINGR_PRESC_Msk  = ( 0xFUL << TIMINGR_PRESC_Pos );
  static constexpr uint32_t TIMINGR_PRESC      = TIMINGR_PRESC_Msk;

  /******************* Bit definition for TIMEOUTR register *******************/
  static constexpr uint32_t TIMEOUTR_TIMEOUTA_Pos = ( 0U );
  static constexpr uint32_t TIMEOUTR_TIMEOUTA_Msk = ( 0xFFFUL << TIMEOUTR_TIMEOUTA_Pos );
  static constexpr uint32_t TIMEOUTR_TIMEOUTA     = TIMEOUTR_TIMEOUTA_Msk;
  static constexpr uint32_t TIMEOUTR_TIDLE_Pos    = ( 12U );
  static constexpr uint32_t TIMEOUTR_TIDLE_Msk    = ( 0x1UL << TIMEOUTR_TIDLE_Pos );
  static constexpr uint32_t TIMEOUTR_TIDLE        = TIMEOUTR_TIDLE_Msk;
  static constexpr uint32_t TIMEOUTR_TIMOUTEN_Pos = ( 15U );
  static constexpr uint32_t TIMEOUTR_TIMOUTEN_Msk = ( 0x1UL << TIMEOUTR_TIMOUTEN_Pos );
  static constexpr uint32_t TIMEOUTR_TIMOUTEN     = TIMEOUTR_TIMOUTEN_Msk;
  static constexpr uint32_t TIMEOUTR_TIMEOUTB_Pos = ( 16U );
  static constexpr uint32_t TIMEOUTR_TIMEOUTB_Msk = ( 0xFFFUL << TIMEOUTR_TIMEOUTB_Pos );
  static constexpr uint32_t TIMEOUTR_TIMEOUTB     = TIMEOUTR_TIMEOUTB_Msk;
  static constexpr uint32_t TIMEOUTR_TEXTEN_Pos   = ( 31U );
  static constexpr uint32_t TIMEOUTR_TEXTEN_Msk   = ( 0x1UL << TIMEOUTR_TEXTEN_Pos );
  static constexpr uint32_t TIMEOUTR_TEXTEN       = TIMEOUTR_TEXTEN_Msk;

  /******************  Bit definition for ISR register  *********************/
  static constexpr uint32_t ISR_ALL_Msk     = 0x00FFFFFF;
  static constexpr uint32_t ISR_TXE_Pos     = ( 0U );
  static constexpr uint32_t ISR_TXE_Msk     = ( 0x1UL << ISR_TXE_Pos );
  static constexpr uint32_t ISR_TXE         = ISR_TXE_Msk;
  static constexpr uint32_t ISR_TXIS_Pos    = ( 1U );
  static constexpr uint32_t ISR_TXIS_Msk    = ( 0x1UL << ISR_TXIS_Pos );
  static constexpr uint32_t ISR_TXIS        = ISR_TXIS_Msk;
  static constexpr uint32_t ISR_RXNE_Pos    = ( 2U );
  static constexpr uint32_t ISR_RXNE_Msk    = ( 0x1UL << ISR_RXNE_Pos );
  static constexpr uint32_t ISR_RXNE        = ISR_RXNE_Msk;
  static constexpr uint32_t ISR_ADDR_Pos    = ( 3U );
  static constexpr uint32_t ISR_ADDR_Msk    = ( 0x1UL << ISR_ADDR_Pos );
  static constexpr uint32_t ISR_ADDR        = ISR_ADDR_Msk;
  static constexpr uint32_t ISR_NACKF_Pos   = ( 4U );
  static constexpr uint32_t ISR_NACKF_Msk   = ( 0x1UL << ISR_NACKF_Pos );
  static constexpr uint32_t ISR_NACKF       = ISR_NACKF_Msk;
  static constexpr uint32_t ISR_STOPF_Pos   = ( 5U );
  static constexpr uint32_t ISR_STOPF_Msk   = ( 0x1UL << ISR_STOPF_Pos );
  static constexpr uint32_t ISR_STOPF       = ISR_STOPF_Msk;
  static constexpr uint32_t ISR_TC_Pos      = ( 6U );
  static constexpr uint32_t ISR_TC_Msk      = ( 0x1UL << ISR_TC_Pos );
  static constexpr uint32_t ISR_TC          = ISR_TC_Msk;
  static constexpr uint32_t ISR_TCR_Pos     = ( 7U );
  static constexpr uint32_t ISR_TCR_Msk     = ( 0x1UL << ISR_TCR_Pos );
  static constexpr uint32_t ISR_TCR         = ISR_TCR_Msk;
  static constexpr uint32_t ISR_BERR_Pos    = ( 8U );
  static constexpr uint32_t ISR_BERR_Msk    = ( 0x1UL << ISR_BERR_Pos );
  static constexpr uint32_t ISR_BERR        = ISR_BERR_Msk;
  static constexpr uint32_t ISR_ARLO_Pos    = ( 9U );
  static constexpr uint32_t ISR_ARLO_Msk    = ( 0x1UL << ISR_ARLO_Pos );
  static constexpr uint32_t ISR_ARLO        = ISR_ARLO_Msk;
  static constexpr uint32_t ISR_OVR_Pos     = ( 10U );
  static constexpr uint32_t ISR_OVR_Msk     = ( 0x1UL << ISR_OVR_Pos );
  static constexpr uint32_t ISR_OVR         = ISR_OVR_Msk;
  static constexpr uint32_t ISR_PECERR_Pos  = ( 11U );
  static constexpr uint32_t ISR_PECERR_Msk  = ( 0x1UL << ISR_PECERR_Pos );
  static constexpr uint32_t ISR_PECERR      = ISR_PECERR_Msk;
  static constexpr uint32_t ISR_TIMEOUT_Pos = ( 12U );
  static constexpr uint32_t ISR_TIMEOUT_Msk = ( 0x1UL << ISR_TIMEOUT_Pos );
  static constexpr uint32_t ISR_TIMEOUT     = ISR_TIMEOUT_Msk;
  static constexpr uint32_t ISR_ALERT_Pos   = ( 13U );
  static constexpr uint32_t ISR_ALERT_Msk   = ( 0x1UL << ISR_ALERT_Pos );
  static constexpr uint32_t ISR_ALERT       = ISR_ALERT_Msk;
  static constexpr uint32_t ISR_BUSY_Pos    = ( 15U );
  static constexpr uint32_t ISR_BUSY_Msk    = ( 0x1UL << ISR_BUSY_Pos );
  static constexpr uint32_t ISR_BUSY        = ISR_BUSY_Msk;
  static constexpr uint32_t ISR_DIR_Pos     = ( 16U );
  static constexpr uint32_t ISR_DIR_Msk     = ( 0x1UL << ISR_DIR_Pos );
  static constexpr uint32_t ISR_DIR         = ISR_DIR_Msk;
  static constexpr uint32_t ISR_ADDCODE_Pos = ( 17U );
  static constexpr uint32_t ISR_ADDCODE_Msk = ( 0x7FUL << ISR_ADDCODE_Pos );
  static constexpr uint32_t ISR_ADDCODE     = ISR_ADDCODE_Msk;

  /******************  Bit definition for ICR register  *********************/
  static constexpr uint32_t ICR_ALL_Msk      = 0x00003F38;
  static constexpr uint32_t ICR_ADDRCF_Pos   = ( 3U );
  static constexpr uint32_t ICR_ADDRCF_Msk   = ( 0x1UL << ICR_ADDRCF_Pos );
  static constexpr uint32_t ICR_ADDRCF       = ICR_ADDRCF_Msk;
  static constexpr uint32_t ICR_NACKCF_Pos   = ( 4U );
  static constexpr uint32_t ICR_NACKCF_Msk   = ( 0x1UL << ICR_NACKCF_Pos );
  static constexpr uint32_t ICR_NACKCF       = ICR_NACKCF_Msk;
  static constexpr uint32_t ICR_STOPCF_Pos   = ( 5U );
  static constexpr uint32_t ICR_STOPCF_Msk   = ( 0x1UL << ICR_STOPCF_Pos );
  static constexpr uint32_t ICR_STOPCF       = ICR_STOPCF_Msk;
  static constexpr uint32_t ICR_BERRCF_Pos   = ( 8U );
  static constexpr uint32_t ICR_BERRCF_Msk   = ( 0x1UL << ICR_BERRCF_Pos );
  static constexpr uint32_t ICR_BERRCF       = ICR_BERRCF_Msk;
  static constexpr uint32_t ICR_ARLOCF_Pos   = ( 9U );
  static constexpr uint32_t ICR_ARLOCF_Msk   = ( 0x1UL << ICR_ARLOCF_Pos );
  static constexpr uint32_t ICR_ARLOCF       = ICR_ARLOCF_Msk;
  static constexpr uint32_t ICR_OVRCF_Pos    = ( 10U );
  static constexpr uint32_t ICR_OVRCF_Msk    = ( 0x1UL << ICR_OVRCF_Pos );
  static constexpr uint32_t ICR_OVRCF        = ICR_OVRCF_Msk;
  static constexpr uint32_t ICR_PECCF_Pos    = ( 11U );
  static constexpr uint32_t ICR_PECCF_Msk    = ( 0x1UL << ICR_PECCF_Pos );
  static constexpr uint32_t ICR_PECCF        = ICR_PECCF_Msk;
  static constexpr uint32_t ICR_TIMOUTCF_Pos = ( 12U );
  static constexpr uint32_t ICR_TIMOUTCF_Msk = ( 0x1UL << ICR_TIMOUTCF_Pos );
  static constexpr uint32_t ICR_TIMOUTCF     = ICR_TIMOUTCF_Msk;
  static constexpr uint32_t ICR_ALERTCF_Pos  = ( 13U );
  static constexpr uint32_t ICR_ALERTCF_Msk  = ( 0x1UL << ICR_ALERTCF_Pos );
  static constexpr uint32_t ICR_ALERTCF      = ICR_ALERTCF_Msk;

  /******************  Bit definition for PECR register  *********************/
  static constexpr uint32_t PECR_PEC_Pos = ( 0U );
  static constexpr uint32_t PECR_PEC_Msk = ( 0xFFUL << PECR_PEC_Pos );
  static constexpr uint32_t PECR_PEC     = PECR_PEC_Msk;

  /******************  Bit definition for RXDR register  *********************/
  static constexpr uint32_t RXDR_RXDATA_Pos = ( 0U );
  static constexpr uint32_t RXDR_RXDATA_Msk = ( 0xFFUL << RXDR_RXDATA_Pos );
  static constexpr uint32_t RXDR_RXDATA     = RXDR_RXDATA_Msk;

  /******************  Bit definition for TXDR register  *********************/
  static constexpr uint32_t TXDR_TXDATA_Pos = ( 0U );
  static constexpr uint32_t TXDR_TXDATA_Msk = ( 0xFFUL << TXDR_TXDATA_Pos );
  static constexpr uint32_t TXDR_TXDATA     = TXDR_TXDATA_Msk;

}    // namespace Thor::LLD::I2C

#endif /* !THOR_HW_I2C_REGISTER_STM32L4XXXX_HPP */
