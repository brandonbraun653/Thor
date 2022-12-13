/******************************************************************************
 *  File Name:
 *    hw_spi_register_stm32f446xx.hpp
 *
 *  Description:
 *    Explicit hardware register definitions for the STM32F446xx SPI peripherals
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_REGISTER_HPP
#define THOR_HW_SPI_REGISTER_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

namespace Thor::LLD::SPI
{
  /*------------------------------------------------
  Control Register 1
  ------------------------------------------------*/
  static constexpr Reg32_t CR1_Msk        = 0xFFFF;
  static constexpr Reg32_t CR1_Rst        = 0x0000;
  static constexpr Reg32_t CR1_CPHA_Pos   = ( 0U );
  static constexpr Reg32_t CR1_CPHA_Msk   = ( 0x1U << CR1_CPHA_Pos );
  static constexpr Reg32_t CR1_CPHA       = CR1_CPHA_Msk;
  static constexpr Reg32_t CR1_CPOL_Pos   = ( 1U );
  static constexpr Reg32_t CR1_CPOL_Msk   = ( 0x1U << CR1_CPOL_Pos );
  static constexpr Reg32_t CR1_CPOL       = CR1_CPOL_Msk;
  static constexpr Reg32_t CR1_MSTR_Pos   = ( 2U );
  static constexpr Reg32_t CR1_MSTR_Msk   = ( 0x1U << CR1_MSTR_Pos );
  static constexpr Reg32_t CR1_MSTR       = CR1_MSTR_Msk;
  static constexpr Reg32_t CR1_BR_Pos     = ( 3U );
  static constexpr Reg32_t CR1_BR_Msk     = ( 0x7U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR         = CR1_BR_Msk;
  static constexpr Reg32_t CR1_BR_0       = ( 0x1U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_1       = ( 0x2U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_2       = ( 0x4U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_2   = ( 0U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_4   = ( 1U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_8   = ( 2U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_16  = ( 3U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_32  = ( 4U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_64  = ( 5U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_128 = ( 6U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_256 = ( 7U << CR1_BR_Pos );

  static constexpr Reg32_t CR1_SPE_Pos      = ( 6U );
  static constexpr Reg32_t CR1_SPE_Msk      = ( 0x1U << CR1_SPE_Pos );
  static constexpr Reg32_t CR1_SPE          = CR1_SPE_Msk;
  static constexpr Reg32_t CR1_LSBFIRST_Pos = ( 7U );
  static constexpr Reg32_t CR1_LSBFIRST_Msk = ( 0x1U << CR1_LSBFIRST_Pos );
  static constexpr Reg32_t CR1_LSBFIRST     = CR1_LSBFIRST_Msk;
  static constexpr Reg32_t CR1_SSI_Pos      = ( 8U );
  static constexpr Reg32_t CR1_SSI_Msk      = ( 0x1U << CR1_SSI_Pos );
  static constexpr Reg32_t CR1_SSI          = CR1_SSI_Msk;
  static constexpr Reg32_t CR1_SSM_Pos      = ( 9U );
  static constexpr Reg32_t CR1_SSM_Msk      = ( 0x1U << CR1_SSM_Pos );
  static constexpr Reg32_t CR1_SSM          = CR1_SSM_Msk;
  static constexpr Reg32_t CR1_RXONLY_Pos   = ( 10U );
  static constexpr Reg32_t CR1_RXONLY_Msk   = ( 0x1U << CR1_RXONLY_Pos );
  static constexpr Reg32_t CR1_RXONLY       = CR1_RXONLY_Msk;
  static constexpr Reg32_t CR1_DFF_Pos      = ( 11U );
  static constexpr Reg32_t CR1_DFF_Msk      = ( 0x1U << CR1_DFF_Pos );
  static constexpr Reg32_t CR1_DFF          = CR1_DFF_Msk;
  static constexpr Reg32_t CR1_CRCNEXT_Pos  = ( 12U );
  static constexpr Reg32_t CR1_CRCNEXT_Msk  = ( 0x1U << CR1_CRCNEXT_Pos );
  static constexpr Reg32_t CR1_CRCNEXT      = CR1_CRCNEXT_Msk;
  static constexpr Reg32_t CR1_CRCEN_Pos    = ( 13U );
  static constexpr Reg32_t CR1_CRCEN_Msk    = ( 0x1U << CR1_CRCEN_Pos );
  static constexpr Reg32_t CR1_CRCEN        = CR1_CRCEN_Msk;
  static constexpr Reg32_t CR1_BIDIOE_Pos   = ( 14U );
  static constexpr Reg32_t CR1_BIDIOE_Msk   = ( 0x1U << CR1_BIDIOE_Pos );
  static constexpr Reg32_t CR1_BIDIOE       = CR1_BIDIOE_Msk;
  static constexpr Reg32_t CR1_BIDIMODE_Pos = ( 15U );
  static constexpr Reg32_t CR1_BIDIMODE_Msk = ( 0x1U << CR1_BIDIMODE_Pos );
  static constexpr Reg32_t CR1_BIDIMODE     = CR1_BIDIMODE_Msk;

  /*------------------------------------------------
  Control Register 2
  ------------------------------------------------*/
  static constexpr Reg32_t CR2_Msk         = 0x00F7;
  static constexpr Reg32_t CR2_Rst         = 0x0000;
  static constexpr Reg32_t CR2_RXDMAEN_Pos = ( 0U );
  static constexpr Reg32_t CR2_RXDMAEN_Msk = ( 0x1U << CR2_RXDMAEN_Pos );
  static constexpr Reg32_t CR2_RXDMAEN     = CR2_RXDMAEN_Msk;
  static constexpr Reg32_t CR2_TXDMAEN_Pos = ( 1U );
  static constexpr Reg32_t CR2_TXDMAEN_Msk = ( 0x1U << CR2_TXDMAEN_Pos );
  static constexpr Reg32_t CR2_TXDMAEN     = CR2_TXDMAEN_Msk;
  static constexpr Reg32_t CR2_SSOE_Pos    = ( 2U );
  static constexpr Reg32_t CR2_SSOE_Msk    = ( 0x1U << CR2_SSOE_Pos );
  static constexpr Reg32_t CR2_SSOE        = CR2_SSOE_Msk;
  static constexpr Reg32_t CR2_FRF_Pos     = ( 4U );
  static constexpr Reg32_t CR2_FRF_Msk     = ( 0x1U << CR2_FRF_Pos );
  static constexpr Reg32_t CR2_FRF         = CR2_FRF_Msk;
  static constexpr Reg32_t CR2_ERRIE_Pos   = ( 5U );
  static constexpr Reg32_t CR2_ERRIE_Msk   = ( 0x1U << CR2_ERRIE_Pos );
  static constexpr Reg32_t CR2_ERRIE       = CR2_ERRIE_Msk;
  static constexpr Reg32_t CR2_RXNEIE_Pos  = ( 6U );
  static constexpr Reg32_t CR2_RXNEIE_Msk  = ( 0x1U << CR2_RXNEIE_Pos );
  static constexpr Reg32_t CR2_RXNEIE      = CR2_RXNEIE_Msk;
  static constexpr Reg32_t CR2_TXEIE_Pos   = ( 7U );
  static constexpr Reg32_t CR2_TXEIE_Msk   = ( 0x1U << CR2_TXEIE_Pos );
  static constexpr Reg32_t CR2_TXEIE       = CR2_TXEIE_Msk;

  /*------------------------------------------------
  Status Register
  ------------------------------------------------*/
  static constexpr Reg32_t SR_Msk        = 0x01FF;
  static constexpr Reg32_t SR_Rst        = 0x0002;
  static constexpr Reg32_t SR_RXNE_Pos   = ( 0U );
  static constexpr Reg32_t SR_RXNE_Msk   = ( 0x1U << SR_RXNE_Pos );
  static constexpr Reg32_t SR_RXNE       = SR_RXNE_Msk;
  static constexpr Reg32_t SR_TXE_Pos    = ( 1U );
  static constexpr Reg32_t SR_TXE_Msk    = ( 0x1U << SR_TXE_Pos );
  static constexpr Reg32_t SR_TXE        = SR_TXE_Msk;
  static constexpr Reg32_t SR_CHSIDE_Pos = ( 2U );
  static constexpr Reg32_t SR_CHSIDE_Msk = ( 0x1U << SR_CHSIDE_Pos );
  static constexpr Reg32_t SR_CHSIDE     = SR_CHSIDE_Msk;
  static constexpr Reg32_t SR_UDR_Pos    = ( 3U );
  static constexpr Reg32_t SR_UDR_Msk    = ( 0x1U << SR_UDR_Pos );
  static constexpr Reg32_t SR_UDR        = SR_UDR_Msk;
  static constexpr Reg32_t SR_CRCERR_Pos = ( 4U );
  static constexpr Reg32_t SR_CRCERR_Msk = ( 0x1U << SR_CRCERR_Pos );
  static constexpr Reg32_t SR_CRCERR     = SR_CRCERR_Msk;
  static constexpr Reg32_t SR_MODF_Pos   = ( 5U );
  static constexpr Reg32_t SR_MODF_Msk   = ( 0x1U << SR_MODF_Pos );
  static constexpr Reg32_t SR_MODF       = SR_MODF_Msk;
  static constexpr Reg32_t SR_OVR_Pos    = ( 6U );
  static constexpr Reg32_t SR_OVR_Msk    = ( 0x1U << SR_OVR_Pos );
  static constexpr Reg32_t SR_OVR        = SR_OVR_Msk;
  static constexpr Reg32_t SR_BSY_Pos    = ( 7U );
  static constexpr Reg32_t SR_BSY_Msk    = ( 0x1U << SR_BSY_Pos );
  static constexpr Reg32_t SR_BSY        = SR_BSY_Msk;
  static constexpr Reg32_t SR_FRE_Pos    = ( 8U );
  static constexpr Reg32_t SR_FRE_Msk    = ( 0x1U << SR_FRE_Pos );
  static constexpr Reg32_t SR_FRE        = SR_FRE_Msk;

  /*------------------------------------------------
  Data Register
  ------------------------------------------------*/
  static constexpr Reg32_t DR_Msk    = 0xFFFF;
  static constexpr Reg32_t DR_Rst    = 0x0000;
  static constexpr Reg32_t DR_DR_Pos = ( 0U );
  static constexpr Reg32_t DR_DR_Msk = ( 0xFFFFU << DR_DR_Pos );
  static constexpr Reg32_t DR_DR     = DR_DR_Msk;

  /*------------------------------------------------
  CRC Polynomial Register
  ------------------------------------------------*/
  static constexpr Reg32_t CRCPR_Msk         = 0xFFFF;
  static constexpr Reg32_t CRCPR_Rst         = 0x0007;
  static constexpr Reg32_t CRCPR_CRCPOLY_Pos = ( 0U );
  static constexpr Reg32_t CRCPR_CRCPOLY_Msk = ( 0xFFFFU << CRCPR_CRCPOLY_Pos );
  static constexpr Reg32_t CRCPR_CRCPOLY     = CRCPR_CRCPOLY_Msk;

  /*------------------------------------------------
  RX CRC Register
  ------------------------------------------------*/
  static constexpr Reg32_t RXCRCR_Msk       = 0xFFFF;
  static constexpr Reg32_t RXCRCR_Rst       = 0x0000;
  static constexpr Reg32_t RXCRCR_RXCRC_Pos = ( 0U );
  static constexpr Reg32_t RXCRCR_RXCRC_Msk = ( 0xFFFFU << RXCRCR_RXCRC_Pos );
  static constexpr Reg32_t RXCRCR_RXCRC     = RXCRCR_RXCRC_Msk;

  /*------------------------------------------------
  TX CRC Register
  ------------------------------------------------*/
  static constexpr Reg32_t TXCRCR_Msk       = 0xFFFF;
  static constexpr Reg32_t TXCRCR_Rst       = 0x0000;
  static constexpr Reg32_t TXCRCR_TXCRC_Pos = ( 0U );
  static constexpr Reg32_t TXCRCR_TXCRC_Msk = ( 0xFFFFU << TXCRCR_TXCRC_Pos );
  static constexpr Reg32_t TXCRCR_TXCRC     = TXCRCR_TXCRC_Msk;

  /*------------------------------------------------
  I2S Configuration Register
  ------------------------------------------------*/
  static constexpr Reg32_t I2SCFGR_Msk         = 0x1FBF;
  static constexpr Reg32_t I2SCFGR_Rst         = 0x0000;
  static constexpr Reg32_t I2SCFGR_CHLEN_Pos   = ( 0U );
  static constexpr Reg32_t I2SCFGR_CHLEN_Msk   = ( 0x1U << I2SCFGR_CHLEN_Pos );
  static constexpr Reg32_t I2SCFGR_CHLEN       = I2SCFGR_CHLEN_Msk;
  static constexpr Reg32_t I2SCFGR_DATLEN_Pos  = ( 1U );
  static constexpr Reg32_t I2SCFGR_DATLEN_Msk  = ( 0x3U << I2SCFGR_DATLEN_Pos );
  static constexpr Reg32_t I2SCFGR_DATLEN      = I2SCFGR_DATLEN_Msk;
  static constexpr Reg32_t I2SCFGR_DATLEN_0    = ( 0x1U << I2SCFGR_DATLEN_Pos );
  static constexpr Reg32_t I2SCFGR_DATLEN_1    = ( 0x2U << I2SCFGR_DATLEN_Pos );
  static constexpr Reg32_t I2SCFGR_CKPOL_Pos   = ( 3U );
  static constexpr Reg32_t I2SCFGR_CKPOL_Msk   = ( 0x1U << I2SCFGR_CKPOL_Pos );
  static constexpr Reg32_t I2SCFGR_CKPOL       = I2SCFGR_CKPOL_Msk;
  static constexpr Reg32_t I2SCFGR_I2SSTD_Pos  = ( 4U );
  static constexpr Reg32_t I2SCFGR_I2SSTD_Msk  = ( 0x3U << I2SCFGR_I2SSTD_Pos );
  static constexpr Reg32_t I2SCFGR_I2SSTD      = I2SCFGR_I2SSTD_Msk;
  static constexpr Reg32_t I2SCFGR_I2SSTD_0    = ( 0x1U << I2SCFGR_I2SSTD_Pos );
  static constexpr Reg32_t I2SCFGR_I2SSTD_1    = ( 0x2U << I2SCFGR_I2SSTD_Pos );
  static constexpr Reg32_t I2SCFGR_PCMSYNC_Pos = ( 7U );
  static constexpr Reg32_t I2SCFGR_PCMSYNC_Msk = ( 0x1U << I2SCFGR_PCMSYNC_Pos );
  static constexpr Reg32_t I2SCFGR_PCMSYNC     = I2SCFGR_PCMSYNC_Msk;
  static constexpr Reg32_t I2SCFGR_I2SCFG_Pos  = ( 8U );
  static constexpr Reg32_t I2SCFGR_I2SCFG_Msk  = ( 0x3U << I2SCFGR_I2SCFG_Pos );
  static constexpr Reg32_t I2SCFGR_I2SCFG      = I2SCFGR_I2SCFG_Msk;
  static constexpr Reg32_t I2SCFGR_I2SCFG_0    = ( 0x1U << I2SCFGR_I2SCFG_Pos );
  static constexpr Reg32_t I2SCFGR_I2SCFG_1    = ( 0x2U << I2SCFGR_I2SCFG_Pos );
  static constexpr Reg32_t I2SCFGR_I2SE_Pos    = ( 10U );
  static constexpr Reg32_t I2SCFGR_I2SE_Msk    = ( 0x1U << I2SCFGR_I2SE_Pos );
  static constexpr Reg32_t I2SCFGR_I2SE        = I2SCFGR_I2SE_Msk;
  static constexpr Reg32_t I2SCFGR_I2SMOD_Pos  = ( 11U );
  static constexpr Reg32_t I2SCFGR_I2SMOD_Msk  = ( 0x1U << I2SCFGR_I2SMOD_Pos );
  static constexpr Reg32_t I2SCFGR_I2SMOD      = I2SCFGR_I2SMOD_Msk;
  static constexpr Reg32_t I2SCFGR_ASTREN_Pos  = ( 12U );
  static constexpr Reg32_t I2SCFGR_ASTREN_Msk  = ( 0x1U << I2SCFGR_ASTREN_Pos );
  static constexpr Reg32_t I2SCFGR_ASTREN      = I2SCFGR_ASTREN_Msk;

  /*------------------------------------------------
  I2S Prescale Register
  ------------------------------------------------*/
  static constexpr Reg32_t I2SPR_Msk       = 0x03FF;
  static constexpr Reg32_t I2SPR_Rst       = 0x0002;
  static constexpr Reg32_t I2SPR_DIV_Pos   = ( 0U );
  static constexpr Reg32_t I2SPR_DIV_Msk   = ( 0xFFU << I2SPR_DIV_Pos );
  static constexpr Reg32_t I2SPR_DIV       = I2SPR_DIV_Msk;
  static constexpr Reg32_t I2SPR_ODD_Pos   = ( 8U );
  static constexpr Reg32_t I2SPR_ODD_Msk   = ( 0x1U << I2SPR_ODD_Pos );
  static constexpr Reg32_t I2SPR_ODD       = I2SPR_ODD_Msk;
  static constexpr Reg32_t I2SPR_MCKOE_Pos = ( 9U );
  static constexpr Reg32_t I2SPR_MCKOE_Msk = ( 0x1U << I2SPR_MCKOE_Pos );
  static constexpr Reg32_t I2SPR_MCKOE     = I2SPR_MCKOE_Msk;

}    // namespace Thor::LLD::SPI

#endif /* THOR_HW_SPI_REGISTER_HPP */