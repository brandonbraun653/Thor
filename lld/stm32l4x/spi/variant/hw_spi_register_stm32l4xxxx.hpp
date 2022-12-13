/******************************************************************************
 *  File Name:
 *    hw_spi_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    SPI register definitions for the STM32L4xxxx series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_REGISTER_STM32L4XXXX_HPP
#define THOR_HW_SPI_REGISTER_STM32L4XXXX_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l432kc.hpp>


namespace Thor::LLD::SPI
{
  /**
   *  Initializes the LLD register resources and memory
   *
   *  @return void
   */


  /*-------------------------------------------------
  Peripheral Instance Memory Map Base
  -------------------------------------------------*/
  static constexpr uint32_t SPI1_BASE_ADDR = Thor::System::MemoryMap::SPI1_PERIPH_START_ADDRESS;
  static constexpr uint32_t SPI2_BASE_ADDR = Thor::System::MemoryMap::SPI2_PERIPH_START_ADDRESS;
  static constexpr uint32_t SPI3_BASE_ADDR = Thor::System::MemoryMap::SPI3_PERIPH_START_ADDRESS;

  /*-------------------------------------------------
  Peripheral Register Definitions
  -------------------------------------------------*/
  /*******************  Bit definition for CR1 register  ********************/
  static constexpr Reg32_t CR1_Msk = 0xFFFF;
  static constexpr Reg32_t CR1_Rst = 0x0000;

  static constexpr Reg32_t CR1_CPHA_Pos     = ( 0U );
  static constexpr Reg32_t CR1_CPHA_Msk     = ( 0x1UL << CR1_CPHA_Pos );
  static constexpr Reg32_t CR1_CPHA         = CR1_CPHA_Msk;
  static constexpr Reg32_t CR1_CPOL_Pos     = ( 1U );
  static constexpr Reg32_t CR1_CPOL_Msk     = ( 0x1UL << CR1_CPOL_Pos );
  static constexpr Reg32_t CR1_CPOL         = CR1_CPOL_Msk;
  static constexpr Reg32_t CR1_MSTR_Pos     = ( 2U );
  static constexpr Reg32_t CR1_MSTR_Msk     = ( 0x1UL << CR1_MSTR_Pos );
  static constexpr Reg32_t CR1_MSTR         = CR1_MSTR_Msk;
  static constexpr Reg32_t CR1_BR_Pos       = ( 3U );
  static constexpr Reg32_t CR1_BR_Msk       = ( 0x7UL << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR           = CR1_BR_Msk;
  static constexpr Reg32_t CR1_BR_0         = ( 0x1UL << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_1         = ( 0x2UL << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_2         = ( 0x4UL << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_2     = ( 0U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_4     = ( 1U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_8     = ( 2U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_16    = ( 3U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_32    = ( 4U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_64    = ( 5U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_128   = ( 6U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_BR_DIV_256   = ( 7U << CR1_BR_Pos );
  static constexpr Reg32_t CR1_SPE_Pos      = ( 6U );
  static constexpr Reg32_t CR1_SPE_Msk      = ( 0x1UL << CR1_SPE_Pos );
  static constexpr Reg32_t CR1_SPE          = CR1_SPE_Msk;
  static constexpr Reg32_t CR1_LSBFIRST_Pos = ( 7U );
  static constexpr Reg32_t CR1_LSBFIRST_Msk = ( 0x1UL << CR1_LSBFIRST_Pos );
  static constexpr Reg32_t CR1_LSBFIRST     = CR1_LSBFIRST_Msk;
  static constexpr Reg32_t CR1_SSI_Pos      = ( 8U );
  static constexpr Reg32_t CR1_SSI_Msk      = ( 0x1UL << CR1_SSI_Pos );
  static constexpr Reg32_t CR1_SSI          = CR1_SSI_Msk;
  static constexpr Reg32_t CR1_SSM_Pos      = ( 9U );
  static constexpr Reg32_t CR1_SSM_Msk      = ( 0x1UL << CR1_SSM_Pos );
  static constexpr Reg32_t CR1_SSM          = CR1_SSM_Msk;
  static constexpr Reg32_t CR1_RXONLY_Pos   = ( 10U );
  static constexpr Reg32_t CR1_RXONLY_Msk   = ( 0x1UL << CR1_RXONLY_Pos );
  static constexpr Reg32_t CR1_RXONLY       = CR1_RXONLY_Msk;
  static constexpr Reg32_t CR1_CRCL_Pos     = ( 11U );
  static constexpr Reg32_t CR1_CRCL_Msk     = ( 0x1UL << CR1_CRCL_Pos );
  static constexpr Reg32_t CR1_CRCL         = CR1_CRCL_Msk;
  static constexpr Reg32_t CR1_CRCNEXT_Pos  = ( 12U );
  static constexpr Reg32_t CR1_CRCNEXT_Msk  = ( 0x1UL << CR1_CRCNEXT_Pos );
  static constexpr Reg32_t CR1_CRCNEXT      = CR1_CRCNEXT_Msk;
  static constexpr Reg32_t CR1_CRCEN_Pos    = ( 13U );
  static constexpr Reg32_t CR1_CRCEN_Msk    = ( 0x1UL << CR1_CRCEN_Pos );
  static constexpr Reg32_t CR1_CRCEN        = CR1_CRCEN_Msk;
  static constexpr Reg32_t CR1_BIDIOE_Pos   = ( 14U );
  static constexpr Reg32_t CR1_BIDIOE_Msk   = ( 0x1UL << CR1_BIDIOE_Pos );
  static constexpr Reg32_t CR1_BIDIOE       = CR1_BIDIOE_Msk;
  static constexpr Reg32_t CR1_BIDIMODE_Pos = ( 15U );
  static constexpr Reg32_t CR1_BIDIMODE_Msk = ( 0x1UL << CR1_BIDIMODE_Pos );
  static constexpr Reg32_t CR1_BIDIMODE     = CR1_BIDIMODE_Msk;

  /*******************  Bit definition for CR2 register  ********************/
  static constexpr Reg32_t CR2_Msk = 0x7FFF;
  static constexpr Reg32_t CR2_Rst = 0x0700;

  static constexpr Reg32_t CR2_RXDMAEN_Pos = ( 0U );
  static constexpr Reg32_t CR2_RXDMAEN_Msk = ( 0x1UL << CR2_RXDMAEN_Pos );
  static constexpr Reg32_t CR2_RXDMAEN     = CR2_RXDMAEN_Msk;
  static constexpr Reg32_t CR2_TXDMAEN_Pos = ( 1U );
  static constexpr Reg32_t CR2_TXDMAEN_Msk = ( 0x1UL << CR2_TXDMAEN_Pos );
  static constexpr Reg32_t CR2_TXDMAEN     = CR2_TXDMAEN_Msk;
  static constexpr Reg32_t CR2_SSOE_Pos    = ( 2U );
  static constexpr Reg32_t CR2_SSOE_Msk    = ( 0x1UL << CR2_SSOE_Pos );
  static constexpr Reg32_t CR2_SSOE        = CR2_SSOE_Msk;
  static constexpr Reg32_t CR2_NSSP_Pos    = ( 3U );
  static constexpr Reg32_t CR2_NSSP_Msk    = ( 0x1UL << CR2_NSSP_Pos );
  static constexpr Reg32_t CR2_NSSP        = CR2_NSSP_Msk;
  static constexpr Reg32_t CR2_FRF_Pos     = ( 4U );
  static constexpr Reg32_t CR2_FRF_Msk     = ( 0x1UL << CR2_FRF_Pos );
  static constexpr Reg32_t CR2_FRF         = CR2_FRF_Msk;
  static constexpr Reg32_t CR2_ERRIE_Pos   = ( 5U );
  static constexpr Reg32_t CR2_ERRIE_Msk   = ( 0x1UL << CR2_ERRIE_Pos );
  static constexpr Reg32_t CR2_ERRIE       = CR2_ERRIE_Msk;
  static constexpr Reg32_t CR2_RXNEIE_Pos  = ( 6U );
  static constexpr Reg32_t CR2_RXNEIE_Msk  = ( 0x1UL << CR2_RXNEIE_Pos );
  static constexpr Reg32_t CR2_RXNEIE      = CR2_RXNEIE_Msk;
  static constexpr Reg32_t CR2_TXEIE_Pos   = ( 7U );
  static constexpr Reg32_t CR2_TXEIE_Msk   = ( 0x1UL << CR2_TXEIE_Pos );
  static constexpr Reg32_t CR2_TXEIE       = CR2_TXEIE_Msk;
  static constexpr Reg32_t CR2_DS_Pos      = ( 8U );
  static constexpr Reg32_t CR2_DS_Msk      = ( 0xFUL << CR2_DS_Pos );
  static constexpr Reg32_t CR2_DS          = CR2_DS_Msk;
  static constexpr Reg32_t CR2_DS_0        = ( 0x1UL << CR2_DS_Pos );
  static constexpr Reg32_t CR2_DS_1        = ( 0x2UL << CR2_DS_Pos );
  static constexpr Reg32_t CR2_DS_2        = ( 0x4UL << CR2_DS_Pos );
  static constexpr Reg32_t CR2_DS_3        = ( 0x8UL << CR2_DS_Pos );
  static constexpr Reg32_t CR2_FRXTH_Pos   = ( 12U );
  static constexpr Reg32_t CR2_FRXTH_Msk   = ( 0x1UL << CR2_FRXTH_Pos );
  static constexpr Reg32_t CR2_FRXTH       = CR2_FRXTH_Msk;
  static constexpr Reg32_t CR2_LDMARX_Pos  = ( 13U );
  static constexpr Reg32_t CR2_LDMARX_Msk  = ( 0x1UL << CR2_LDMARX_Pos );
  static constexpr Reg32_t CR2_LDMARX      = CR2_LDMARX_Msk;
  static constexpr Reg32_t CR2_LDMATX_Pos  = ( 14U );
  static constexpr Reg32_t CR2_LDMATX_Msk  = ( 0x1UL << CR2_LDMATX_Pos );
  static constexpr Reg32_t CR2_LDMATX      = CR2_LDMATX_Msk;

  /********************  Bit definition for SR register  ********************/
  static constexpr Reg32_t SR_Msk = 0x1FF3;
  static constexpr Reg32_t SR_Rst = 0x0002;

  static constexpr Reg32_t SR_RXNE_Pos   = ( 0U );
  static constexpr Reg32_t SR_RXNE_Msk   = ( 0x1UL << SR_RXNE_Pos );
  static constexpr Reg32_t SR_RXNE       = SR_RXNE_Msk;
  static constexpr Reg32_t SR_TXE_Pos    = ( 1U );
  static constexpr Reg32_t SR_TXE_Msk    = ( 0x1UL << SR_TXE_Pos );
  static constexpr Reg32_t SR_TXE        = SR_TXE_Msk;
  static constexpr Reg32_t SR_CHSIDE_Pos = ( 2U );
  static constexpr Reg32_t SR_CHSIDE_Msk = ( 0x1UL << SR_CHSIDE_Pos );
  static constexpr Reg32_t SR_CHSIDE     = SR_CHSIDE_Msk;
  static constexpr Reg32_t SR_UDR_Pos    = ( 3U );
  static constexpr Reg32_t SR_UDR_Msk    = ( 0x1UL << SR_UDR_Pos );
  static constexpr Reg32_t SR_UDR        = SR_UDR_Msk;
  static constexpr Reg32_t SR_CRCERR_Pos = ( 4U );
  static constexpr Reg32_t SR_CRCERR_Msk = ( 0x1UL << SR_CRCERR_Pos );
  static constexpr Reg32_t SR_CRCERR     = SR_CRCERR_Msk;
  static constexpr Reg32_t SR_MODF_Pos   = ( 5U );
  static constexpr Reg32_t SR_MODF_Msk   = ( 0x1UL << SR_MODF_Pos );
  static constexpr Reg32_t SR_MODF       = SR_MODF_Msk;
  static constexpr Reg32_t SR_OVR_Pos    = ( 6U );
  static constexpr Reg32_t SR_OVR_Msk    = ( 0x1UL << SR_OVR_Pos );
  static constexpr Reg32_t SR_OVR        = SR_OVR_Msk;
  static constexpr Reg32_t SR_BSY_Pos    = ( 7U );
  static constexpr Reg32_t SR_BSY_Msk    = ( 0x1UL << SR_BSY_Pos );
  static constexpr Reg32_t SR_BSY        = SR_BSY_Msk;
  static constexpr Reg32_t SR_FRE_Pos    = ( 8U );
  static constexpr Reg32_t SR_FRE_Msk    = ( 0x1UL << SR_FRE_Pos );
  static constexpr Reg32_t SR_FRE        = SR_FRE_Msk;
  static constexpr Reg32_t SR_FRLVL_Pos  = ( 9U );
  static constexpr Reg32_t SR_FRLVL_Msk  = ( 0x3UL << SR_FRLVL_Pos );
  static constexpr Reg32_t SR_FRLVL      = SR_FRLVL_Msk;
  static constexpr Reg32_t SR_FRLVL_0    = ( 0x1UL << SR_FRLVL_Pos );
  static constexpr Reg32_t SR_FRLVL_1    = ( 0x2UL << SR_FRLVL_Pos );
  static constexpr Reg32_t SR_FTLVL_Pos  = ( 11U );
  static constexpr Reg32_t SR_FTLVL_Msk  = ( 0x3UL << SR_FTLVL_Pos );
  static constexpr Reg32_t SR_FTLVL      = SR_FTLVL_Msk;
  static constexpr Reg32_t SR_FTLVL_0    = ( 0x1UL << SR_FTLVL_Pos );
  static constexpr Reg32_t SR_FTLVL_1    = ( 0x2UL << SR_FTLVL_Pos );

  /********************  Bit definition for DR register  ********************/
  static constexpr Reg32_t DR_DR_Pos = ( 0U );
  static constexpr Reg32_t DR_DR_Msk = ( 0xFFFFUL << DR_DR_Pos );
  static constexpr Reg32_t DR_DR     = DR_DR_Msk;

  /*******************  Bit definition for CRCPR register  ******************/
  static constexpr Reg32_t CRCPR_Rst = 0x0007;

  static constexpr Reg32_t CRCPR_CRCPOLY_Pos = ( 0U );
  static constexpr Reg32_t CRCPR_CRCPOLY_Msk = ( 0xFFFFUL << CRCPR_CRCPOLY_Pos );
  static constexpr Reg32_t CRCPR_CRCPOLY     = CRCPR_CRCPOLY_Msk;

  /******************  Bit definition for RXCRCR register  ******************/
  static constexpr Reg32_t RXCRCR_Rst = 0x0000;

  static constexpr Reg32_t RXCRCR_RXCRC_Pos = ( 0U );
  static constexpr Reg32_t RXCRCR_RXCRC_Msk = ( 0xFFFFUL << RXCRCR_RXCRC_Pos );
  static constexpr Reg32_t RXCRCR_RXCRC     = RXCRCR_RXCRC_Msk;

  /******************  Bit definition for TXCRCR register  ******************/
  static constexpr Reg32_t TXCRCR_Rst = 0x0000;

  static constexpr Reg32_t TXCRCR_TXCRC_Pos = ( 0U );
  static constexpr Reg32_t TXCRCR_TXCRC_Msk = ( 0xFFFFUL << TXCRCR_TXCRC_Pos );
  static constexpr Reg32_t TXCRCR_TXCRC     = TXCRCR_TXCRC_Msk;

}    // namespace Thor::LLD::SPI

#endif /* !THOR_HW_SPI_REGISTER_STM32L4XXXX_HPP */
