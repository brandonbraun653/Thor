/********************************************************************************
 *  File Name:
 *    hw_spi_types.hpp
 *
 *  Description:
 *    STM32 Types for the SPI Peripheral
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_TYPES_HPP
#define THOR_HW_SPI_TYPES_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  namespace Configuration
  {
    namespace Direction
    {
      static constexpr Reg32_t HALF_DUPLEX = CR1_BIDIMODE;
      static constexpr Reg32_t FULL_DUPLEX = 0u;
    }    // namespace Direction

    namespace HalfDuplexMode
    {
      static constexpr Reg32_t RX_ONLY = 0u;
      static constexpr Reg32_t TX_ONLY = CR1_BIDIOE;
    }    // namespace HalfDuplexMode

    namespace CRC
    {
      static constexpr Reg32_t ENABLED  = CR1_CRCEN;
      static constexpr Reg32_t DISABLED = 0u;
    }    // namespace CRC

    namespace Width
    {
      static constexpr Reg32_t WIDTH_8_BIT  = 0u;
      static constexpr Reg32_t WIDTH_16_BIT = CR1_DFF;
    }    // namespace Width

    namespace ChipSelectManager
    {
      static constexpr Reg32_t SOFTWARE = CR1_SSM;
      static constexpr Reg32_t HARDWARE = 0u;
    }    // namespace ChipSelectManager

    namespace DataFormat
    {
      static constexpr Reg32_t LSB = CR1_LSBFIRST;
      static constexpr Reg32_t MSB = 0u;
    }    // namespace DataFormat

    namespace ClockFormat
    {
      static constexpr Reg32_t MODE0 = 0u;
      static constexpr Reg32_t MODE1 = CR1_CPHA;
      static constexpr Reg32_t MODE2 = CR1_CPOL;
      static constexpr Reg32_t MODE3 = CR1_CPOL | CR1_CPHA;
    }    // namespace ClockFormat

    namespace ClockDivisor
    {
      static constexpr Reg32_t DIV_2      = CR1_BR_DIV_2;
      static constexpr Reg32_t DIV_4      = CR1_BR_DIV_4;
      static constexpr Reg32_t DIV_8      = CR1_BR_DIV_8;
      static constexpr Reg32_t DIV_16     = CR1_BR_DIV_16;
      static constexpr Reg32_t DIV_32     = CR1_BR_DIV_32;
      static constexpr Reg32_t DIV_64     = CR1_BR_DIV_64;
      static constexpr Reg32_t DIV_128    = CR1_BR_DIV_128;
      static constexpr Reg32_t DIV_256    = CR1_BR_DIV_256;
      static constexpr size_t NUM_OPTIONS = 8u;

      static constexpr std::array<Reg32_t, NUM_OPTIONS> regOptions = { DIV_2,  DIV_4,  DIV_8,   DIV_16,
                                                                       DIV_32, DIV_64, DIV_128, DIV_256 };
      static constexpr std::array<size_t, NUM_OPTIONS> valOptions  = { 2u, 4u, 8u, 16u, 32u, 64u, 128u, 256u };
    }    // namespace ClockDivisor

    namespace Mode
    {
      static constexpr Reg32_t MASTER = CR1_MSTR;
      static constexpr Reg32_t SLAVE  = 0u;
    }    // namespace Mode

    namespace Interrupt
    {
      static constexpr Reg32_t TX_BUFFER_EMPTY_ENABLE      = CR2_TXEIE;
      static constexpr Reg32_t TX_BUFFER_EMPTY_DISABLE     = 0u;
      static constexpr Reg32_t RX_BUFFER_NOT_EMPTY_ENABLE  = CR2_RXNEIE;
      static constexpr Reg32_t RX_BUFFER_NOT_EMPTY_DISABLE = 0u;
      static constexpr Reg32_t ERROR_ENABLE                = CR2_ERRIE;
      static constexpr Reg32_t ERROR_DISABLE               = 0u;
    }    // namespace Interrupt

    namespace DMA
    {
      static constexpr Reg32_t TX_ENABLE  = CR2_TXDMAEN;
      static constexpr Reg32_t TX_DISABLE = 0u;
      static constexpr Reg32_t RX_ENABLE  = CR2_RXDMAEN;
      static constexpr Reg32_t RX_DISABLE = 0u;
    }    // namespace DMA
  }      // namespace Configuration


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    volatile Reg32_t CR1;     /**< Control Register 1,            Address Offset: 0x00 */
    volatile Reg32_t CR2;     /**< Control Register 2,            Address Offset: 0x04 */
    volatile Reg32_t SR;      /**< Status Register,               Address Offset: 0x08 */
    volatile Reg32_t DR;      /**< Data Register,                 Address Offset: 0x0C */
    volatile Reg32_t CRCPR;   /**< CRC Polynomial Register,       Address Offset: 0x10 */
    volatile Reg32_t RXCRCR;  /**< RX CRC Register,               Address Offset: 0x14 */
    volatile Reg32_t TXCRCR;  /**< TX CRC Register,               Address Offset: 0x18 */
    volatile Reg32_t I2SCFGR; /**< I2S Configuration Register,    Address Offset: 0x1C */
    volatile Reg32_t I2SPR;   /**< I2S Pre-scale Register,        Address Offset: 0x20 */
  };


  /*-------------------------------------------------------------------------------
  Classes
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  Control Register 1
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR1, CR1_Msk, CR1_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_BIDIMODE_Msk, BIDIMODE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_BIDIOE_Msk, BIDIOE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_CRCEN_Msk, CRCEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_CRCNEXT_Msk, CRCNEXT, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_RXONLY_Msk, RXONLY, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SSM_Msk, SSM, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SSI_Msk, SSI, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_LSBFIRST_Msk, LSBFIRST, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_SPE_Msk, SPE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_BR_Msk, BR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_MSTR_Msk, MSTR, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_CPOL_Msk, CPOL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR1, CR1_CPHA_Msk, CPHA, BIT_ACCESS_RW );

  // Named "DS" for consistency with the common driver. Functionally the same thing.
  REG_ACCESSOR( RegisterMap, CR1, CR1_DFF_Msk, DS, BIT_ACCESS_RW );

  /*------------------------------------------------
  Control Register 2
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CR2, CR2_Msk, CR2_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_TXEIE_Msk, TXEIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_RXNEIE_Msk, RXNEIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_ERRIE_Msk, ERRIE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_FRF_Msk, FRF, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_SSOE_Msk, SSOE, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_TXDMAEN_Msk, TXDMAEN, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CR2, CR2_RXDMAEN_Msk, RXDMAEN, BIT_ACCESS_RW );

  /*------------------------------------------------
  Status Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, SR, SR_Msk, SR_ALL, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, SR, SR_FRE_Msk, FRE, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR, SR_BSY_Msk, BSY, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR, SR_OVR_Msk, OVR, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR, SR_MODF_Msk, MODF, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR, SR_CRCERR_Msk, CRCERR, BIT_ACCESS_RCW0 );
  REG_ACCESSOR( RegisterMap, SR, SR_TXE_Msk, TXE, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, SR, SR_RXNE_Msk, RXNE, BIT_ACCESS_R );

  /*------------------------------------------------
  CRC Polynomial Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CRCPR, CRCPR_CRCPOLY_Msk, CRCPOLY, BIT_ACCESS_RW );

  /*------------------------------------------------
  RX CRC Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, RXCRCR, RXCRCR_RXCRC_Msk, RXCRC, BIT_ACCESS_RW );

  /*------------------------------------------------
  TX CRC Register
  ------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, TXCRCR, TXCRCR_TXCRC_Msk, TXCRC, BIT_ACCESS_RW );

}    // namespace Thor::LLD::SPI

#endif /* THOR_HW_SPI_REGISTER_HPP */