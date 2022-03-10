/********************************************************************************
 *  File Name:
 *    hw_dma_types.hpp
 *
 *  Description:
 *    STM32 Driver for the DMA Peripheral
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_TYPES_HPP
#define THOR_HW_DMA_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <array>

/* Driver Includes */
#include <Thor/lld/stm32l4x/dma/hw_dma_prj.hpp>
#include <Thor/lld/common/registers/field_accessor.hpp>

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  namespace Configuration
  {
    namespace Mode
    {
      static constexpr uint32_t Normal   = 0u;
      static constexpr uint32_t Circular = CCR_CIRC;
    }    // namespace Mode

    namespace PriorityLevel
    {
      static constexpr uint32_t Low    = ( 0u << CCR_PL_Pos ) & CCR_PL_Msk;
      static constexpr uint32_t Medium = ( 1u << CCR_PL_Pos ) & CCR_PL_Msk;
      static constexpr uint32_t High   = ( 2u << CCR_PL_Pos ) & CCR_PL_Msk;
      static constexpr uint32_t Ultra  = ( 3u << CCR_PL_Pos ) & CCR_PL_Msk;
    }    // namespace PriorityLevel

    namespace MemoryDataSize
    {
      static constexpr uint32_t Byte     = ( 0u << CCR_MSIZE_Pos ) & CCR_MSIZE_Msk;
      static constexpr uint32_t HalfWord = ( 1u << CCR_MSIZE_Pos ) & CCR_MSIZE_Msk;
      static constexpr uint32_t Word     = ( 2u << CCR_MSIZE_Pos ) & CCR_MSIZE_Msk;
    }    // namespace MemoryDataSize

    namespace PeriphDataSize
    {
      static constexpr uint32_t Byte     = ( 0u << CCR_PSIZE_Pos ) & CCR_PSIZE_Msk;
      static constexpr uint32_t HalfWord = ( 1u << CCR_PSIZE_Pos ) & CCR_PSIZE_Msk;
      static constexpr uint32_t Word     = ( 2u << CCR_PSIZE_Pos ) & CCR_PSIZE_Msk;
    }    // namespace PeriphDataSize

    namespace MemoryIncrementMode
    {
      static constexpr uint32_t Fixed     = 0;
      static constexpr uint32_t Increment = CCR_MINC;
    }    // namespace MemoryIncrementMode

    namespace PeriphIncrementMode
    {
      static constexpr uint32_t Fixed     = 0;
      static constexpr uint32_t Increment = CCR_PINC;
    }    // namespace PeriphIncrementMode

    namespace CircularMode
    {
      static constexpr uint32_t Disabled = 0;
      static constexpr uint32_t Enabled  = CCR_CIRC;
    }

    namespace Direction
    {
      static constexpr uint32_t INVALID = 0;
      static constexpr uint32_t P2M     = ( 0u << CCR_DIR_Pos ) & CCR_DIR_Msk;
      static constexpr uint32_t P2P     = ( 0u << CCR_DIR_Pos ) & CCR_DIR_Msk;
      static constexpr uint32_t M2P     = ( 1u << CCR_DIR_Pos ) & CCR_DIR_Msk;
      static constexpr uint32_t M2M     = ( 1u << CCR_DIR_Pos ) & CCR_DIR_Msk;
    }    // namespace Direction
  }    // namespace Configuration

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct StreamMap
  {
    volatile uint32_t CCR;   /**< DMA stream x configuration register     , Address offset: 0x08 + 0x14 * ( x - 1 ) (x=1-7) */
    volatile uint32_t CNDTR; /**< DMA stream x number of data register    , Address offset: 0x0C + 0x14 * ( x - 1 ) (x=1-7) */
    volatile uint32_t CPAR;  /**< DMA stream x peripheral address register, Address offset: 0x10 + 0x14 * ( x - 1 ) (x=1-7) */
    volatile uint32_t CMAR;  /**< DMA stream x memory address register    , Address offset: 0x14 + 0x14 * ( x - 1 ) (x=1-7) */
    volatile uint32_t _reserved; /**< Unused */
  };
  static_assert( sizeof( StreamMap ) == 0x14 );

  struct RegisterMap
  {
    volatile uint32_t ISR;   /**< DMA low interrupt status register,      Address offset: 0x00 */
    volatile uint32_t IFCR;  /**< DMA low interrupt flag clear register,  Address offset: 0x04 */
    StreamMap STREAM1;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM2;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM3;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM4;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM5;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM6;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap STREAM7;       /**< DMA Stream control registers, Address offset: 0x18 x stream number */
    StreamMap _reserved;     /**< Padding */
    volatile uint32_t CSELR; /**< DMA channel selection register      , Address offset: 0xA8 */
  };
  static_assert( offsetof( RegisterMap, STREAM1 ) == 0x08 );
  static_assert( offsetof( RegisterMap, STREAM2 ) == 0x1C );
  static_assert( offsetof( RegisterMap, CSELR ) == 0xA8 );

  /*-------------------------------------------------------------------------------
  Register Classes
  -------------------------------------------------------------------------------*/
  /*---------------------------------------------------------------------------
  Interrupt Status Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, ISR, ISR_Msk, ISR_ALL, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, ISR, ISR_TCIF7, TCIF7, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_HTIF7, HTIF7, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TEIF7, TEIF7, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_GIF7, GIF7, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, ISR, ISR_TCIF6, TCIF6, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_HTIF6, HTIF6, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TEIF6, TEIF6, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_GIF6, GIF6, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, ISR, ISR_TCIF5, TCIF5, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_HTIF5, HTIF5, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TEIF5, TEIF5, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_GIF5, GIF5, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, ISR, ISR_TCIF4, TCIF4, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_HTIF4, HTIF4, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TEIF4, TEIF4, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_GIF4, GIF4, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, ISR, ISR_TCIF3, TCIF3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_HTIF3, HTIF3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TEIF3, TEIF3, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_GIF3,  GIF3, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, ISR, ISR_TCIF2, TCIF2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_HTIF2, HTIF2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TEIF2, TEIF2, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_GIF2, GIF2, BIT_ACCESS_R );

  REG_ACCESSOR( RegisterMap, ISR, ISR_TCIF1, TCIF1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_HTIF1, HTIF1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_TEIF1, TEIF1, BIT_ACCESS_R );
  REG_ACCESSOR( RegisterMap, ISR, ISR_GIF1, GIF1, BIT_ACCESS_R );

  /*---------------------------------------------------------------------------
  Interrupt Flag Clear Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTCIF7, CTCIF7, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CHTIF7, CHTIF7, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTEIF7, CTEIF7, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CGIF7, CGIF7, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTCIF6, CTCIF6, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CHTIF6, CHTIF6, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTEIF6, CTEIF6, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CGIF6, CGIF6, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTCIF5, CTCIF5, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CHTIF5, CHTIF5, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTEIF5, CTEIF5, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CGIF5, CGIF5, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTCIF4, CTCIF4, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CHTIF4, CHTIF4, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTEIF4, CTEIF4, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CGIF4, CGIF4, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTCIF3, CTCIF3, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CHTIF3, CHTIF3, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTEIF3, CTEIF3, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CGIF3, CGIF3, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTCIF2, CTCIF2, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CHTIF2, CHTIF2, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTEIF2, CTEIF2, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CGIF2, CGIF2, BIT_ACCESS_W );

  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTCIF1, CTCIF1, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CHTIF1, CHTIF1, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CTEIF1, CTEIF1, BIT_ACCESS_W );
  REG_ACCESSOR( RegisterMap, IFCR, IFCR_CGIF1, CGIF1, BIT_ACCESS_W );

  /*---------------------------------------------------------------------------
  Channel Configuration Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( StreamMap, CCR, CCR_Msk, CR_ALL, BIT_ACCESS_RW );

  REG_ACCESSOR( StreamMap, CCR, CCR_MEM2MEM, MEM2MEM, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_PL, PL, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_MSIZE, MSIZE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_PSIZE, PSIZE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_MINC, MINC, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_PINC, PINC, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_CIRC, CIRC, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_DIR, DIR, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_TCIE, TCIE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_HTIE, HTIE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_TEIE, TEIE, BIT_ACCESS_RW );
  REG_ACCESSOR( StreamMap, CCR, CCR_EN, EN, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  Channel Number of Data to Transfer Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( StreamMap, CNDTR, CNDTR_NDT, NDT, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  Channel Peripheral Address Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( StreamMap, CPAR, CPAR_PA, PA, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  Channel Memory Address Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( StreamMap, CMAR, CMAR_MA, MA, BIT_ACCESS_RW );

  /*---------------------------------------------------------------------------
  Channel Selection Register
  ---------------------------------------------------------------------------*/
  REG_ACCESSOR( RegisterMap, CSELR, CSELR_Msk, CSELR_ALL, BIT_ACCESS_RW );

  REG_ACCESSOR( RegisterMap, CSELR, CSELR_C1S, CS1S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSELR, CSELR_C2S, CS2S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSELR, CSELR_C3S, CS3S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSELR, CSELR_C4S, CS4S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSELR, CSELR_C5S, CS5S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSELR, CSELR_C6S, CS6S, BIT_ACCESS_RW );
  REG_ACCESSOR( RegisterMap, CSELR, CSELR_C7S, CS7S, BIT_ACCESS_RW );

}    // namespace Thor::LLD::DMA

#endif /* !THOR_HW_DMA_TYPES_HPP */
