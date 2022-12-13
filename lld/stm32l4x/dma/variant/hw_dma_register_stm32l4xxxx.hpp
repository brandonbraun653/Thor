/******************************************************************************
 *  File Name:
 *    hw_dma_register_stm32l4xxxx.hpp
 *
 *  Description:
 *    DMA register definitions for the STM32F4xxxx series chips
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_REGISTER_STM32L4XXX_HPP
#define THOR_HW_DMA_REGISTER_STM32L4XXX_HPP

#include <Chimera/common>
#include <Thor/lld/stm32l4x/system/sys_memory_map_prj.hpp>

#if defined( STM32L432xx )
#include <Thor/lld/stm32l4x/dma/variant/hw_dma_register_stm32l432xx.hpp>
#endif

namespace Thor::LLD::DMA
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DMA_STREAMS_TOTAL         = NUM_DMA_PERIPHS * NUM_DMA_STREAMS_PER_PERIPH;
  static constexpr size_t NUM_DMA_CHANNELS_TOTAL        = NUM_DMA_PERIPHS * NUM_DMA_CHANNELS_PER_STREAM;
  static constexpr size_t NUM_DMA_REQUEST_SIGNALS_TOTAL = NUM_DMA_STREAMS_PER_PERIPH * NUM_DMA_CHANNELS_PER_STREAM;

  /*---------------------------------------------------------------------------
  Peripheral Instance Memory Map
  ---------------------------------------------------------------------------*/
  static constexpr Reg32_t DMA1_BASE_ADDR         = Thor::System::MemoryMap::DMA1_PERIPH_START_ADDRESS;
  static constexpr Reg32_t DMA1_STREAM1_BASE_ADDR = DMA1_BASE_ADDR + 0x008u;
  static constexpr Reg32_t DMA1_STREAM2_BASE_ADDR = DMA1_BASE_ADDR + 0x01Cu;
  static constexpr Reg32_t DMA1_STREAM3_BASE_ADDR = DMA1_BASE_ADDR + 0x030u;
  static constexpr Reg32_t DMA1_STREAM4_BASE_ADDR = DMA1_BASE_ADDR + 0x044u;
  static constexpr Reg32_t DMA1_STREAM5_BASE_ADDR = DMA1_BASE_ADDR + 0x058u;
  static constexpr Reg32_t DMA1_STREAM6_BASE_ADDR = DMA1_BASE_ADDR + 0x06Cu;
  static constexpr Reg32_t DMA1_STREAM7_BASE_ADDR = DMA1_BASE_ADDR + 0x080u;

  static constexpr Reg32_t DMA2_BASE_ADDR         = Thor::System::MemoryMap::DMA2_PERIPH_START_ADDRESS;
  static constexpr Reg32_t DMA2_STREAM1_BASE_ADDR = DMA2_BASE_ADDR + 0x008u;
  static constexpr Reg32_t DMA2_STREAM2_BASE_ADDR = DMA2_BASE_ADDR + 0x01Cu;
  static constexpr Reg32_t DMA2_STREAM3_BASE_ADDR = DMA2_BASE_ADDR + 0x030u;
  static constexpr Reg32_t DMA2_STREAM4_BASE_ADDR = DMA2_BASE_ADDR + 0x044u;
  static constexpr Reg32_t DMA2_STREAM5_BASE_ADDR = DMA2_BASE_ADDR + 0x058u;
  static constexpr Reg32_t DMA2_STREAM6_BASE_ADDR = DMA2_BASE_ADDR + 0x06Cu;
  static constexpr Reg32_t DMA2_STREAM7_BASE_ADDR = DMA2_BASE_ADDR + 0x080u;

  /*---------------------------------------------------------------------------
  Peripheral Register Definitions
  ---------------------------------------------------------------------------*/
  /*---------------------------------------------------------------------------
  Interrupt Status Register
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t ISR_Msk       = 0x0FFFFFFF;
  static constexpr uint32_t ISR_Rst       = 0x00;
  static constexpr uint32_t ISR_GIF1_Pos  = ( 0U );
  static constexpr uint32_t ISR_GIF1_Msk  = ( 0x1UL << ISR_GIF1_Pos );
  static constexpr uint32_t ISR_GIF1      = ISR_GIF1_Msk;
  static constexpr uint32_t ISR_TCIF1_Pos = ( 1U );
  static constexpr uint32_t ISR_TCIF1_Msk = ( 0x1UL << ISR_TCIF1_Pos );
  static constexpr uint32_t ISR_TCIF1     = ISR_TCIF1_Msk;
  static constexpr uint32_t ISR_HTIF1_Pos = ( 2U );
  static constexpr uint32_t ISR_HTIF1_Msk = ( 0x1UL << ISR_HTIF1_Pos );
  static constexpr uint32_t ISR_HTIF1     = ISR_HTIF1_Msk;
  static constexpr uint32_t ISR_TEIF1_Pos = ( 3U );
  static constexpr uint32_t ISR_TEIF1_Msk = ( 0x1UL << ISR_TEIF1_Pos );
  static constexpr uint32_t ISR_TEIF1     = ISR_TEIF1_Msk;
  static constexpr uint32_t ISR_GIF2_Pos  = ( 4U );
  static constexpr uint32_t ISR_GIF2_Msk  = ( 0x1UL << ISR_GIF2_Pos );
  static constexpr uint32_t ISR_GIF2      = ISR_GIF2_Msk;
  static constexpr uint32_t ISR_TCIF2_Pos = ( 5U );
  static constexpr uint32_t ISR_TCIF2_Msk = ( 0x1UL << ISR_TCIF2_Pos );
  static constexpr uint32_t ISR_TCIF2     = ISR_TCIF2_Msk;
  static constexpr uint32_t ISR_HTIF2_Pos = ( 6U );
  static constexpr uint32_t ISR_HTIF2_Msk = ( 0x1UL << ISR_HTIF2_Pos );
  static constexpr uint32_t ISR_HTIF2     = ISR_HTIF2_Msk;
  static constexpr uint32_t ISR_TEIF2_Pos = ( 7U );
  static constexpr uint32_t ISR_TEIF2_Msk = ( 0x1UL << ISR_TEIF2_Pos );
  static constexpr uint32_t ISR_TEIF2     = ISR_TEIF2_Msk;
  static constexpr uint32_t ISR_GIF3_Pos  = ( 8U );
  static constexpr uint32_t ISR_GIF3_Msk  = ( 0x1UL << ISR_GIF3_Pos );
  static constexpr uint32_t ISR_GIF3      = ISR_GIF3_Msk;
  static constexpr uint32_t ISR_TCIF3_Pos = ( 9U );
  static constexpr uint32_t ISR_TCIF3_Msk = ( 0x1UL << ISR_TCIF3_Pos );
  static constexpr uint32_t ISR_TCIF3     = ISR_TCIF3_Msk;
  static constexpr uint32_t ISR_HTIF3_Pos = ( 10U );
  static constexpr uint32_t ISR_HTIF3_Msk = ( 0x1UL << ISR_HTIF3_Pos );
  static constexpr uint32_t ISR_HTIF3     = ISR_HTIF3_Msk;
  static constexpr uint32_t ISR_TEIF3_Pos = ( 11U );
  static constexpr uint32_t ISR_TEIF3_Msk = ( 0x1UL << ISR_TEIF3_Pos );
  static constexpr uint32_t ISR_TEIF3     = ISR_TEIF3_Msk;
  static constexpr uint32_t ISR_GIF4_Pos  = ( 12U );
  static constexpr uint32_t ISR_GIF4_Msk  = ( 0x1UL << ISR_GIF4_Pos );
  static constexpr uint32_t ISR_GIF4      = ISR_GIF4_Msk;
  static constexpr uint32_t ISR_TCIF4_Pos = ( 13U );
  static constexpr uint32_t ISR_TCIF4_Msk = ( 0x1UL << ISR_TCIF4_Pos );
  static constexpr uint32_t ISR_TCIF4     = ISR_TCIF4_Msk;
  static constexpr uint32_t ISR_HTIF4_Pos = ( 14U );
  static constexpr uint32_t ISR_HTIF4_Msk = ( 0x1UL << ISR_HTIF4_Pos );
  static constexpr uint32_t ISR_HTIF4     = ISR_HTIF4_Msk;
  static constexpr uint32_t ISR_TEIF4_Pos = ( 15U );
  static constexpr uint32_t ISR_TEIF4_Msk = ( 0x1UL << ISR_TEIF4_Pos );
  static constexpr uint32_t ISR_TEIF4     = ISR_TEIF4_Msk;
  static constexpr uint32_t ISR_GIF5_Pos  = ( 16U );
  static constexpr uint32_t ISR_GIF5_Msk  = ( 0x1UL << ISR_GIF5_Pos );
  static constexpr uint32_t ISR_GIF5      = ISR_GIF5_Msk;
  static constexpr uint32_t ISR_TCIF5_Pos = ( 17U );
  static constexpr uint32_t ISR_TCIF5_Msk = ( 0x1UL << ISR_TCIF5_Pos );
  static constexpr uint32_t ISR_TCIF5     = ISR_TCIF5_Msk;
  static constexpr uint32_t ISR_HTIF5_Pos = ( 18U );
  static constexpr uint32_t ISR_HTIF5_Msk = ( 0x1UL << ISR_HTIF5_Pos );
  static constexpr uint32_t ISR_HTIF5     = ISR_HTIF5_Msk;
  static constexpr uint32_t ISR_TEIF5_Pos = ( 19U );
  static constexpr uint32_t ISR_TEIF5_Msk = ( 0x1UL << ISR_TEIF5_Pos );
  static constexpr uint32_t ISR_TEIF5     = ISR_TEIF5_Msk;
  static constexpr uint32_t ISR_GIF6_Pos  = ( 20U );
  static constexpr uint32_t ISR_GIF6_Msk  = ( 0x1UL << ISR_GIF6_Pos );
  static constexpr uint32_t ISR_GIF6      = ISR_GIF6_Msk;
  static constexpr uint32_t ISR_TCIF6_Pos = ( 21U );
  static constexpr uint32_t ISR_TCIF6_Msk = ( 0x1UL << ISR_TCIF6_Pos );
  static constexpr uint32_t ISR_TCIF6     = ISR_TCIF6_Msk;
  static constexpr uint32_t ISR_HTIF6_Pos = ( 22U );
  static constexpr uint32_t ISR_HTIF6_Msk = ( 0x1UL << ISR_HTIF6_Pos );
  static constexpr uint32_t ISR_HTIF6     = ISR_HTIF6_Msk;
  static constexpr uint32_t ISR_TEIF6_Pos = ( 23U );
  static constexpr uint32_t ISR_TEIF6_Msk = ( 0x1UL << ISR_TEIF6_Pos );
  static constexpr uint32_t ISR_TEIF6     = ISR_TEIF6_Msk;
  static constexpr uint32_t ISR_GIF7_Pos  = ( 24U );
  static constexpr uint32_t ISR_GIF7_Msk  = ( 0x1UL << ISR_GIF7_Pos );
  static constexpr uint32_t ISR_GIF7      = ISR_GIF7_Msk;
  static constexpr uint32_t ISR_TCIF7_Pos = ( 25U );
  static constexpr uint32_t ISR_TCIF7_Msk = ( 0x1UL << ISR_TCIF7_Pos );
  static constexpr uint32_t ISR_TCIF7     = ISR_TCIF7_Msk;
  static constexpr uint32_t ISR_HTIF7_Pos = ( 26U );
  static constexpr uint32_t ISR_HTIF7_Msk = ( 0x1UL << ISR_HTIF7_Pos );
  static constexpr uint32_t ISR_HTIF7     = ISR_HTIF7_Msk;
  static constexpr uint32_t ISR_TEIF7_Pos = ( 27U );
  static constexpr uint32_t ISR_TEIF7_Msk = ( 0x1UL << ISR_TEIF7_Pos );
  static constexpr uint32_t ISR_TEIF7     = ISR_TEIF7_Msk;

  /*---------------------------------------------------------------------------
  Interrupt Flag Clear Register
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t IFCR_Msk        = 0x0FFFFFFF;
  static constexpr uint32_t IFCR_Rst        = 0x00;
  static constexpr uint32_t IFCR_CGIF1_Pos  = ( 0U );
  static constexpr uint32_t IFCR_CGIF1_Msk  = ( 0x1UL << IFCR_CGIF1_Pos );
  static constexpr uint32_t IFCR_CGIF1      = IFCR_CGIF1_Msk;
  static constexpr uint32_t IFCR_CTCIF1_Pos = ( 1U );
  static constexpr uint32_t IFCR_CTCIF1_Msk = ( 0x1UL << IFCR_CTCIF1_Pos );
  static constexpr uint32_t IFCR_CTCIF1     = IFCR_CTCIF1_Msk;
  static constexpr uint32_t IFCR_CHTIF1_Pos = ( 2U );
  static constexpr uint32_t IFCR_CHTIF1_Msk = ( 0x1UL << IFCR_CHTIF1_Pos );
  static constexpr uint32_t IFCR_CHTIF1     = IFCR_CHTIF1_Msk;
  static constexpr uint32_t IFCR_CTEIF1_Pos = ( 3U );
  static constexpr uint32_t IFCR_CTEIF1_Msk = ( 0x1UL << IFCR_CTEIF1_Pos );
  static constexpr uint32_t IFCR_CTEIF1     = IFCR_CTEIF1_Msk;
  static constexpr uint32_t IFCR_CGIF2_Pos  = ( 4U );
  static constexpr uint32_t IFCR_CGIF2_Msk  = ( 0x1UL << IFCR_CGIF2_Pos );
  static constexpr uint32_t IFCR_CGIF2      = IFCR_CGIF2_Msk;
  static constexpr uint32_t IFCR_CTCIF2_Pos = ( 5U );
  static constexpr uint32_t IFCR_CTCIF2_Msk = ( 0x1UL << IFCR_CTCIF2_Pos );
  static constexpr uint32_t IFCR_CTCIF2     = IFCR_CTCIF2_Msk;
  static constexpr uint32_t IFCR_CHTIF2_Pos = ( 6U );
  static constexpr uint32_t IFCR_CHTIF2_Msk = ( 0x1UL << IFCR_CHTIF2_Pos );
  static constexpr uint32_t IFCR_CHTIF2     = IFCR_CHTIF2_Msk;
  static constexpr uint32_t IFCR_CTEIF2_Pos = ( 7U );
  static constexpr uint32_t IFCR_CTEIF2_Msk = ( 0x1UL << IFCR_CTEIF2_Pos );
  static constexpr uint32_t IFCR_CTEIF2     = IFCR_CTEIF2_Msk;
  static constexpr uint32_t IFCR_CGIF3_Pos  = ( 8U );
  static constexpr uint32_t IFCR_CGIF3_Msk  = ( 0x1UL << IFCR_CGIF3_Pos );
  static constexpr uint32_t IFCR_CGIF3      = IFCR_CGIF3_Msk;
  static constexpr uint32_t IFCR_CTCIF3_Pos = ( 9U );
  static constexpr uint32_t IFCR_CTCIF3_Msk = ( 0x1UL << IFCR_CTCIF3_Pos );
  static constexpr uint32_t IFCR_CTCIF3     = IFCR_CTCIF3_Msk;
  static constexpr uint32_t IFCR_CHTIF3_Pos = ( 10U );
  static constexpr uint32_t IFCR_CHTIF3_Msk = ( 0x1UL << IFCR_CHTIF3_Pos );
  static constexpr uint32_t IFCR_CHTIF3     = IFCR_CHTIF3_Msk;
  static constexpr uint32_t IFCR_CTEIF3_Pos = ( 11U );
  static constexpr uint32_t IFCR_CTEIF3_Msk = ( 0x1UL << IFCR_CTEIF3_Pos );
  static constexpr uint32_t IFCR_CTEIF3     = IFCR_CTEIF3_Msk;
  static constexpr uint32_t IFCR_CGIF4_Pos  = ( 12U );
  static constexpr uint32_t IFCR_CGIF4_Msk  = ( 0x1UL << IFCR_CGIF4_Pos );
  static constexpr uint32_t IFCR_CGIF4      = IFCR_CGIF4_Msk;
  static constexpr uint32_t IFCR_CTCIF4_Pos = ( 13U );
  static constexpr uint32_t IFCR_CTCIF4_Msk = ( 0x1UL << IFCR_CTCIF4_Pos );
  static constexpr uint32_t IFCR_CTCIF4     = IFCR_CTCIF4_Msk;
  static constexpr uint32_t IFCR_CHTIF4_Pos = ( 14U );
  static constexpr uint32_t IFCR_CHTIF4_Msk = ( 0x1UL << IFCR_CHTIF4_Pos );
  static constexpr uint32_t IFCR_CHTIF4     = IFCR_CHTIF4_Msk;
  static constexpr uint32_t IFCR_CTEIF4_Pos = ( 15U );
  static constexpr uint32_t IFCR_CTEIF4_Msk = ( 0x1UL << IFCR_CTEIF4_Pos );
  static constexpr uint32_t IFCR_CTEIF4     = IFCR_CTEIF4_Msk;
  static constexpr uint32_t IFCR_CGIF5_Pos  = ( 16U );
  static constexpr uint32_t IFCR_CGIF5_Msk  = ( 0x1UL << IFCR_CGIF5_Pos );
  static constexpr uint32_t IFCR_CGIF5      = IFCR_CGIF5_Msk;
  static constexpr uint32_t IFCR_CTCIF5_Pos = ( 17U );
  static constexpr uint32_t IFCR_CTCIF5_Msk = ( 0x1UL << IFCR_CTCIF5_Pos );
  static constexpr uint32_t IFCR_CTCIF5     = IFCR_CTCIF5_Msk;
  static constexpr uint32_t IFCR_CHTIF5_Pos = ( 18U );
  static constexpr uint32_t IFCR_CHTIF5_Msk = ( 0x1UL << IFCR_CHTIF5_Pos );
  static constexpr uint32_t IFCR_CHTIF5     = IFCR_CHTIF5_Msk;
  static constexpr uint32_t IFCR_CTEIF5_Pos = ( 19U );
  static constexpr uint32_t IFCR_CTEIF5_Msk = ( 0x1UL << IFCR_CTEIF5_Pos );
  static constexpr uint32_t IFCR_CTEIF5     = IFCR_CTEIF5_Msk;
  static constexpr uint32_t IFCR_CGIF6_Pos  = ( 20U );
  static constexpr uint32_t IFCR_CGIF6_Msk  = ( 0x1UL << IFCR_CGIF6_Pos );
  static constexpr uint32_t IFCR_CGIF6      = IFCR_CGIF6_Msk;
  static constexpr uint32_t IFCR_CTCIF6_Pos = ( 21U );
  static constexpr uint32_t IFCR_CTCIF6_Msk = ( 0x1UL << IFCR_CTCIF6_Pos );
  static constexpr uint32_t IFCR_CTCIF6     = IFCR_CTCIF6_Msk;
  static constexpr uint32_t IFCR_CHTIF6_Pos = ( 22U );
  static constexpr uint32_t IFCR_CHTIF6_Msk = ( 0x1UL << IFCR_CHTIF6_Pos );
  static constexpr uint32_t IFCR_CHTIF6     = IFCR_CHTIF6_Msk;
  static constexpr uint32_t IFCR_CTEIF6_Pos = ( 23U );
  static constexpr uint32_t IFCR_CTEIF6_Msk = ( 0x1UL << IFCR_CTEIF6_Pos );
  static constexpr uint32_t IFCR_CTEIF6     = IFCR_CTEIF6_Msk;
  static constexpr uint32_t IFCR_CGIF7_Pos  = ( 24U );
  static constexpr uint32_t IFCR_CGIF7_Msk  = ( 0x1UL << IFCR_CGIF7_Pos );
  static constexpr uint32_t IFCR_CGIF7      = IFCR_CGIF7_Msk;
  static constexpr uint32_t IFCR_CTCIF7_Pos = ( 25U );
  static constexpr uint32_t IFCR_CTCIF7_Msk = ( 0x1UL << IFCR_CTCIF7_Pos );
  static constexpr uint32_t IFCR_CTCIF7     = IFCR_CTCIF7_Msk;
  static constexpr uint32_t IFCR_CHTIF7_Pos = ( 26U );
  static constexpr uint32_t IFCR_CHTIF7_Msk = ( 0x1UL << IFCR_CHTIF7_Pos );
  static constexpr uint32_t IFCR_CHTIF7     = IFCR_CHTIF7_Msk;
  static constexpr uint32_t IFCR_CTEIF7_Pos = ( 27U );
  static constexpr uint32_t IFCR_CTEIF7_Msk = ( 0x1UL << IFCR_CTEIF7_Pos );
  static constexpr uint32_t IFCR_CTEIF7     = IFCR_CTEIF7_Msk;

  /*---------------------------------------------------------------------------
  Channel Configuration Register
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t CCR_Msk         = 0x00007FFF;
  static constexpr uint32_t CCR_Rst         = 0x00;
  static constexpr uint32_t CCR_EN_Pos      = ( 0U );
  static constexpr uint32_t CCR_EN_Msk      = ( 0x1UL << CCR_EN_Pos );
  static constexpr uint32_t CCR_EN          = CCR_EN_Msk;
  static constexpr uint32_t CCR_TCIE_Pos    = ( 1U );
  static constexpr uint32_t CCR_TCIE_Msk    = ( 0x1UL << CCR_TCIE_Pos );
  static constexpr uint32_t CCR_TCIE        = CCR_TCIE_Msk;
  static constexpr uint32_t CCR_HTIE_Pos    = ( 2U );
  static constexpr uint32_t CCR_HTIE_Msk    = ( 0x1UL << CCR_HTIE_Pos );
  static constexpr uint32_t CCR_HTIE        = CCR_HTIE_Msk;
  static constexpr uint32_t CCR_TEIE_Pos    = ( 3U );
  static constexpr uint32_t CCR_TEIE_Msk    = ( 0x1UL << CCR_TEIE_Pos );
  static constexpr uint32_t CCR_TEIE        = CCR_TEIE_Msk;
  static constexpr uint32_t CCR_DIR_Pos     = ( 4U );
  static constexpr uint32_t CCR_DIR_Msk     = ( 0x1UL << CCR_DIR_Pos );
  static constexpr uint32_t CCR_DIR         = CCR_DIR_Msk;
  static constexpr uint32_t CCR_CIRC_Pos    = ( 5U );
  static constexpr uint32_t CCR_CIRC_Msk    = ( 0x1UL << CCR_CIRC_Pos );
  static constexpr uint32_t CCR_CIRC        = CCR_CIRC_Msk;
  static constexpr uint32_t CCR_PINC_Pos    = ( 6U );
  static constexpr uint32_t CCR_PINC_Msk    = ( 0x1UL << CCR_PINC_Pos );
  static constexpr uint32_t CCR_PINC        = CCR_PINC_Msk;
  static constexpr uint32_t CCR_MINC_Pos    = ( 7U );
  static constexpr uint32_t CCR_MINC_Msk    = ( 0x1UL << CCR_MINC_Pos );
  static constexpr uint32_t CCR_MINC        = CCR_MINC_Msk;
  static constexpr uint32_t CCR_PSIZE_Pos   = ( 8U );
  static constexpr uint32_t CCR_PSIZE_Msk   = ( 0x3UL << CCR_PSIZE_Pos );
  static constexpr uint32_t CCR_PSIZE       = CCR_PSIZE_Msk;
  static constexpr uint32_t CCR_PSIZE_0     = ( 0x1UL << CCR_PSIZE_Pos );
  static constexpr uint32_t CCR_PSIZE_1     = ( 0x2UL << CCR_PSIZE_Pos );
  static constexpr uint32_t CCR_MSIZE_Pos   = ( 10U );
  static constexpr uint32_t CCR_MSIZE_Msk   = ( 0x3UL << CCR_MSIZE_Pos );
  static constexpr uint32_t CCR_MSIZE       = CCR_MSIZE_Msk;
  static constexpr uint32_t CCR_MSIZE_0     = ( 0x1UL << CCR_MSIZE_Pos );
  static constexpr uint32_t CCR_MSIZE_1     = ( 0x2UL << CCR_MSIZE_Pos );
  static constexpr uint32_t CCR_PL_Pos      = ( 12U );
  static constexpr uint32_t CCR_PL_Msk      = ( 0x3UL << CCR_PL_Pos );
  static constexpr uint32_t CCR_PL          = CCR_PL_Msk;
  static constexpr uint32_t CCR_PL_0        = ( 0x1UL << CCR_PL_Pos );
  static constexpr uint32_t CCR_PL_1        = ( 0x2UL << CCR_PL_Pos );
  static constexpr uint32_t CCR_MEM2MEM_Pos = ( 14U );
  static constexpr uint32_t CCR_MEM2MEM_Msk = ( 0x1UL << CCR_MEM2MEM_Pos );
  static constexpr uint32_t CCR_MEM2MEM     = CCR_MEM2MEM_Msk;

  /*---------------------------------------------------------------------------
  Channel Number of Data to Transfer Register
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t CNDTR_Msk     = 0x0000FFFF;
  static constexpr uint32_t CNDTR_Rst     = 0x00000000;
  static constexpr uint32_t CNDTR_NDT_Pos = ( 0U );
  static constexpr uint32_t CNDTR_NDT_Msk = ( 0xFFFFUL << CNDTR_NDT_Pos );
  static constexpr uint32_t CNDTR_NDT     = CNDTR_NDT_Msk;

  /*---------------------------------------------------------------------------
  Channel Peripheral Address Register
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t CPAR_Msk    = 0xFFFFFFFF;
  static constexpr uint32_t CPAR_Rst    = 0x00000000;
  static constexpr uint32_t CPAR_PA_Pos = ( 0U );
  static constexpr uint32_t CPAR_PA_Msk = ( 0xFFFFFFFFUL << CPAR_PA_Pos );
  static constexpr uint32_t CPAR_PA     = CPAR_PA_Msk;

  /*---------------------------------------------------------------------------
  Channel Memory Address Register
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t CMAR_Msk    = 0xFFFFFFFF;
  static constexpr uint32_t CMAR_Rst    = 0x00000000;
  static constexpr uint32_t CMAR_MA_Pos = ( 0U );
  static constexpr uint32_t CMAR_MA_Msk = ( 0xFFFFFFFFUL << CMAR_MA_Pos );
  static constexpr uint32_t CMAR_MA     = CMAR_MA_Msk;

  /*---------------------------------------------------------------------------
  Channel Selection Register
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t CSELR_Msk     = 0x0FFFFFFF;
  static constexpr uint32_t CSELR_Rst     = 0x00000000;
  static constexpr uint32_t CSELR_C1S_Pos = ( 0U );
  static constexpr uint32_t CSELR_C1S_Msk = ( 0xFUL << CSELR_C1S_Pos );
  static constexpr uint32_t CSELR_C1S     = CSELR_C1S_Msk;
  static constexpr uint32_t CSELR_C2S_Pos = ( 4U );
  static constexpr uint32_t CSELR_C2S_Msk = ( 0xFUL << CSELR_C2S_Pos );
  static constexpr uint32_t CSELR_C2S     = CSELR_C2S_Msk;
  static constexpr uint32_t CSELR_C3S_Pos = ( 8U );
  static constexpr uint32_t CSELR_C3S_Msk = ( 0xFUL << CSELR_C3S_Pos );
  static constexpr uint32_t CSELR_C3S     = CSELR_C3S_Msk;
  static constexpr uint32_t CSELR_C4S_Pos = ( 12U );
  static constexpr uint32_t CSELR_C4S_Msk = ( 0xFUL << CSELR_C4S_Pos );
  static constexpr uint32_t CSELR_C4S     = CSELR_C4S_Msk;
  static constexpr uint32_t CSELR_C5S_Pos = ( 16U );
  static constexpr uint32_t CSELR_C5S_Msk = ( 0xFUL << CSELR_C5S_Pos );
  static constexpr uint32_t CSELR_C5S     = CSELR_C5S_Msk;
  static constexpr uint32_t CSELR_C6S_Pos = ( 20U );
  static constexpr uint32_t CSELR_C6S_Msk = ( 0xFUL << CSELR_C6S_Pos );
  static constexpr uint32_t CSELR_C6S     = CSELR_C6S_Msk;
  static constexpr uint32_t CSELR_C7S_Pos = ( 24U );
  static constexpr uint32_t CSELR_C7S_Msk = ( 0xFUL << CSELR_C7S_Pos );
  static constexpr uint32_t CSELR_C7S     = CSELR_C7S_Msk;

}    // namespace Thor::LLD::DMA

#endif  /* THOR_HW_DMA_REGISTER_STM32L4XXX_HPP */
