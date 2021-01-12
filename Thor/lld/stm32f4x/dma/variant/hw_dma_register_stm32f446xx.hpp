/********************************************************************************
 *   File Name:
 *    hw_dma_register_stm32f446xx.hpp
 *
 *   Description:
 *    Explicit hardware register definitions for the STM32F446xx DMA peripheral
 *
 *   2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_REGISTER_HPP
#define THOR_HW_DMA_REGISTER_HPP

/* C++ Includes */
#include <array>
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

#define STM32_DMA1_PERIPH_AVAILABLE
#define STM32_DMA1_STREAM0_AVAILABLE
#define STM32_DMA1_STREAM1_AVAILABLE
#define STM32_DMA1_STREAM2_AVAILABLE
#define STM32_DMA1_STREAM3_AVAILABLE
#define STM32_DMA1_STREAM4_AVAILABLE
#define STM32_DMA1_STREAM5_AVAILABLE
#define STM32_DMA1_STREAM6_AVAILABLE
#define STM32_DMA1_STREAM7_AVAILABLE

#define STM32_DMA2_PERIPH_AVAILABLE
#define STM32_DMA2_STREAM0_AVAILABLE
#define STM32_DMA2_STREAM1_AVAILABLE
#define STM32_DMA2_STREAM2_AVAILABLE
#define STM32_DMA2_STREAM3_AVAILABLE
#define STM32_DMA2_STREAM4_AVAILABLE
#define STM32_DMA2_STREAM5_AVAILABLE
#define STM32_DMA2_STREAM6_AVAILABLE
#define STM32_DMA2_STREAM7_AVAILABLE

namespace Thor::LLD::DMA
{


  static constexpr Reg32_t DMA1_BASE_ADDR         = Thor::System::MemoryMap::DMA1_PERIPH_START_ADDRESS;
  static constexpr Reg32_t DMA1_STREAM0_BASE_ADDR = DMA1_BASE_ADDR + 0x010u;
  static constexpr Reg32_t DMA1_STREAM1_BASE_ADDR = DMA1_BASE_ADDR + 0x028u;
  static constexpr Reg32_t DMA1_STREAM2_BASE_ADDR = DMA1_BASE_ADDR + 0x040u;
  static constexpr Reg32_t DMA1_STREAM3_BASE_ADDR = DMA1_BASE_ADDR + 0x058u;
  static constexpr Reg32_t DMA1_STREAM4_BASE_ADDR = DMA1_BASE_ADDR + 0x070u;
  static constexpr Reg32_t DMA1_STREAM5_BASE_ADDR = DMA1_BASE_ADDR + 0x088u;
  static constexpr Reg32_t DMA1_STREAM6_BASE_ADDR = DMA1_BASE_ADDR + 0x0A0u;
  static constexpr Reg32_t DMA1_STREAM7_BASE_ADDR = DMA1_BASE_ADDR + 0x0B8u;

  static constexpr Reg32_t DMA2_BASE_ADDR         = Thor::System::MemoryMap::DMA2_PERIPH_START_ADDRESS;
  static constexpr Reg32_t DMA2_STREAM0_BASE_ADDR = DMA2_BASE_ADDR + 0x010u;
  static constexpr Reg32_t DMA2_STREAM1_BASE_ADDR = DMA2_BASE_ADDR + 0x028u;
  static constexpr Reg32_t DMA2_STREAM2_BASE_ADDR = DMA2_BASE_ADDR + 0x040u;
  static constexpr Reg32_t DMA2_STREAM3_BASE_ADDR = DMA2_BASE_ADDR + 0x058u;
  static constexpr Reg32_t DMA2_STREAM4_BASE_ADDR = DMA2_BASE_ADDR + 0x070u;
  static constexpr Reg32_t DMA2_STREAM5_BASE_ADDR = DMA2_BASE_ADDR + 0x088u;
  static constexpr Reg32_t DMA2_STREAM6_BASE_ADDR = DMA2_BASE_ADDR + 0x0A0u;
  static constexpr Reg32_t DMA2_STREAM7_BASE_ADDR = DMA2_BASE_ADDR + 0x0B8u;

  static constexpr Reg32_t NUM_DMA_PERIPHS            = 2u;
  static constexpr Reg32_t NUM_DMA_STREAMS_PER_PERIPH = 8u;
  static constexpr Reg32_t NUM_DMA_STREAMS            = NUM_DMA_PERIPHS * NUM_DMA_STREAMS_PER_PERIPH;

  static constexpr std::array<Reg32_t, NUM_DMA_STREAMS> streamAddressList = {
    DMA1_STREAM0_BASE_ADDR, DMA1_STREAM1_BASE_ADDR, DMA1_STREAM2_BASE_ADDR, DMA1_STREAM3_BASE_ADDR,
    DMA1_STREAM4_BASE_ADDR, DMA1_STREAM5_BASE_ADDR, DMA1_STREAM6_BASE_ADDR, DMA1_STREAM7_BASE_ADDR,
    DMA2_STREAM0_BASE_ADDR, DMA2_STREAM1_BASE_ADDR, DMA2_STREAM2_BASE_ADDR, DMA2_STREAM3_BASE_ADDR,
    DMA2_STREAM4_BASE_ADDR, DMA2_STREAM5_BASE_ADDR, DMA2_STREAM6_BASE_ADDR, DMA2_STREAM7_BASE_ADDR
  };

  static constexpr uint8_t DMA1_RESOURCE_INDEX = 0u;
  static constexpr uint8_t DMA2_RESOURCE_INDEX = 1u;

  static constexpr uint8_t DMA1_RESOURCE_INDEX_START   = 0u;
  static constexpr uint8_t DMA1_STREAM0_RESOURCE_INDEX = DMA1_RESOURCE_INDEX_START;
  static constexpr uint8_t DMA1_STREAM1_RESOURCE_INDEX = 1u;
  static constexpr uint8_t DMA1_STREAM2_RESOURCE_INDEX = 2u;
  static constexpr uint8_t DMA1_STREAM3_RESOURCE_INDEX = 3u;
  static constexpr uint8_t DMA1_STREAM4_RESOURCE_INDEX = 4u;
  static constexpr uint8_t DMA1_STREAM5_RESOURCE_INDEX = 5u;
  static constexpr uint8_t DMA1_STREAM6_RESOURCE_INDEX = 6u;
  static constexpr uint8_t DMA1_STREAM7_RESOURCE_INDEX = 7u;
  static constexpr uint8_t DMA1_RESOURCE_INDEX_END     = DMA1_STREAM7_RESOURCE_INDEX;
  static constexpr uint8_t DMA2_RESOURCE_INDEX_START   = DMA1_RESOURCE_INDEX_END + 1u;
  static constexpr uint8_t DMA2_STREAM0_RESOURCE_INDEX = DMA2_RESOURCE_INDEX_START;
  static constexpr uint8_t DMA2_STREAM1_RESOURCE_INDEX = 9u;
  static constexpr uint8_t DMA2_STREAM2_RESOURCE_INDEX = 10u;
  static constexpr uint8_t DMA2_STREAM3_RESOURCE_INDEX = 11u;
  static constexpr uint8_t DMA2_STREAM4_RESOURCE_INDEX = 12u;
  static constexpr uint8_t DMA2_STREAM5_RESOURCE_INDEX = 13u;
  static constexpr uint8_t DMA2_STREAM6_RESOURCE_INDEX = 14u;
  static constexpr uint8_t DMA2_STREAM7_RESOURCE_INDEX = 15u;
  static constexpr uint8_t DMA2_RESOURCE_INDEX_END     = DMA2_STREAM7_RESOURCE_INDEX;

  static constexpr uint8_t DMA_RESOURCE_INDEX_START = DMA1_RESOURCE_INDEX_START;
  static constexpr uint8_t DMA_RESOURCE_INDEX_END   = DMA2_RESOURCE_INDEX_END;


  /*------------------------------------------------
  Low Interrupt Status Register
  ------------------------------------------------*/
  static constexpr Reg32_t LISR_Msk        = 0x0F7D0F7D;
  static constexpr Reg32_t LISR_Rst        = 0u;
  static constexpr Reg32_t LISR_TCIF3_Pos  = ( 27U );
  static constexpr Reg32_t LISR_TCIF3_Msk  = ( 0x1U << LISR_TCIF3_Pos );
  static constexpr Reg32_t LISR_TCIF3      = LISR_TCIF3_Msk;
  static constexpr Reg32_t LISR_HTIF3_Pos  = ( 26U );
  static constexpr Reg32_t LISR_HTIF3_Msk  = ( 0x1U << LISR_HTIF3_Pos );
  static constexpr Reg32_t LISR_HTIF3      = LISR_HTIF3_Msk;
  static constexpr Reg32_t LISR_TEIF3_Pos  = ( 25U );
  static constexpr Reg32_t LISR_TEIF3_Msk  = ( 0x1U << LISR_TEIF3_Pos );
  static constexpr Reg32_t LISR_TEIF3      = LISR_TEIF3_Msk;
  static constexpr Reg32_t LISR_DMEIF3_Pos = ( 24U );
  static constexpr Reg32_t LISR_DMEIF3_Msk = ( 0x1U << LISR_DMEIF3_Pos );
  static constexpr Reg32_t LISR_DMEIF3     = LISR_DMEIF3_Msk;
  static constexpr Reg32_t LISR_FEIF3_Pos  = ( 22U );
  static constexpr Reg32_t LISR_FEIF3_Msk  = ( 0x1U << LISR_FEIF3_Pos );
  static constexpr Reg32_t LISR_FEIF3      = LISR_FEIF3_Msk;
  static constexpr Reg32_t LISR_TCIF2_Pos  = ( 21U );
  static constexpr Reg32_t LISR_TCIF2_Msk  = ( 0x1U << LISR_TCIF2_Pos );
  static constexpr Reg32_t LISR_TCIF2      = LISR_TCIF2_Msk;
  static constexpr Reg32_t LISR_HTIF2_Pos  = ( 20U );
  static constexpr Reg32_t LISR_HTIF2_Msk  = ( 0x1U << LISR_HTIF2_Pos );
  static constexpr Reg32_t LISR_HTIF2      = LISR_HTIF2_Msk;
  static constexpr Reg32_t LISR_TEIF2_Pos  = ( 19U );
  static constexpr Reg32_t LISR_TEIF2_Msk  = ( 0x1U << LISR_TEIF2_Pos );
  static constexpr Reg32_t LISR_TEIF2      = LISR_TEIF2_Msk;
  static constexpr Reg32_t LISR_DMEIF2_Pos = ( 18U );
  static constexpr Reg32_t LISR_DMEIF2_Msk = ( 0x1U << LISR_DMEIF2_Pos );
  static constexpr Reg32_t LISR_DMEIF2     = LISR_DMEIF2_Msk;
  static constexpr Reg32_t LISR_FEIF2_Pos  = ( 16U );
  static constexpr Reg32_t LISR_FEIF2_Msk  = ( 0x1U << LISR_FEIF2_Pos );
  static constexpr Reg32_t LISR_FEIF2      = LISR_FEIF2_Msk;
  static constexpr Reg32_t LISR_TCIF1_Pos  = ( 11U );
  static constexpr Reg32_t LISR_TCIF1_Msk  = ( 0x1U << LISR_TCIF1_Pos );
  static constexpr Reg32_t LISR_TCIF1      = LISR_TCIF1_Msk;
  static constexpr Reg32_t LISR_HTIF1_Pos  = ( 10U );
  static constexpr Reg32_t LISR_HTIF1_Msk  = ( 0x1U << LISR_HTIF1_Pos );
  static constexpr Reg32_t LISR_HTIF1      = LISR_HTIF1_Msk;
  static constexpr Reg32_t LISR_TEIF1_Pos  = ( 9U );
  static constexpr Reg32_t LISR_TEIF1_Msk  = ( 0x1U << LISR_TEIF1_Pos );
  static constexpr Reg32_t LISR_TEIF1      = LISR_TEIF1_Msk;
  static constexpr Reg32_t LISR_DMEIF1_Pos = ( 8U );
  static constexpr Reg32_t LISR_DMEIF1_Msk = ( 0x1U << LISR_DMEIF1_Pos );
  static constexpr Reg32_t LISR_DMEIF1     = LISR_DMEIF1_Msk;
  static constexpr Reg32_t LISR_FEIF1_Pos  = ( 6U );
  static constexpr Reg32_t LISR_FEIF1_Msk  = ( 0x1U << LISR_FEIF1_Pos );
  static constexpr Reg32_t LISR_FEIF1      = LISR_FEIF1_Msk;
  static constexpr Reg32_t LISR_TCIF0_Pos  = ( 5U );
  static constexpr Reg32_t LISR_TCIF0_Msk  = ( 0x1U << LISR_TCIF0_Pos );
  static constexpr Reg32_t LISR_TCIF0      = LISR_TCIF0_Msk;
  static constexpr Reg32_t LISR_HTIF0_Pos  = ( 4U );
  static constexpr Reg32_t LISR_HTIF0_Msk  = ( 0x1U << LISR_HTIF0_Pos );
  static constexpr Reg32_t LISR_HTIF0      = LISR_HTIF0_Msk;
  static constexpr Reg32_t LISR_TEIF0_Pos  = ( 3U );
  static constexpr Reg32_t LISR_TEIF0_Msk  = ( 0x1U << LISR_TEIF0_Pos );
  static constexpr Reg32_t LISR_TEIF0      = LISR_TEIF0_Msk;
  static constexpr Reg32_t LISR_DMEIF0_Pos = ( 2U );
  static constexpr Reg32_t LISR_DMEIF0_Msk = ( 0x1U << LISR_DMEIF0_Pos );
  static constexpr Reg32_t LISR_DMEIF0     = LISR_DMEIF0_Msk;
  static constexpr Reg32_t LISR_FEIF0_Pos  = ( 0U );
  static constexpr Reg32_t LISR_FEIF0_Msk  = ( 0x1U << LISR_FEIF0_Pos );
  static constexpr Reg32_t LISR_FEIF0      = LISR_FEIF0_Msk;

  /*------------------------------------------------
  High Interrupt Status Register
  ------------------------------------------------*/
  static constexpr Reg32_t HISR_Msk        = 0x0F7D0F7D;
  static constexpr Reg32_t HISR_Rst        = 0u;
  static constexpr Reg32_t HISR_TCIF7_Pos  = ( 27U );
  static constexpr Reg32_t HISR_TCIF7_Msk  = ( 0x1U << HISR_TCIF7_Pos );
  static constexpr Reg32_t HISR_TCIF7      = HISR_TCIF7_Msk;
  static constexpr Reg32_t HISR_HTIF7_Pos  = ( 26U );
  static constexpr Reg32_t HISR_HTIF7_Msk  = ( 0x1U << HISR_HTIF7_Pos );
  static constexpr Reg32_t HISR_HTIF7      = HISR_HTIF7_Msk;
  static constexpr Reg32_t HISR_TEIF7_Pos  = ( 25U );
  static constexpr Reg32_t HISR_TEIF7_Msk  = ( 0x1U << HISR_TEIF7_Pos );
  static constexpr Reg32_t HISR_TEIF7      = HISR_TEIF7_Msk;
  static constexpr Reg32_t HISR_DMEIF7_Pos = ( 24U );
  static constexpr Reg32_t HISR_DMEIF7_Msk = ( 0x1U << HISR_DMEIF7_Pos );
  static constexpr Reg32_t HISR_DMEIF7     = HISR_DMEIF7_Msk;
  static constexpr Reg32_t HISR_FEIF7_Pos  = ( 22U );
  static constexpr Reg32_t HISR_FEIF7_Msk  = ( 0x1U << HISR_FEIF7_Pos );
  static constexpr Reg32_t HISR_FEIF7      = HISR_FEIF7_Msk;
  static constexpr Reg32_t HISR_TCIF6_Pos  = ( 21U );
  static constexpr Reg32_t HISR_TCIF6_Msk  = ( 0x1U << HISR_TCIF6_Pos );
  static constexpr Reg32_t HISR_TCIF6      = HISR_TCIF6_Msk;
  static constexpr Reg32_t HISR_HTIF6_Pos  = ( 20U );
  static constexpr Reg32_t HISR_HTIF6_Msk  = ( 0x1U << HISR_HTIF6_Pos );
  static constexpr Reg32_t HISR_HTIF6      = HISR_HTIF6_Msk;
  static constexpr Reg32_t HISR_TEIF6_Pos  = ( 19U );
  static constexpr Reg32_t HISR_TEIF6_Msk  = ( 0x1U << HISR_TEIF6_Pos );
  static constexpr Reg32_t HISR_TEIF6      = HISR_TEIF6_Msk;
  static constexpr Reg32_t HISR_DMEIF6_Pos = ( 18U );
  static constexpr Reg32_t HISR_DMEIF6_Msk = ( 0x1U << HISR_DMEIF6_Pos );
  static constexpr Reg32_t HISR_DMEIF6     = HISR_DMEIF6_Msk;
  static constexpr Reg32_t HISR_FEIF6_Pos  = ( 16U );
  static constexpr Reg32_t HISR_FEIF6_Msk  = ( 0x1U << HISR_FEIF6_Pos );
  static constexpr Reg32_t HISR_FEIF6      = HISR_FEIF6_Msk;
  static constexpr Reg32_t HISR_TCIF5_Pos  = ( 11U );
  static constexpr Reg32_t HISR_TCIF5_Msk  = ( 0x1U << HISR_TCIF5_Pos );
  static constexpr Reg32_t HISR_TCIF5      = HISR_TCIF5_Msk;
  static constexpr Reg32_t HISR_HTIF5_Pos  = ( 10U );
  static constexpr Reg32_t HISR_HTIF5_Msk  = ( 0x1U << HISR_HTIF5_Pos );
  static constexpr Reg32_t HISR_HTIF5      = HISR_HTIF5_Msk;
  static constexpr Reg32_t HISR_TEIF5_Pos  = ( 9U );
  static constexpr Reg32_t HISR_TEIF5_Msk  = ( 0x1U << HISR_TEIF5_Pos );
  static constexpr Reg32_t HISR_TEIF5      = HISR_TEIF5_Msk;
  static constexpr Reg32_t HISR_DMEIF5_Pos = ( 8U );
  static constexpr Reg32_t HISR_DMEIF5_Msk = ( 0x1U << HISR_DMEIF5_Pos );
  static constexpr Reg32_t HISR_DMEIF5     = HISR_DMEIF5_Msk;
  static constexpr Reg32_t HISR_FEIF5_Pos  = ( 6U );
  static constexpr Reg32_t HISR_FEIF5_Msk  = ( 0x1U << HISR_FEIF5_Pos );
  static constexpr Reg32_t HISR_FEIF5      = HISR_FEIF5_Msk;
  static constexpr Reg32_t HISR_TCIF4_Pos  = ( 5U );
  static constexpr Reg32_t HISR_TCIF4_Msk  = ( 0x1U << HISR_TCIF4_Pos );
  static constexpr Reg32_t HISR_TCIF4      = HISR_TCIF4_Msk;
  static constexpr Reg32_t HISR_HTIF4_Pos  = ( 4U );
  static constexpr Reg32_t HISR_HTIF4_Msk  = ( 0x1U << HISR_HTIF4_Pos );
  static constexpr Reg32_t HISR_HTIF4      = HISR_HTIF4_Msk;
  static constexpr Reg32_t HISR_TEIF4_Pos  = ( 3U );
  static constexpr Reg32_t HISR_TEIF4_Msk  = ( 0x1U << HISR_TEIF4_Pos );
  static constexpr Reg32_t HISR_TEIF4      = HISR_TEIF4_Msk;
  static constexpr Reg32_t HISR_DMEIF4_Pos = ( 2U );
  static constexpr Reg32_t HISR_DMEIF4_Msk = ( 0x1U << HISR_DMEIF4_Pos );
  static constexpr Reg32_t HISR_DMEIF4     = HISR_DMEIF4_Msk;
  static constexpr Reg32_t HISR_FEIF4_Pos  = ( 0U );
  static constexpr Reg32_t HISR_FEIF4_Msk  = ( 0x1U << HISR_FEIF4_Pos );
  static constexpr Reg32_t HISR_FEIF4      = HISR_FEIF4_Msk;

  /*------------------------------------------------
  Low Interrupt Flag Clear Register
  ------------------------------------------------*/
  static constexpr Reg32_t LIFCR_Msk         = 0x0F7D0F7D;
  static constexpr Reg32_t LIFCR_Rst         = 0u;
  static constexpr Reg32_t LIFCR_CTCIF3_Pos  = ( 27U );
  static constexpr Reg32_t LIFCR_CTCIF3_Msk  = ( 0x1U << LIFCR_CTCIF3_Pos );
  static constexpr Reg32_t LIFCR_CTCIF3      = LIFCR_CTCIF3_Msk;
  static constexpr Reg32_t LIFCR_CHTIF3_Pos  = ( 26U );
  static constexpr Reg32_t LIFCR_CHTIF3_Msk  = ( 0x1U << LIFCR_CHTIF3_Pos );
  static constexpr Reg32_t LIFCR_CHTIF3      = LIFCR_CHTIF3_Msk;
  static constexpr Reg32_t LIFCR_CTEIF3_Pos  = ( 25U );
  static constexpr Reg32_t LIFCR_CTEIF3_Msk  = ( 0x1U << LIFCR_CTEIF3_Pos );
  static constexpr Reg32_t LIFCR_CTEIF3      = LIFCR_CTEIF3_Msk;
  static constexpr Reg32_t LIFCR_CDMEIF3_Pos = ( 24U );
  static constexpr Reg32_t LIFCR_CDMEIF3_Msk = ( 0x1U << LIFCR_CDMEIF3_Pos );
  static constexpr Reg32_t LIFCR_CDMEIF3     = LIFCR_CDMEIF3_Msk;
  static constexpr Reg32_t LIFCR_CFEIF3_Pos  = ( 22U );
  static constexpr Reg32_t LIFCR_CFEIF3_Msk  = ( 0x1U << LIFCR_CFEIF3_Pos );
  static constexpr Reg32_t LIFCR_CFEIF3      = LIFCR_CFEIF3_Msk;
  static constexpr Reg32_t LIFCR_CTCIF2_Pos  = ( 21U );
  static constexpr Reg32_t LIFCR_CTCIF2_Msk  = ( 0x1U << LIFCR_CTCIF2_Pos );
  static constexpr Reg32_t LIFCR_CTCIF2      = LIFCR_CTCIF2_Msk;
  static constexpr Reg32_t LIFCR_CHTIF2_Pos  = ( 20U );
  static constexpr Reg32_t LIFCR_CHTIF2_Msk  = ( 0x1U << LIFCR_CHTIF2_Pos );
  static constexpr Reg32_t LIFCR_CHTIF2      = LIFCR_CHTIF2_Msk;
  static constexpr Reg32_t LIFCR_CTEIF2_Pos  = ( 19U );
  static constexpr Reg32_t LIFCR_CTEIF2_Msk  = ( 0x1U << LIFCR_CTEIF2_Pos );
  static constexpr Reg32_t LIFCR_CTEIF2      = LIFCR_CTEIF2_Msk;
  static constexpr Reg32_t LIFCR_CDMEIF2_Pos = ( 18U );
  static constexpr Reg32_t LIFCR_CDMEIF2_Msk = ( 0x1U << LIFCR_CDMEIF2_Pos );
  static constexpr Reg32_t LIFCR_CDMEIF2     = LIFCR_CDMEIF2_Msk;
  static constexpr Reg32_t LIFCR_CFEIF2_Pos  = ( 16U );
  static constexpr Reg32_t LIFCR_CFEIF2_Msk  = ( 0x1U << LIFCR_CFEIF2_Pos );
  static constexpr Reg32_t LIFCR_CFEIF2      = LIFCR_CFEIF2_Msk;
  static constexpr Reg32_t LIFCR_CTCIF1_Pos  = ( 11U );
  static constexpr Reg32_t LIFCR_CTCIF1_Msk  = ( 0x1U << LIFCR_CTCIF1_Pos );
  static constexpr Reg32_t LIFCR_CTCIF1      = LIFCR_CTCIF1_Msk;
  static constexpr Reg32_t LIFCR_CHTIF1_Pos  = ( 10U );
  static constexpr Reg32_t LIFCR_CHTIF1_Msk  = ( 0x1U << LIFCR_CHTIF1_Pos );
  static constexpr Reg32_t LIFCR_CHTIF1      = LIFCR_CHTIF1_Msk;
  static constexpr Reg32_t LIFCR_CTEIF1_Pos  = ( 9U );
  static constexpr Reg32_t LIFCR_CTEIF1_Msk  = ( 0x1U << LIFCR_CTEIF1_Pos );
  static constexpr Reg32_t LIFCR_CTEIF1      = LIFCR_CTEIF1_Msk;
  static constexpr Reg32_t LIFCR_CDMEIF1_Pos = ( 8U );
  static constexpr Reg32_t LIFCR_CDMEIF1_Msk = ( 0x1U << LIFCR_CDMEIF1_Pos );
  static constexpr Reg32_t LIFCR_CDMEIF1     = LIFCR_CDMEIF1_Msk;
  static constexpr Reg32_t LIFCR_CFEIF1_Pos  = ( 6U );
  static constexpr Reg32_t LIFCR_CFEIF1_Msk  = ( 0x1U << LIFCR_CFEIF1_Pos );
  static constexpr Reg32_t LIFCR_CFEIF1      = LIFCR_CFEIF1_Msk;
  static constexpr Reg32_t LIFCR_CTCIF0_Pos  = ( 5U );
  static constexpr Reg32_t LIFCR_CTCIF0_Msk  = ( 0x1U << LIFCR_CTCIF0_Pos );
  static constexpr Reg32_t LIFCR_CTCIF0      = LIFCR_CTCIF0_Msk;
  static constexpr Reg32_t LIFCR_CHTIF0_Pos  = ( 4U );
  static constexpr Reg32_t LIFCR_CHTIF0_Msk  = ( 0x1U << LIFCR_CHTIF0_Pos );
  static constexpr Reg32_t LIFCR_CHTIF0      = LIFCR_CHTIF0_Msk;
  static constexpr Reg32_t LIFCR_CTEIF0_Pos  = ( 3U );
  static constexpr Reg32_t LIFCR_CTEIF0_Msk  = ( 0x1U << LIFCR_CTEIF0_Pos );
  static constexpr Reg32_t LIFCR_CTEIF0      = LIFCR_CTEIF0_Msk;
  static constexpr Reg32_t LIFCR_CDMEIF0_Pos = ( 2U );
  static constexpr Reg32_t LIFCR_CDMEIF0_Msk = ( 0x1U << LIFCR_CDMEIF0_Pos );
  static constexpr Reg32_t LIFCR_CDMEIF0     = LIFCR_CDMEIF0_Msk;
  static constexpr Reg32_t LIFCR_CFEIF0_Pos  = ( 0U );
  static constexpr Reg32_t LIFCR_CFEIF0_Msk  = ( 0x1U << LIFCR_CFEIF0_Pos );
  static constexpr Reg32_t LIFCR_CFEIF0      = LIFCR_CFEIF0_Msk;

  /*------------------------------------------------
  High Interrupt Flag Clear Register
  ------------------------------------------------*/
  static constexpr Reg32_t HIFCR_Msk         = 0x0F7D0F7D;
  static constexpr Reg32_t HIFCR_Rst         = 0u;
  static constexpr Reg32_t HIFCR_CTCIF7_Pos  = ( 27U );
  static constexpr Reg32_t HIFCR_CTCIF7_Msk  = ( 0x1U << HIFCR_CTCIF7_Pos );
  static constexpr Reg32_t HIFCR_CTCIF7      = HIFCR_CTCIF7_Msk;
  static constexpr Reg32_t HIFCR_CHTIF7_Pos  = ( 26U );
  static constexpr Reg32_t HIFCR_CHTIF7_Msk  = ( 0x1U << HIFCR_CHTIF7_Pos );
  static constexpr Reg32_t HIFCR_CHTIF7      = HIFCR_CHTIF7_Msk;
  static constexpr Reg32_t HIFCR_CTEIF7_Pos  = ( 25U );
  static constexpr Reg32_t HIFCR_CTEIF7_Msk  = ( 0x1U << HIFCR_CTEIF7_Pos );
  static constexpr Reg32_t HIFCR_CTEIF7      = HIFCR_CTEIF7_Msk;
  static constexpr Reg32_t HIFCR_CDMEIF7_Pos = ( 24U );
  static constexpr Reg32_t HIFCR_CDMEIF7_Msk = ( 0x1U << HIFCR_CDMEIF7_Pos );
  static constexpr Reg32_t HIFCR_CDMEIF7     = HIFCR_CDMEIF7_Msk;
  static constexpr Reg32_t HIFCR_CFEIF7_Pos  = ( 22U );
  static constexpr Reg32_t HIFCR_CFEIF7_Msk  = ( 0x1U << HIFCR_CFEIF7_Pos );
  static constexpr Reg32_t HIFCR_CFEIF7      = HIFCR_CFEIF7_Msk;
  static constexpr Reg32_t HIFCR_CTCIF6_Pos  = ( 21U );
  static constexpr Reg32_t HIFCR_CTCIF6_Msk  = ( 0x1U << HIFCR_CTCIF6_Pos );
  static constexpr Reg32_t HIFCR_CTCIF6      = HIFCR_CTCIF6_Msk;
  static constexpr Reg32_t HIFCR_CHTIF6_Pos  = ( 20U );
  static constexpr Reg32_t HIFCR_CHTIF6_Msk  = ( 0x1U << HIFCR_CHTIF6_Pos );
  static constexpr Reg32_t HIFCR_CHTIF6      = HIFCR_CHTIF6_Msk;
  static constexpr Reg32_t HIFCR_CTEIF6_Pos  = ( 19U );
  static constexpr Reg32_t HIFCR_CTEIF6_Msk  = ( 0x1U << HIFCR_CTEIF6_Pos );
  static constexpr Reg32_t HIFCR_CTEIF6      = HIFCR_CTEIF6_Msk;
  static constexpr Reg32_t HIFCR_CDMEIF6_Pos = ( 18U );
  static constexpr Reg32_t HIFCR_CDMEIF6_Msk = ( 0x1U << HIFCR_CDMEIF6_Pos );
  static constexpr Reg32_t HIFCR_CDMEIF6     = HIFCR_CDMEIF6_Msk;
  static constexpr Reg32_t HIFCR_CFEIF6_Pos  = ( 16U );
  static constexpr Reg32_t HIFCR_CFEIF6_Msk  = ( 0x1U << HIFCR_CFEIF6_Pos );
  static constexpr Reg32_t HIFCR_CFEIF6      = HIFCR_CFEIF6_Msk;
  static constexpr Reg32_t HIFCR_CTCIF5_Pos  = ( 11U );
  static constexpr Reg32_t HIFCR_CTCIF5_Msk  = ( 0x1U << HIFCR_CTCIF5_Pos );
  static constexpr Reg32_t HIFCR_CTCIF5      = HIFCR_CTCIF5_Msk;
  static constexpr Reg32_t HIFCR_CHTIF5_Pos  = ( 10U );
  static constexpr Reg32_t HIFCR_CHTIF5_Msk  = ( 0x1U << HIFCR_CHTIF5_Pos );
  static constexpr Reg32_t HIFCR_CHTIF5      = HIFCR_CHTIF5_Msk;
  static constexpr Reg32_t HIFCR_CTEIF5_Pos  = ( 9U );
  static constexpr Reg32_t HIFCR_CTEIF5_Msk  = ( 0x1U << HIFCR_CTEIF5_Pos );
  static constexpr Reg32_t HIFCR_CTEIF5      = HIFCR_CTEIF5_Msk;
  static constexpr Reg32_t HIFCR_CDMEIF5_Pos = ( 8U );
  static constexpr Reg32_t HIFCR_CDMEIF5_Msk = ( 0x1U << HIFCR_CDMEIF5_Pos );
  static constexpr Reg32_t HIFCR_CDMEIF5     = HIFCR_CDMEIF5_Msk;
  static constexpr Reg32_t HIFCR_CFEIF5_Pos  = ( 6U );
  static constexpr Reg32_t HIFCR_CFEIF5_Msk  = ( 0x1U << HIFCR_CFEIF5_Pos );
  static constexpr Reg32_t HIFCR_CFEIF5      = HIFCR_CFEIF5_Msk;
  static constexpr Reg32_t HIFCR_CTCIF4_Pos  = ( 5U );
  static constexpr Reg32_t HIFCR_CTCIF4_Msk  = ( 0x1U << HIFCR_CTCIF4_Pos );
  static constexpr Reg32_t HIFCR_CTCIF4      = HIFCR_CTCIF4_Msk;
  static constexpr Reg32_t HIFCR_CHTIF4_Pos  = ( 4U );
  static constexpr Reg32_t HIFCR_CHTIF4_Msk  = ( 0x1U << HIFCR_CHTIF4_Pos );
  static constexpr Reg32_t HIFCR_CHTIF4      = HIFCR_CHTIF4_Msk;
  static constexpr Reg32_t HIFCR_CTEIF4_Pos  = ( 3U );
  static constexpr Reg32_t HIFCR_CTEIF4_Msk  = ( 0x1U << HIFCR_CTEIF4_Pos );
  static constexpr Reg32_t HIFCR_CTEIF4      = HIFCR_CTEIF4_Msk;
  static constexpr Reg32_t HIFCR_CDMEIF4_Pos = ( 2U );
  static constexpr Reg32_t HIFCR_CDMEIF4_Msk = ( 0x1U << HIFCR_CDMEIF4_Pos );
  static constexpr Reg32_t HIFCR_CDMEIF4     = HIFCR_CDMEIF4_Msk;
  static constexpr Reg32_t HIFCR_CFEIF4_Pos  = ( 0U );
  static constexpr Reg32_t HIFCR_CFEIF4_Msk  = ( 0x1U << HIFCR_CFEIF4_Pos );
  static constexpr Reg32_t HIFCR_CFEIF4      = HIFCR_CFEIF4_Msk;

  /*------------------------------------------------
  Stream X Configuration Register
  ------------------------------------------------*/
  static constexpr Reg32_t SxCR_Msk        = 0x0FEFFFFF;
  static constexpr Reg32_t SxCR_Rst        = 0u;
  static constexpr Reg32_t SxCR_CHSEL_Pos  = ( 25U );
  static constexpr Reg32_t SxCR_CHSEL_Msk  = ( 0x7U << SxCR_CHSEL_Pos );
  static constexpr Reg32_t SxCR_CHSEL      = SxCR_CHSEL_Msk;
  static constexpr Reg32_t SxCR_CHSEL_0    = 0x02000000U;
  static constexpr Reg32_t SxCR_CHSEL_1    = 0x04000000U;
  static constexpr Reg32_t SxCR_CHSEL_2    = 0x08000000U;
  static constexpr Reg32_t SxCR_MBURST_Pos = ( 23U );
  static constexpr Reg32_t SxCR_MBURST_Msk = ( 0x3U << SxCR_MBURST_Pos );
  static constexpr Reg32_t SxCR_MBURST     = SxCR_MBURST_Msk;
  static constexpr Reg32_t SxCR_MBURST_0   = ( 0x1U << SxCR_MBURST_Pos );
  static constexpr Reg32_t SxCR_MBURST_1   = ( 0x2U << SxCR_MBURST_Pos );
  static constexpr Reg32_t SxCR_PBURST_Pos = ( 21U );
  static constexpr Reg32_t SxCR_PBURST_Msk = ( 0x3U << SxCR_PBURST_Pos );
  static constexpr Reg32_t SxCR_PBURST     = SxCR_PBURST_Msk;
  static constexpr Reg32_t SxCR_PBURST_0   = ( 0x1U << SxCR_PBURST_Pos );
  static constexpr Reg32_t SxCR_PBURST_1   = ( 0x2U << SxCR_PBURST_Pos );
  static constexpr Reg32_t SxCR_CT_Pos     = ( 19U );
  static constexpr Reg32_t SxCR_CT_Msk     = ( 0x1U << SxCR_CT_Pos );
  static constexpr Reg32_t SxCR_CT         = SxCR_CT_Msk;
  static constexpr Reg32_t SxCR_DBM_Pos    = ( 18U );
  static constexpr Reg32_t SxCR_DBM_Msk    = ( 0x1U << SxCR_DBM_Pos );
  static constexpr Reg32_t SxCR_DBM        = SxCR_DBM_Msk;
  static constexpr Reg32_t SxCR_PL_Pos     = ( 16U );
  static constexpr Reg32_t SxCR_PL_Msk     = ( 0x3U << SxCR_PL_Pos );
  static constexpr Reg32_t SxCR_PL         = SxCR_PL_Msk;
  static constexpr Reg32_t SxCR_PL_0       = ( 0x1U << SxCR_PL_Pos );
  static constexpr Reg32_t SxCR_PL_1       = ( 0x2U << SxCR_PL_Pos );
  static constexpr Reg32_t SxCR_PINCOS_Pos = ( 15U );
  static constexpr Reg32_t SxCR_PINCOS_Msk = ( 0x1U << SxCR_PINCOS_Pos );
  static constexpr Reg32_t SxCR_PINCOS     = SxCR_PINCOS_Msk;
  static constexpr Reg32_t SxCR_MSIZE_Pos  = ( 13U );
  static constexpr Reg32_t SxCR_MSIZE_Msk  = ( 0x3U << SxCR_MSIZE_Pos );
  static constexpr Reg32_t SxCR_MSIZE      = SxCR_MSIZE_Msk;
  static constexpr Reg32_t SxCR_MSIZE_0    = ( 0x1U << SxCR_MSIZE_Pos );
  static constexpr Reg32_t SxCR_MSIZE_1    = ( 0x2U << SxCR_MSIZE_Pos );
  static constexpr Reg32_t SxCR_PSIZE_Pos  = ( 11U );
  static constexpr Reg32_t SxCR_PSIZE_Msk  = ( 0x3U << SxCR_PSIZE_Pos );
  static constexpr Reg32_t SxCR_PSIZE      = SxCR_PSIZE_Msk;
  static constexpr Reg32_t SxCR_PSIZE_0    = ( 0x1U << SxCR_PSIZE_Pos );
  static constexpr Reg32_t SxCR_PSIZE_1    = ( 0x2U << SxCR_PSIZE_Pos );
  static constexpr Reg32_t SxCR_MINC_Pos   = ( 10U );
  static constexpr Reg32_t SxCR_MINC_Msk   = ( 0x1U << SxCR_MINC_Pos );
  static constexpr Reg32_t SxCR_MINC       = SxCR_MINC_Msk;
  static constexpr Reg32_t SxCR_PINC_Pos   = ( 9U );
  static constexpr Reg32_t SxCR_PINC_Msk   = ( 0x1U << SxCR_PINC_Pos );
  static constexpr Reg32_t SxCR_PINC       = SxCR_PINC_Msk;
  static constexpr Reg32_t SxCR_CIRC_Pos   = ( 8U );
  static constexpr Reg32_t SxCR_CIRC_Msk   = ( 0x1U << SxCR_CIRC_Pos );
  static constexpr Reg32_t SxCR_CIRC       = SxCR_CIRC_Msk;
  static constexpr Reg32_t SxCR_DIR_Pos    = ( 6U );
  static constexpr Reg32_t SxCR_DIR_Msk    = ( 0x3U << SxCR_DIR_Pos );
  static constexpr Reg32_t SxCR_DIR        = SxCR_DIR_Msk;
  static constexpr Reg32_t SxCR_DIR_0      = ( 0x1U << SxCR_DIR_Pos );
  static constexpr Reg32_t SxCR_DIR_1      = ( 0x2U << SxCR_DIR_Pos );
  static constexpr Reg32_t SxCR_PFCTRL_Pos = ( 5U );
  static constexpr Reg32_t SxCR_PFCTRL_Msk = ( 0x1U << SxCR_PFCTRL_Pos );
  static constexpr Reg32_t SxCR_PFCTRL     = SxCR_PFCTRL_Msk;
  static constexpr Reg32_t SxCR_TCIE_Pos   = ( 4U );
  static constexpr Reg32_t SxCR_TCIE_Msk   = ( 0x1U << SxCR_TCIE_Pos );
  static constexpr Reg32_t SxCR_TCIE       = SxCR_TCIE_Msk;
  static constexpr Reg32_t SxCR_HTIE_Pos   = ( 3U );
  static constexpr Reg32_t SxCR_HTIE_Msk   = ( 0x1U << SxCR_HTIE_Pos );
  static constexpr Reg32_t SxCR_HTIE       = SxCR_HTIE_Msk;
  static constexpr Reg32_t SxCR_TEIE_Pos   = ( 2U );
  static constexpr Reg32_t SxCR_TEIE_Msk   = ( 0x1U << SxCR_TEIE_Pos );
  static constexpr Reg32_t SxCR_TEIE       = SxCR_TEIE_Msk;
  static constexpr Reg32_t SxCR_DMEIE_Pos  = ( 1U );
  static constexpr Reg32_t SxCR_DMEIE_Msk  = ( 0x1U << SxCR_DMEIE_Pos );
  static constexpr Reg32_t SxCR_DMEIE      = SxCR_DMEIE_Msk;
  static constexpr Reg32_t SxCR_EN_Pos     = ( 0U );
  static constexpr Reg32_t SxCR_EN_Msk     = ( 0x1U << SxCR_EN_Pos );
  static constexpr Reg32_t SxCR_EN         = SxCR_EN_Msk;

  /*------------------------------------------------
  Stream X Number of Data Register
  ------------------------------------------------*/
  static constexpr Reg32_t SxNDT_Rst = 0u;
  static constexpr Reg32_t SxNDT_Pos = ( 0U );
  static constexpr Reg32_t SxNDT_Msk = ( 0xFFFFU << SxNDT_Pos );
  static constexpr Reg32_t SxNDT     = SxNDT_Msk;
  static constexpr Reg32_t SxNDT_0   = ( 0x0001U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_1   = ( 0x0002U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_2   = ( 0x0004U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_3   = ( 0x0008U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_4   = ( 0x0010U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_5   = ( 0x0020U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_6   = ( 0x0040U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_7   = ( 0x0080U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_8   = ( 0x0100U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_9   = ( 0x0200U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_10  = ( 0x0400U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_11  = ( 0x0800U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_12  = ( 0x1000U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_13  = ( 0x2000U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_14  = ( 0x4000U << SxNDT_Pos );
  static constexpr Reg32_t SxNDT_15  = ( 0x8000U << SxNDT_Pos );

  /*------------------------------------------------
  Stream X Peripheral Address Register
  ------------------------------------------------*/
  static constexpr Reg32_t SxPAR_Msk    = 0xFFFFFFFF;
  static constexpr Reg32_t SxPAR_Rst    = 0x00000000;
  static constexpr Reg32_t SxPAR_PA_Pos = ( 0U );
  static constexpr Reg32_t SxPAR_PA_Msk = ( 0xFFFFFFFFU << SxPAR_PA_Pos );
  static constexpr Reg32_t SxPAR_PA     = SxPAR_PA_Msk;

  /*------------------------------------------------
  Stream X Memory 0 Address Register
  ------------------------------------------------*/
  static constexpr Reg32_t SxM0AR_Msk     = 0xFFFFFFFF;
  static constexpr Reg32_t SxM0AR_Rst     = 0x00000000;
  static constexpr Reg32_t SxM0AR_M0A_Pos = ( 0U );
  static constexpr Reg32_t SxM0AR_M0A_Msk = ( 0xFFFFFFFFU << SxM0AR_M0A_Pos );
  static constexpr Reg32_t SxM0AR_M0A     = SxM0AR_M0A_Msk;

  /*------------------------------------------------
  Stream X Memory 1 Address Register
  ------------------------------------------------*/
  static constexpr Reg32_t SxM1AR_Msk     = 0xFFFFFFFF;
  static constexpr Reg32_t SxM1AR_Rst     = 0x00000000;
  static constexpr Reg32_t SxM1AR_M1A_Pos = ( 0U );
  static constexpr Reg32_t SxM1AR_M1A_Msk = ( 0xFFFFFFFFU << SxM1AR_M1A_Pos );
  static constexpr Reg32_t SxM1AR_M1A     = SxM1AR_M1A_Msk;

  /*------------------------------------------------
  Stream X FIFO Control Register
  ------------------------------------------------*/
  static constexpr Reg32_t SxFCR_Msk       = 0x000000BF;
  static constexpr Reg32_t SxFCR_Rst       = 0x00000021;
  static constexpr Reg32_t SxFCR_FEIE_Pos  = ( 7U );
  static constexpr Reg32_t SxFCR_FEIE_Msk  = ( 0x1U << SxFCR_FEIE_Pos );
  static constexpr Reg32_t SxFCR_FEIE      = SxFCR_FEIE_Msk;
  static constexpr Reg32_t SxFCR_FS_Pos    = ( 3U );
  static constexpr Reg32_t SxFCR_FS_Msk    = ( 0x7U << SxFCR_FS_Pos );
  static constexpr Reg32_t SxFCR_FS        = SxFCR_FS_Msk;
  static constexpr Reg32_t SxFCR_FS_0      = ( 0x1U << SxFCR_FS_Pos );
  static constexpr Reg32_t SxFCR_FS_1      = ( 0x2U << SxFCR_FS_Pos );
  static constexpr Reg32_t SxFCR_FS_2      = ( 0x4U << SxFCR_FS_Pos );
  static constexpr Reg32_t SxFCR_DMDIS_Pos = ( 2U );
  static constexpr Reg32_t SxFCR_DMDIS_Msk = ( 0x1U << SxFCR_DMDIS_Pos );
  static constexpr Reg32_t SxFCR_DMDIS     = SxFCR_DMDIS_Msk;
  static constexpr Reg32_t SxFCR_FTH_Pos   = ( 0U );
  static constexpr Reg32_t SxFCR_FTH_Msk   = ( 0x3U << SxFCR_FTH_Pos );
  static constexpr Reg32_t SxFCR_FTH       = SxFCR_FTH_Msk;
  static constexpr Reg32_t SxFCR_FTH_0     = ( 0x1U << SxFCR_FTH_Pos );
  static constexpr Reg32_t SxFCR_FTH_1     = ( 0x2U << SxFCR_FTH_Pos );

}    // namespace Thor::LLD::DMA

#endif /* !THOR_HW_DMA_REGISTER_HPP */