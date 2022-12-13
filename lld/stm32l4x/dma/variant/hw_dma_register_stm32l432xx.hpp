/******************************************************************************
 *   File Name:
 *    hw_dma_register_stm32l432xx.hpp
 *
 *   Description:
 *    Explicit hardware register definitions for the STM32L432xx DMA peripheral
 *
 *   2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_REGISTER_STM32L432XX_HPP
#define THOR_HW_DMA_REGISTER_STM32L432XX_HPP

/* C++ Includes */
#include <cstddef>

#define STM32_DMA1_PERIPH_AVAILABLE
#define STM32_DMA1_STREAM1_AVAILABLE
#define STM32_DMA1_STREAM2_AVAILABLE
#define STM32_DMA1_STREAM3_AVAILABLE
#define STM32_DMA1_STREAM4_AVAILABLE
#define STM32_DMA1_STREAM5_AVAILABLE
#define STM32_DMA1_STREAM6_AVAILABLE
#define STM32_DMA1_STREAM7_AVAILABLE

#define STM32_DMA2_PERIPH_AVAILABLE
#define STM32_DMA2_STREAM1_AVAILABLE
#define STM32_DMA2_STREAM2_AVAILABLE
#define STM32_DMA2_STREAM3_AVAILABLE
#define STM32_DMA2_STREAM4_AVAILABLE
#define STM32_DMA2_STREAM5_AVAILABLE
#define STM32_DMA2_STREAM6_AVAILABLE
#define STM32_DMA2_STREAM7_AVAILABLE

namespace Thor::LLD::DMA
{
  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_DMA_PERIPHS             = 2u;
  static constexpr size_t NUM_DMA_SOURCES             = 72;
  static constexpr size_t NUM_DMA_STREAMS_PER_PERIPH  = 7u;
  static constexpr size_t NUM_DMA_CHANNELS_PER_STREAM = 8u;

  static constexpr size_t DMA1_RESOURCE_INDEX         = 0u;
  static constexpr size_t DMA2_RESOURCE_INDEX         = 1u;

  static constexpr size_t DMA1_STREAM1_RESOURCE_INDEX = 0u;
  static constexpr size_t DMA1_STREAM2_RESOURCE_INDEX = 1u;
  static constexpr size_t DMA1_STREAM3_RESOURCE_INDEX = 2u;
  static constexpr size_t DMA1_STREAM4_RESOURCE_INDEX = 3u;
  static constexpr size_t DMA1_STREAM5_RESOURCE_INDEX = 4u;
  static constexpr size_t DMA1_STREAM6_RESOURCE_INDEX = 5u;
  static constexpr size_t DMA1_STREAM7_RESOURCE_INDEX = 6u;

  static constexpr size_t DMA2_STREAM1_RESOURCE_INDEX = 7u;
  static constexpr size_t DMA2_STREAM2_RESOURCE_INDEX = 8u;
  static constexpr size_t DMA2_STREAM3_RESOURCE_INDEX = 9u;
  static constexpr size_t DMA2_STREAM4_RESOURCE_INDEX = 10u;
  static constexpr size_t DMA2_STREAM5_RESOURCE_INDEX = 11u;
  static constexpr size_t DMA2_STREAM6_RESOURCE_INDEX = 12u;
  static constexpr size_t DMA2_STREAM7_RESOURCE_INDEX = 13u;

  static constexpr size_t DMA_MAX_RESOURCE_IDX        = 14u;

  static constexpr size_t DMA1_FIRST_STREAM_RESOURCE_INDEX = DMA1_STREAM1_RESOURCE_INDEX;
  static constexpr size_t DMA2_FIRST_STREAM_RESOURCE_INDEX = DMA2_STREAM1_RESOURCE_INDEX;

}    // namespace Thor::LLD::DMA

#endif /* !THOR_HW_DMA_REGISTER_STM32L432XX_HPP */