/********************************************************************************
 *   File Name:
 *    hw_dma_mapping.hpp
 *
 *   Description:
 *    STM32 Mappings for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

 #pragma once
#ifndef THOR_HW_DMA_MAPPING_HPP
#define THOR_HW_DMA_MAPPING_HPP

/* C++ Includes */
#include <unordered_map>

/* Chimera Includes */
#include <Chimera/dma.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/interrupt/hw_it_prj.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_types.hpp>
#include <Thor/drivers/f4/dma/hw_dma_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
  /**
   *  Maps a DMA peripheral into the corresponding resource index
   */
  extern const Thor::Driver::RCC::ResourceMap_t InstanceToResourceIndex;

  /**
   *  Maps a DMA peripheral stream into the corresponding resource index
   */
  extern const Thor::Driver::RCC::ResourceMap_t StreamToResourceIndex;

  /**
   *  Maps a DMA peripheral stream into the corresponding register index
   *  which is used to access offsets in the hardware registers
   */
  extern const Thor::Driver::RCC::ResourceMap_t StreamToRegisterIndex;

  extern const std::unordered_map<Chimera::DMA::TransferDirection, uint32_t> TransferMap;
  extern const std::unordered_map<Chimera::DMA::Mode, uint32_t> ModeMap;
  extern const std::unordered_map<Chimera::DMA::Channel, uint32_t> ChannelMap;
  extern const std::unordered_map<Chimera::DMA::MemoryIncrement, uint32_t> MemoryIncrementMap;
  extern const std::unordered_map<Chimera::DMA::MemoryAlignment, uint32_t> MemoryAlignmentMap;
  extern const std::unordered_map<Chimera::DMA::PeripheralIncrement, uint32_t> PeripheralIncrementMap;
  extern const std::unordered_map<Chimera::DMA::PeripheralAlignment, uint32_t> PeripheralAlignmentMap;
  extern const std::unordered_map<Chimera::DMA::Priority, uint32_t> PriorityMap;


  /**
   *  Gets the interrupt request number tied to a DMA instance.
   *
   *  @note Must match the mapping in InstanceToResourceIndex
   */
  extern const IRQn_Type DMAStream_IRQn[ NUM_DMA_STREAMS ];

  /**
   *  Converts resource indexes into the flags used to parse
   *  the LISR and HISR status registers.
   */
  extern const uint32_t DMAStream_TCIF[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_HTIF[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_TEIF[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_DMEIF[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_FEIF[ NUM_DMA_STREAMS ];

  extern const std::unordered_map<Chimera::DMA::Channel, StreamX *const> ChannelToStream;
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
#endif /* !THOR_HW_DMA_MAPPING_HPP */
