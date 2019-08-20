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
#include <Chimera/container.hpp>

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
  extern const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;

  /**
   *  Maps a DMA peripheral stream into the corresponding resource index
   */
  extern const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> StreamToResourceIndex;

  /**
   *  Maps a DMA peripheral stream into the corresponding register index
   *  which is used to access offsets in the hardware registers
   */
  extern const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> StreamToRegisterIndex;

  /**
   *  Conversion lookup tables between Chimera DMA configuration options 
   *  and their register equivalents on the hardware side for Thor.
   */
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::TransferDirection::NUM_OPTIONS)> TransferMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::Mode::NUM_OPTIONS)> ModeMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::MemoryIncrement::NUM_OPTIONS)> MemoryIncrementMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::MemoryAlignment::NUM_OPTIONS)> MemoryAlignmentMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::PeripheralIncrement::NUM_OPTIONS)> PeripheralIncrementMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::PeripheralAlignment::NUM_OPTIONS)> PeripheralAlignmentMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::Priority::NUM_OPTIONS)> PriorityMap;


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
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
#endif /* !THOR_HW_DMA_MAPPING_HPP */
