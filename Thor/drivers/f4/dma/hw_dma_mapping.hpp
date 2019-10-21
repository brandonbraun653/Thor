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
/*------------------------------------------------
DMA1 Register and Stream Memory Mappings 
------------------------------------------------*/
#if defined( STM32_DMA1_PERIPH_AVAILABLE )
  extern RegisterMap *DMA1_PERIPH;

#if defined( STM32_DMA1_STREAM0_AVAILABLE )
  extern StreamX *DMA1_STREAM0;
#endif

#if defined( STM32_DMA1_STREAM1_AVAILABLE )
  extern StreamX *DMA1_STREAM1;
#endif

#if defined( STM32_DMA1_STREAM2_AVAILABLE )
  extern StreamX *DMA1_STREAM2;
#endif

#if defined( STM32_DMA1_STREAM3_AVAILABLE )
  extern StreamX *DMA1_STREAM3;
#endif

#if defined( STM32_DMA1_STREAM4_AVAILABLE )
  extern StreamX *DMA1_STREAM4;
#endif

#if defined( STM32_DMA1_STREAM5_AVAILABLE )
  extern StreamX *DMA1_STREAM5;
#endif

#if defined( STM32_DMA1_STREAM6_AVAILABLE )
  extern StreamX *DMA1_STREAM6;
#endif

#if defined( STM32_DMA1_STREAM7_AVAILABLE )
  extern StreamX *DMA1_STREAM7;
#endif

#endif /* STM32_DMA1_PERIPH_AVAILABLE */

/*------------------------------------------------
DMA2 Register and Stream Memory Mappings
------------------------------------------------*/
#if defined( STM32_DMA2_PERIPH_AVAILABLE )
  extern RegisterMap *DMA2_PERIPH;

#if defined( STM32_DMA2_STREAM0_AVAILABLE )
  extern StreamX *DMA2_STREAM0;
#endif

#if defined( STM32_DMA2_STREAM1_AVAILABLE )
  extern StreamX *DMA2_STREAM1;
#endif

#if defined( STM32_DMA2_STREAM2_AVAILABLE )
  extern StreamX *DMA2_STREAM2;
#endif

#if defined( STM32_DMA2_STREAM3_AVAILABLE )
  extern StreamX *DMA2_STREAM3;
#endif

#if defined( STM32_DMA2_STREAM4_AVAILABLE )
  extern StreamX *DMA2_STREAM4;
#endif

#if defined( STM32_DMA2_STREAM5_AVAILABLE )
  extern StreamX *DMA2_STREAM5;
#endif

#if defined( STM32_DMA2_STREAM6_AVAILABLE )
  extern StreamX *DMA2_STREAM6;
#endif

#if defined( STM32_DMA2_STREAM7_AVAILABLE )
  extern StreamX *DMA2_STREAM7;
#endif

#endif /* STM32_DMA2_PERIPH_AVAILABLE */

  /*------------------------------------------------
  Peripheral Memory Mapping
  ------------------------------------------------*/
  extern PeriphRegisterList periphInstanceList;
  extern StreamRegisterList streamInstanceList;
  extern DriverInstanceList dmaObjects;

  /*------------------------------------------------
  Maps a DMA peripheral or Stream into the corresponding resource index
  ------------------------------------------------*/
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> StreamToResourceIndex;
  extern Chimera::Container::LightFlatMap<std::uintptr_t, size_t> StreamToRegisterIndex;

  /*------------------------------------------------
  Conversion lookup tables between Chimera DMA configuration options
  and their register equivalents on the hardware side for Thor.
  ------------------------------------------------*/
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::TransferDirection::NUM_OPTIONS)> TransferMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::Mode::NUM_OPTIONS)> ModeMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::MemoryIncrement::NUM_OPTIONS)> MemoryIncrementMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::MemoryAlignment::NUM_OPTIONS)> MemoryAlignmentMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::PeripheralIncrement::NUM_OPTIONS)> PeripheralIncrementMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::PeripheralAlignment::NUM_OPTIONS)> PeripheralAlignmentMap;
  extern const std::array<uint32_t, static_cast<uint8_t>(Chimera::DMA::Priority::NUM_OPTIONS)> PriorityMap;

  /*------------------------------------------------
  Hardware Mappings for each Stream
  ------------------------------------------------*/
  extern const IRQn_Type DMAStream_IRQn[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_TCIF[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_HTIF[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_TEIF[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_DMEIF[ NUM_DMA_STREAMS ];
  extern const uint32_t DMAStream_FEIF[ NUM_DMA_STREAMS ];

  /**
   *  Initializes memory associated with mapping
   *  
   *  @return void
   */
  extern void initializeMapping();

  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  extern bool isDMA( const std::uintptr_t address );
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
#endif /* !THOR_HW_DMA_MAPPING_HPP */
