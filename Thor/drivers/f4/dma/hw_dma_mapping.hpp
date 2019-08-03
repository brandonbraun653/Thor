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

  /**
   *  Gets the interrupt request number tied to a DMA instance.
   *
   *  @note Must match the mapping in InstanceToResourceIndex
   */
  extern const IRQn_Type DMAStream_IRQn[ NUM_DMA_STREAMS * NUM_DMA_PERIPHS ];


  extern const std::unordered_map<Chimera::DMA::Channel, StreamX *const> ChannelToStream;
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
#endif /* !THOR_HW_DMA_MAPPING_HPP */
