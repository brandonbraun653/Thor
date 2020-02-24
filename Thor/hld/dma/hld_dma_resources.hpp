/********************************************************************************
 *  File Name:
 *    dma_resources.hpp
 *
 *  Description:
 *    Thor resources used in the DMA driver. Chip agnostic.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_RESOURCES_HPP
#define THOR_DMA_RESOURCES_HPP

/* C++ Includes */
#include <array>

/* Thor Includes */
#include <Thor/hld/dma/hld_dma_types.hpp>

namespace Thor::DMA
{
  /**
   *  The number of possible request generators that are currently supported
   *  in the DMA driver. Currently this is sized only for signals that have 
   *  actually been used in project code. This minimizes memory usage nicely.
   */
  static constexpr size_t NUM_REQUEST_GENERATORS = 9u;
  extern std::array<Thor::LLD::DMA::StreamResources, NUM_REQUEST_GENERATORS> RequestGenerators;

}    // namespace Thor::DMA

#endif /* !THOR_DMA_RESOURCES_HPP */