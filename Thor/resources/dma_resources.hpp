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

/* Boost Includes */
#include <boost/function.hpp>

/* Thor Includes */
#include <Thor/definitions/dma_definitions.hpp>

namespace Thor::DMA
{
  /**
   *  Stores callbacks that can be used when a particular DMA request interrupt completes
   */
  extern std::array<boost::function<void( void )>, Source::NUM_DMA_REQUESTORS> requestHandlers;


  extern const std::array<std::array<uint8_t, 8>, 8> dma1RequestMapping;

  extern const std::array<std::array<uint8_t, 8>, 8> dma2RequestMapping;

}    // namespace Thor::DMA

#endif /* !THOR_DMA_RESOURCES_HPP */