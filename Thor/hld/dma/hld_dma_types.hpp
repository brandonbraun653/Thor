/********************************************************************************
 *  File Name:
 *    dma_types.hpp
 *
 *  Description:
 *    Thor DMA types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_TYPES_HPP
#define THOR_DMA_TYPES_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::DMA
{
  enum class TransferDirection : uint8_t
  {
    PERIPH_TO_MEM,
    MEM_TO_PERIPH,
    MEM_TO_MEM,
    TRANSFER_DIRECTION_UNDEFINED
  };
}

#endif /* !THOR_DMA_TYPES_HPP */