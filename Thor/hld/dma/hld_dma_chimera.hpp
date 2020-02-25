/********************************************************************************
 *  File Name:
 *    hld_dma_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing DMA
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_CHIMERA_HOOKS_HPP
#define THOR_DMA_CHIMERA_HOOKS_HPP

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/dma>

namespace Chimera::DMA::Backend
{
  Chimera::Status_t initialize();

  Chimera::Status_t reset();

  Chimera::DMA::DMA_sPtr create_shared_ptr();

  Chimera::DMA::DMA_uPtr create_unique_ptr();
}    // namespace Chimera::DMA::Backend

#endif /* !THOR_DMA_CHIMERA_HOOKS_HPP */
