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
  IDMA_sPtr getDriver( const Controller channel );
}    // namespace Chimera::DMA::Backend

#endif /* !THOR_DMA_CHIMERA_HOOKS_HPP */
