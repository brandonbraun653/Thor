/********************************************************************************
 *  File Name:
 *    hld_dma_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing DMA
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_CHIMERA_HOOKS_HPP
#define THOR_DMA_CHIMERA_HOOKS_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/dma>

namespace Chimera::DMA::Backend
{
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  RequestId constructPipe( const PipeConfig &config );
  RequestId transfer( const MemTransfer &transfer );
  RequestId transfer( const PipeTransfer &transfer );
}    // namespace Chimera::DMA::Backend

#endif /* !THOR_DMA_CHIMERA_HOOKS_HPP */
