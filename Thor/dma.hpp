/********************************************************************************
 *  File Name:
 *    dma.hpp
 *
 *  Description:
 *    Thor implementation of the DMA driver
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_HPP
#define THOR_DMA_HPP

/* Chimera Includes */
#include <Chimera/interface/dma_intf.hpp>

/* Thor Includes */
#include <Thor/definitions/dma_definitions.hpp>

namespace Thor::DMA
{

  class DMAClass : public Chimera::DMA::Interface
  {
  public:
    DMAClass();
    ~DMAClass();


  };
}    // namespace Thor::DMA

#endif /* !THOR_DMA_HPP */
