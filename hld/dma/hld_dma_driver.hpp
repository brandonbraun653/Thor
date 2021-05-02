/********************************************************************************
 *  File Name:
 *    hld_dma_driver.hpp
 *
 *  Description:
 *    Thor implementation of the DMA driver
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_HPP
#define THOR_DMA_HPP

/* Chimera Includes */
#include <Chimera/thread>
#include <Chimera/dma>

/* Thor Includes */
#include <Thor/hld/dma/hld_dma_intf.hpp>

namespace Thor::DMA
{
  Chimera::Status_t initialize();


}    // namespace Thor::DMA

#endif /* !THOR_DMA_HPP */
