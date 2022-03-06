/********************************************************************************
 *  File Name:
 *    dma_detail.hpp
 *
 *  Description:
 *    Common header for Thor DMA that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_DETAIL_HPP
#define THOR_DMA_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/dma/mock/dma_mock.hpp>
#include <Thor/lld/interface/dma/mock/dma_mock_variant.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/dma/hw_dma_types.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_prj.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/dma/hw_dma_types.hpp>
#include <Thor/lld/stm32l4x/dma/hw_dma_prj.hpp>
#else
#pragma message( "dma_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_DMA_DETAIL_HPP */
