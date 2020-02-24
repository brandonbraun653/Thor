/********************************************************************************
 *  File Name:
 *    dma.hpp
 *
 *  Description:
 *    Common header for Thor DMA that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_CONFIG_HPP
#define THOR_DMA_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/dma/hw_dma_driver.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_prj.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_mapping.hpp>
#endif

#endif /* !THOR_DMA_CONFIG_HPP */