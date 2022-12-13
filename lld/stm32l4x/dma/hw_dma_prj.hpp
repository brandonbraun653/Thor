/******************************************************************************
 *  File Name:
 *    hw_dma_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_PROJECT_HPP
#define THOR_HW_DMA_PROJECT_HPP

#include <Thor/lld/stm32l4x/dma/variant/hw_dma_register_stm32l4xxxx.hpp>

#if defined( STM32L432xx )
#include <Thor/lld/stm32l4x/dma/variant/hw_dma_register_stm32l432xx.hpp>
#endif

#endif /* !THOR_HW_DMA_PROJECT_HPP */
