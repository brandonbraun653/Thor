/******************************************************************************
 *  File Name:
 *    hw_dma_prj.hpp
 *
 *  Description:
 *    Pulls in target specific definitions and resources used in the actual driver
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_HW_DMA_PROJECT_HPP
#define THOR_HW_DMA_PROJECT_HPP

#include <Thor/lld/stm32f4x/dma/variant/hw_dma_register_stm32f4xxxx.hpp>

#if defined( STM32F446xx )
#include <Thor/lld/stm32f4x/dma/variant/hw_dma_register_stm32f446xx.hpp>
#endif

#endif /* !THOR_HW_DMA_PROJECT_HPP */
