/********************************************************************************
 *  File Name:
 *    dma.hpp
 *
 *  Description:
 *    Common header for Thor DMA that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_CONFIG_HPP
#define THOR_DMA_CONFIG_HPP

#include <Thor/preprocessor.hpp>

/*-------------------------------------------------
Using the custom STM32 driver
-------------------------------------------------*/
#if defined( THOR_CUSTOM_DRIVERS ) && ( THOR_CUSTOM_DRIVERS == 1 )

#if defined( TARGET_STM32F4 )
#include <Thor/drivers/f4/dma/hw_dma_driver.hpp>
#include <Thor/drivers/f4/dma/hw_dma_prj.hpp>
#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>
#endif

#if defined( TARGET_STM32F7 )
#include <Thor/drivers/f7/dma/hw_dma_driver.hpp>
#endif

/*-------------------------------------------------
Using an unsupported driver?
-------------------------------------------------*/
#else
#error Unknown Thor gpio driver implementation
#endif /* THOR_CUSTOM_DRIVERS */

#endif /* !THOR_DMA_CONFIG_HPP */