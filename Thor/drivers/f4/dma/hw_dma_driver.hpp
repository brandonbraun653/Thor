/********************************************************************************
 *   File Name:
 *    hw_dma_driver.hpp
 *
 *   Description:
 *    STM32 Driver for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

 #pragma once
#ifndef THOR_HW_DMA_DRIVER_HPP
#define THOR_HW_DMA_DRIVER_HPP

/* C++ Includes */

/* Driver Includes */
#include <Thor/headers.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */
#endif /* !THOR_HW_DMA_DRIVER_HPP */
