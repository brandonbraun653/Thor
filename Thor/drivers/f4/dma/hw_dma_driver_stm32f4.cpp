/********************************************************************************
 *   File Name:
 *    hw_dma_mapping.hpp
 *
 *   Description:
 *    STM32 Mappings for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/dma/hw_dma_driver.hpp>
#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>
#include <Thor/drivers/f4/dma/hw_dma_prj.hpp>
#include <Thor/drivers/f4/dma/hw_dma_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */