/********************************************************************************
 *   File Name:
 *    hw_flash_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/flash/hw_flash_mapping.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_FLASH )

namespace Thor::LLD::FLASH
{
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_FLASH */