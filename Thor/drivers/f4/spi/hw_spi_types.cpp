/********************************************************************************
 *   File Name:
 *    hw_spi_types.cpp
 *
 *   Description:
 *    STM32F4 specific types for SPI
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */

/* Chimera Includes */

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/spi/hw_spi_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */