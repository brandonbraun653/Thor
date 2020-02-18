/********************************************************************************
 *   File Name:
 *    hw_flash_driver_stm32f4.cpp
 *
 *   Description:
 *    Implements the STM32F4 flash driver
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/flash/hw_flash_driver.hpp>
#include <Thor/drivers/f4/flash/hw_flash_prj.hpp>
#include <Thor/drivers/f4/flash/hw_flash_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_FLASH == 1 )

namespace Thor::Driver::Flash
{
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_FLASH */