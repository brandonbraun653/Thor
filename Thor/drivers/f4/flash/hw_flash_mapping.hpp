/********************************************************************************
 *   File Name:
 *    hw_flash_mapping.hpp
 *
 *   Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once 
#ifndef THOR_HW_FLASH_MAPPING_HPP
#define THOR_HW_FLASH_MAPPING_HPP

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/flash/hw_flash_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_FLASH == 1 )

namespace Thor::Driver::Flash
{
  extern RegisterMap *const FLASH_PERIPH;
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_FLASH */
#endif /* !THOR_HW_FLASH_MAPPING_HPP */