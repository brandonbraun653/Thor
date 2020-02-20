/********************************************************************************
 *   File Name:
 *    hw_flash_driver.hpp
 *
 *   Description:
 *    Implements the STM32F4 Flash driver interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_FLASH_HPP
#define THOR_HW_DRIVER_FLASH_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32f4x/flash/hw_flash_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_FLASH )

namespace Thor::Driver::Flash
{

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_FLASH */
#endif /* !THOR_HW_DRIVER_FLASH_HPP */