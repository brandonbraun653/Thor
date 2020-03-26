/********************************************************************************
 *  File Name:
 *    hw_flash_driver.hpp
 *
 *  Description:
 *    Declares the LLD interface to the STM32L4 series FLASH hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_FLASH_DRIVER_STM32L4_HPP
#define THOR_HW_FLASH_DRIVER_STM32L4_HPP

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/stm32l4x/flash/hw_flash_types.hpp>
#include <Thor/lld/stm32l4x/flash/hw_flash_mapping.hpp>

namespace Thor::LLD::FLASH
{

}    // namespace Thor::LLD::FLASH

#endif /* !THOR_HW_FLASH_DRIVER_STM32L4_HPP */
