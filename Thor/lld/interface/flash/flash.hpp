/********************************************************************************
 *  File Name:
 *    flash.hpp
 *
 *  Description:
 *    Common header for Thor Flash that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_FLASH_CONFIG_HPP
#define THOR_FLASH_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/flash/hw_flash_driver.hpp>
#endif

#endif /* !THOR_FLASH_CONFIG_HPP */