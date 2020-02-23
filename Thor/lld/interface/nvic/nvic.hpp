/********************************************************************************
 *  File Name:
 *    nvic.hpp
 *
 *  Description:
 *    Common header for Thor NVIC that configures the driver based on which
 *    chip family is being compiled against.
 * 
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_NVIC_CONFIG_HPP
#define THOR_NVIC_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/nvic/hw_nvic_driver.hpp>
#endif

#if defined( TARGET_STM32F7 )
#include <Thor/drivers/f7/nvic/hw_nvic_driver.hpp>
#endif

#endif /* !THOR_NVIC_CONFIG_HPP */