/********************************************************************************
 *  File Name:
 *    rcc.hpp
 *
 *  Description:
 *    Common header for Thor RCC that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_RCC_CONFIG_HPP
#define THOR_RCC_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/rcc/hw_rcc_driver.hpp>
#endif

#if defined( TARGET_STM32F7 )
#include <Thor/drivers/f7/rcc/hw_rcc_driver.hpp>
#endif

#endif /* !THOR_RCC_CONFIG_HPP */