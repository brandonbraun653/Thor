/********************************************************************************
 *  File Name:
 *    nvic.hpp
 *
 *  Description:
 *    Common header for Thor NVIC that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_NVIC_CONFIG_HPP
#define THOR_NVIC_CONFIG_HPP

#include <Thor/preprocessor.hpp>

/*-------------------------------------------------
Using the custom STM32 driver
-------------------------------------------------*/
#if defined( THOR_CUSTOM_DRIVERS ) && ( THOR_CUSTOM_DRIVERS == 1 )

#if defined( TARGET_STM32F4 )
#include <Thor/drivers/f4/nvic/hw_nvic_driver.hpp>
#endif

#if defined( TARGET_STM32F7 )
#include <Thor/drivers/f7/nvic/hw_nvic_driver.hpp>
#endif

/*-------------------------------------------------
Using the STM32HAL
-------------------------------------------------*/
#elif defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )

/*-------------------------------------------------
Using an unsupported driver?
-------------------------------------------------*/
#else
#error Unknown Thor gpio driver implementation
#endif /* THOR_CUSTOM_DRIVERS */

#endif /* !THOR_NVIC_CONFIG_HPP */