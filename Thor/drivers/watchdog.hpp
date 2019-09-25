/********************************************************************************
 *  File Name:
 *    watchdog.hpp
 *
 *  Description:
 *    Common header for Thor Watchdog that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_WATCHDOG_CONFIG_HPP
#define THOR_WATCHDOG_CONFIG_HPP

#include <Thor/preprocessor.hpp>

/*-------------------------------------------------
Using the custom STM32 driver
-------------------------------------------------*/
#if defined( THOR_CUSTOM_DRIVERS ) && ( THOR_CUSTOM_DRIVERS == 1 )

#if defined( TARGET_STM32F4 )
#include <Thor/drivers/f4/iwdg/hw_iwdg_driver.hpp>
#include <Thor/drivers/f4/wwdg/hw_wwdg_driver.hpp>
#endif

#if defined( TARGET_STM32F7 )
#include <Thor/drivers/f7/dma/hw_dma_driver.hpp>
#endif

/*-------------------------------------------------
Using an unsupported driver?
-------------------------------------------------*/
#else
#error Unknown Thor gpio driver implementation
#endif /* THOR_CUSTOM_DRIVERS */

#endif /* !THOR_WATCHDOG_CONFIG_HPP */