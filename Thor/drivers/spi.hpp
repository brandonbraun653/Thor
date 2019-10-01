/********************************************************************************
 *   File Name:
 *    spi.hpp
 *
 *   Description:
 *    Common header for Thor SPI that configures the driver based on which
 *    chip family is being compiled against.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_CONFIG_HPP
#define THOR_SPI_CONFIG_HPP

#include <Thor/preprocessor.hpp>

/*-------------------------------------------------
Using the custom STM32 driver
-------------------------------------------------*/
#if defined( THOR_CUSTOM_DRIVERS ) && ( THOR_CUSTOM_DRIVERS == 1 )

#if defined( TARGET_STM32F4 )
#include <Thor/drivers/f4/spi/hw_spi_driver.hpp>
#endif

/*-------------------------------------------------
Using an unsupported driver?
-------------------------------------------------*/
#else
#error Unknown Thor SPI driver implementation
#endif /* THOR_CUSTOM_DRIVERS */

#endif  /* !THOR_SPI_CONFIG_HPP */