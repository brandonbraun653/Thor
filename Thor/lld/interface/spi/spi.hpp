/********************************************************************************
 *  File Name:
 *    spi.hpp
 *
 *  Description:
 *    Common header for Thor SPI that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SPI_CONFIG_HPP
#define THOR_SPI_CONFIG_HPP

#include <Thor/hld/common/preprocessor.hpp>


#if defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/spi/hw_spi_driver.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_mapping.hpp>
#else
#pragma message( "Unknown target device for SPI low level driver" )
#endif

#endif  /* !THOR_SPI_CONFIG_HPP */