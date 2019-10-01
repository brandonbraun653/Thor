/********************************************************************************
 *   File Name:
 *    hw_spi_mapping.hpp
 *
 *   Description:
 *    Useful mappings for the SPI peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_MAPPING_HPP
#define THOR_HW_SPI_MAPPING_HPP

/* Thor Includes */
#include <Thor/headers.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
}

#endif  /* TARGET_STM32F4 && THOR_DRIVER_SPI */
#endif  /* THOR_HW_SPI_MAPPING_HPP */
