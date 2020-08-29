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
#ifndef THOR_SPI_DETAIL_HPP
#define THOR_SPI_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/spi/mock/spi_mock.hpp>
#include <Thor/lld/interface/spi/mock/spi_mock_variant.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/spi/hw_spi_driver.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_mapping.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/spi/hw_spi_driver.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_mapping.hpp>
#else
#pragma message( "spi_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_SPI_DETAIL_HPP */
