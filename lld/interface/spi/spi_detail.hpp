/********************************************************************************
 *  File Name:
 *    spi.hpp
 *
 *  Description:
 *    Common header for Thor SPI that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_INTF_DETAIL_HPP
#define THOR_LLD_SPI_INTF_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/spi/mock/spi_mock.hpp>
#include <Thor/lld/interface/spi/mock/spi_mock_variant.hpp>
#elif defined( TARGET_LLD_TEST )
#include <Thor/lld/interface/spi/sim/spi_sim_variant.hpp>
#include <Thor/lld/interface/spi/sim/spi_sim_types.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_types.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/spi/hw_spi_prj.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_types.hpp>
#else
#pragma message( "spi_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_LLD_SPI_INTF_DETAIL_HPP */
