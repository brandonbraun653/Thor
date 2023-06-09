/******************************************************************************
 *  File Name:
 *    uart.hpp
 *
 *  Description:
 *    Common header for Thor UART that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_UART_DETAIL_HPP
#define THOR_UART_DETAIL_HPP

#include <cstddef>

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/uart/mock/uart_mock.hpp>
#include <Thor/lld/interface/uart/mock/uart_mock_variant.hpp>
#elif defined( TARGET_STM32F4 ) || defined( TARGET_STM32L4 )
// No software supported UARTs on these devices. Yet.
namespace Thor::LLD::UART
{
  static constexpr size_t NUM_UART_PERIPHS = 0;
} // namespace Thor::LLD::UART
#else
#pragma message( "uart_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_UART_DETAIL_HPP */
