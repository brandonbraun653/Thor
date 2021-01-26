/********************************************************************************
 *  File Name:
 *    uart.hpp
 *
 *  Description:
 *    Common header for Thor UART that configures the driver based on which
 *    chip family is being compiled against.
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_UART_DETAIL_HPP
#define THOR_UART_DETAIL_HPP

#if defined( TARGET_LLD_MOCK )
#include <Thor/lld/interface/uart/mock/uart_mock.hpp>
#include <Thor/lld/interface/uart/mock/uart_mock_variant.hpp>
#elif defined( TARGET_STM32F4 )
#include <Thor/lld/stm32f4x/uart/hw_uart_prj.hpp>
#include <Thor/lld/stm32f4x/uart/hw_uart_types.hpp>
#elif defined( TARGET_STM32L4 )
#include <Thor/lld/stm32l4x/uart/hw_uart_prj.hpp>
#include <Thor/lld/stm32l4x/uart/hw_uart_types.hpp>
#else
#pragma message( "uart_detail.hpp: Unknown target for LLD" )
#endif

#endif /* !THOR_UART_DETAIL_HPP */
