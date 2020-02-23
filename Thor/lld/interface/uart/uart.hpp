/********************************************************************************
 *  File Name:
 *    uart.hpp
 *  
 *  Description:
 *    Common header for Thor UART that configures the driver based on which
 *    chip family is being compiled against.
 *  
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_UART_CONFIG_HPP
#define THOR_UART_CONFIG_HPP

#if defined( TARGET_STM32F4 )
#include <Thor/drivers/f4/uart/hw_uart_driver.hpp>
#endif

#endif /* !THOR_UART_CONFIG_HPP */