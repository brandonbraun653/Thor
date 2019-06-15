/********************************************************************************
 *  File Name:
 *    Uart.hpp
 *  
 *  Description:
 *    Common header for Thor UART that configures the driver based on which
 *    chip family is being compiled against.
 *  
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_UART_CONFIG_HPP
#define THOR_UART_CONFIG_HPP

#include <Thor/preprocessor.hpp>

#if defined( STM32F4 )
#include <Thor/drivers/f4/uart/hw_uart_driver.hpp>
#endif /* STM32F4 */

#if defined( STM32F7 )
#include <Thor/drivers/f7/uart/hw_uart_driver.hpp>
#endif /* STM32F7 */

#endif /* !THOR_UART_CONFIG_HPP */