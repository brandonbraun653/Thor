/********************************************************************************
 *  File Name:
 *    hld_uart_chimera.hpp
 *
 *	 Description:
 *    Chimera hooks for implementing UART
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_UART_CHIMERA_HOOKS_HPP
#define THOR_UART_CHIMERA_HOOKS_HPP

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/uart>

namespace Chimera::UART::Backend
{
  Chimera::Status_t initialize();
  Chimera::Status_t reset();
  bool isChannelUART( const Chimera::Serial::Channel channel );
  IUART_sPtr getDriver( const Chimera::Serial::Channel channel );
}    // namespace Chimera::UART::Backend

#endif /* !THOR_UART_CHIMERA_HOOKS_HPP */

