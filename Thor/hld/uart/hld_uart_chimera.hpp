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

  Chimera::UART::UART_sPtr create_shared_ptr();

  Chimera::UART::UART_uPtr create_unique_ptr();
}    // namespace Chimera::UART::Backend

#endif /* !THOR_UART_CHIMERA_HOOKS_HPP */

