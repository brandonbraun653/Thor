/********************************************************************************
 *   File Name:
 *    serial_definitions.hpp
 *
 *   Description:
 *    Thor Serial Definitions
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SERIAL_DEFS_HPP
#define THOR_SERIAL_DEFS_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/definitions/uart_definitions.hpp>
#include <Thor/definitions/usart_definitions.hpp>

namespace Thor::Serial
{
  constexpr uint8_t MAX_SERIAL_CHANNELS = Thor::UART::MAX_UART_CHANNELS + Thor::USART::MAX_USART_CHANNELS;

}    // namespace Thor::Serial

#endif /* !THOR_SERIAL_DEFS_HPP */