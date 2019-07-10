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
  /**
   *  Channel 0 doesn't really exist in hardware (virtual)
   */
  static constexpr uint8_t INVALID_CHANNEL = 0u;

  /**
   *  Total number of hardware channels
   */
  static constexpr uint8_t MAX_PHYSICAL_CHANNELS = Thor::UART::MAX_UART_CHANNELS + Thor::USART::MAX_USART_CHANNELS;

  /** 
   *  Total number of hardware channels plus the imaginary invalid channel
   */
  static constexpr uint8_t MAX_VIRTUAL_CHANNELS = MAX_PHYSICAL_CHANNELS + 1;

}    // namespace Thor::Serial

#endif /* !THOR_SERIAL_DEFS_HPP */