/********************************************************************************
 * File Name:
 *   serial_defaults.hpp
 *
 * Description:
 *   Provides definitions for external hardware configuration data
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SERIAL_DEFAULTS_HPP
#define THOR_SERIAL_DEFAULTS_HPP

/* Thor Includes */
#include <Thor/types/serial_types.hpp>
#include <Thor/definitions/serial_definitions.hpp>

namespace Thor::Serial
{
  static constexpr uint32_t BLOCKING_TIMEOUT_MS = 10;

  extern const std::array<const Thor::Serial::SerialConfig *const, Thor::Serial::MAX_SERIAL_CHANNELS + 1> hwConfig;

#if defined( STM32F7 ) || defined( STM32F4 )
  extern const USART_InitTypeDef dflt_USART_Init;
  extern const UART_InitTypeDef dflt_UART_Init;
  extern const DMA_InitTypeDef dflt_DMA_Init_TX;
  extern const DMA_InitTypeDef dflt_DMA_Init_RX;
#endif

#ifdef STM32F7
  extern const UART_AdvFeatureInitTypeDef dflt_UART_AdvInit;
#endif
}    // namespace Thor::Serial

#endif /* !THOR_SERIAL_DEFAULTS_HPP */
