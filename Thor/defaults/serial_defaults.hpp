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
#include <Thor/definitions/dma_definitions.hpp>
#include <Thor/definitions/serial_definitions.hpp>
#include <Thor/types/serial_types.hpp>

namespace Thor::Serial
{
  static constexpr uint32_t BLOCKING_TIMEOUT_MS = 10;

  extern const std::array<const Thor::Serial::SerialConfig *const, Thor::Serial::MAX_SERIAL_CHANNELS + 1> hwConfig;

  /*------------------------------------------------
  DMA Request Signals
  ------------------------------------------------*/
  constexpr std::array<uint8_t, Thor::Serial::MAX_SERIAL_CHANNELS + 1> DMARXRequestSignal = {
    DMA::Source::NONE,       DMA::Source::S_USART1_RX, DMA::Source::S_USART2_RX, DMA::Source::S_USART3_RX, DMA::Source::S_UART4_RX,
    DMA::Source::S_UART5_RX, DMA::Source::S_USART6_RX, DMA::Source::NONE,        DMA::Source::NONE
  };

  constexpr std::array<uint8_t, Thor::Serial::MAX_SERIAL_CHANNELS + 1> DMATXRequestSignal = {
    DMA::Source::NONE,       DMA::Source::S_USART1_TX, DMA::Source::S_USART2_TX, DMA::Source::S_USART3_TX, DMA::Source::S_UART4_TX,
    DMA::Source::S_UART5_TX, DMA::Source::S_USART6_TX, DMA::Source::NONE,        DMA::Source::NONE
  };
  
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
