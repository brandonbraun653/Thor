/********************************************************************************
 * File Name:
 *   defaults.hpp
 *
 * Description:
 *   Provides definitions for external hardware configuration data
 *
 * 2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/
#pragma once
#ifndef THOR_DEFAULTS_HPP
#define THOR_DEFAULTS_HPP

#include <Thor/definitions.hpp>

namespace Thor
{
  namespace Defaults
  {
    namespace Serial
    {
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
    }    // namespace Serial

    namespace SPI
    {
      extern const Thor::SPI::SPIConfig spi_cfg[];

#if defined( STM32F7 ) || defined( STM32F4 )
      extern const SPI_InitTypeDef dflt_SPI_Init;
      extern const DMA_InitTypeDef dflt_DMA_Init_TX;
      extern const DMA_InitTypeDef dflt_DMA_Init_RX;
#endif
    }    // namespace SPI

  }    // namespace Defaults
}    // namespace Thor

#endif /* !THOR_DEFAULTS_HPP */
