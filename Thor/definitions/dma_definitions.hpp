/********************************************************************************
 *   File Name:
 *    dma_definitions.hpp
 *
 *   Description:
 *    Thor DMA Definitions
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DMA_DEFS_HPP
#define THOR_DMA_DEFS_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/headers.hpp>

namespace Thor::DMA
{
  /*------------------------------------------------
  Describes the available sources from which a DMA operation can
  be started with.
  ------------------------------------------------*/
  namespace Source
  {
    static constexpr uint8_t NONE         = 0;
    static constexpr uint8_t S_ADC1       = 1;
    static constexpr uint8_t S_ADC2       = 2;
    static constexpr uint8_t S_ADC3       = 3;
    static constexpr uint8_t S_DAC1       = 4;
    static constexpr uint8_t S_DAC2       = 5;
    static constexpr uint8_t S_DCMI       = 6;
    static constexpr uint8_t S_FMPI2C1_RX = 7;
    static constexpr uint8_t S_I2C1_TX    = 8;
    static constexpr uint8_t S_I2C1_RX    = 9;
    static constexpr uint8_t S_I2C2_TX    = 10;
    static constexpr uint8_t S_I2C2_RX    = 11;
    static constexpr uint8_t S_I2C3_TX    = 12;
    static constexpr uint8_t S_I2C3_RX    = 13;
    static constexpr uint8_t S_SPDIFRX_DT = 14;
    static constexpr uint8_t S_SPDIFRX_CS = 15;
    static constexpr uint8_t S_SPI1_TX    = 16;
    static constexpr uint8_t S_SPI1_RX    = 17;
    static constexpr uint8_t S_SPI2_TX    = 18;
    static constexpr uint8_t S_SPI2_RX    = 19;
    static constexpr uint8_t S_SPI3_TX    = 20;
    static constexpr uint8_t S_SPI3_RX    = 21;
    static constexpr uint8_t S_SPI4_TX    = 22;
    static constexpr uint8_t S_SPI4_RX    = 23;
    static constexpr uint8_t S_QUADSPI    = 24;
    static constexpr uint8_t S_SAI1_A     = 25;
    static constexpr uint8_t S_SAI1_B     = 26;
    static constexpr uint8_t S_SAI2_A     = 27;
    static constexpr uint8_t S_SAI2_B     = 28;
    static constexpr uint8_t S_SDIO       = 29;
    static constexpr uint8_t S_TIM1_UP    = 30;
    static constexpr uint8_t S_TIM1_TRIG  = 31;
    static constexpr uint8_t S_TIM1_COM   = 32;
    static constexpr uint8_t S_TIM1_CH1   = 33;
    static constexpr uint8_t S_TIM1_CH2   = 34;
    static constexpr uint8_t S_TIM1_CH3   = 35;
    static constexpr uint8_t S_TIM1_CH4   = 36;
    static constexpr uint8_t S_TIM2_UP    = 37;
    static constexpr uint8_t S_TIM2_CH1   = 38;
    static constexpr uint8_t S_TIM2_CH2   = 39;
    static constexpr uint8_t S_TIM2_CH3   = 40;
    static constexpr uint8_t S_TIM2_CH4   = 41;
    static constexpr uint8_t S_TIM3_UP    = 42;
    static constexpr uint8_t S_TIM3_TRIG  = 43;
    static constexpr uint8_t S_TIM3_CH1   = 44;
    static constexpr uint8_t S_TIM3_CH2   = 45;
    static constexpr uint8_t S_TIM3_CH3   = 46;
    static constexpr uint8_t S_TIM3_CH4   = 47;
    static constexpr uint8_t S_TIM4_UP    = 48;
    static constexpr uint8_t S_TIM4_CH1   = 49;
    static constexpr uint8_t S_TIM4_CH2   = 50;
    static constexpr uint8_t S_TIM4_CH3   = 51;
    static constexpr uint8_t S_TIM5_UP    = 52;
    static constexpr uint8_t S_TIM5_TRIG  = 53;
    static constexpr uint8_t S_TIM5_CH1   = 54;
    static constexpr uint8_t S_TIM5_CH2   = 55;
    static constexpr uint8_t S_TIM5_CH3   = 56;
    static constexpr uint8_t S_TIM5_CH4   = 57;
    static constexpr uint8_t S_TIM6_UP    = 58;
    static constexpr uint8_t S_TIM7_UP    = 59;
    static constexpr uint8_t S_TIM8_UP    = 60;
    static constexpr uint8_t S_TIM8_TRIG  = 61;
    static constexpr uint8_t S_TIM8_COM   = 62;
    static constexpr uint8_t S_TIM8_CH4   = 63;
    static constexpr uint8_t S_TIM8_CH1   = 64;
    static constexpr uint8_t S_TIM8_CH2   = 65;
    static constexpr uint8_t S_TIM8_CH3   = 66;
    static constexpr uint8_t S_UART4_TX   = 67;
    static constexpr uint8_t S_UART4_RX   = 68;
    static constexpr uint8_t S_UART5_TX   = 69;
    static constexpr uint8_t S_UART5_RX   = 70;
    static constexpr uint8_t S_USART1_TX  = 71;
    static constexpr uint8_t S_USART1_RX  = 72;
    static constexpr uint8_t S_USART2_TX  = 73;
    static constexpr uint8_t S_USART2_RX  = 74;
    static constexpr uint8_t S_USART3_TX  = 75;
    static constexpr uint8_t S_USART3_RX  = 76;
    static constexpr uint8_t S_USART6_TX  = 77;
    static constexpr uint8_t S_USART6_RX  = 78;

    static constexpr uint8_t NUM_DMA_REQUESTORS = 79;
  }    // namespace Source
}    // namespace Thor::DMA

#endif /* !THOR_DMA_DEFS_HPP */