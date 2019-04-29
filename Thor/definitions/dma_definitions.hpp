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
/* Useful Macros for Generating DMA Register Addresses*/
#define DMA_OFFSET_LISR 0x00U
#define DMA_OFFSET_HISR 0x04U
#define DMA_OFFSET_LIFCR 0x08U
#define DMA_OFFSET_HIFCR 0x0CU
#define DMA_OFFSET_SxCR( STREAM_NUMBER ) ( 0x10U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxNDTR( STREAM_NUMBER ) ( 0x14U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxPAR( STREAM_NUMBER ) ( 0x18U + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxM0AR( STREAM_NUMBER ) ( 0x1CU + 0x18U * ( uint32_t )STREAM_NUMBER )
#define DMA_OFFSET_SxM1AR( STREAM_NUMBER ) ( 0x20U + 0x18U * ( uint32_t )STREAM_NUMBER )

// The data sheet has conflicting address definitions for this register. The register map
// does not match manual calculations using the equation below...unsure which is right
#define DMA_OFFSET_SxFCR( STREAM_NUMBER ) ( 0x24U + 0x24U * ( uint32_t )STREAM_NUMBER )

/*------------------------------------------------
DMA1 Register Address Accessors
------------------------------------------------*/
#define DMA1_LISR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_LISR ) )
#define DMA1_HISR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_HISR ) )
#define DMA1_LIFCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_LIFCR ) )
#define DMA1_HIFCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_HIFCR ) )
#define DMA1_S0CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 0 ) ) )
#define DMA1_S1CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 1 ) ) ) /* DMA1_SxCR */
#define DMA1_S2CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 2 ) ) )
#define DMA1_S3CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 3 ) ) )
#define DMA1_S4CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 4 ) ) )
#define DMA1_S5CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 5 ) ) )
#define DMA1_S6CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 6 ) ) )
#define DMA1_S7CR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxCR( 7 ) ) )
#define DMA1_S0NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 0 ) ) )
#define DMA1_S1NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 1 ) ) ) /* DMA1_SxNDTR */
#define DMA1_S2NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 2 ) ) )
#define DMA1_S3NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 3 ) ) )
#define DMA1_S4NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 4 ) ) )
#define DMA1_S5NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 5 ) ) )
#define DMA1_S6NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 6 ) ) )
#define DMA1_S7NDTR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxNDTR( 7 ) ) )
#define DMA1_S0PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 0 ) ) )
#define DMA1_S1PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 1 ) ) ) /* DMA1_SxPAR */
#define DMA1_S2PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 2 ) ) )
#define DMA1_S3PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 3 ) ) )
#define DMA1_S4PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 4 ) ) )
#define DMA1_S5PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 5 ) ) )
#define DMA1_S6PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 6 ) ) )
#define DMA1_S7PAR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxPAR( 7 ) ) )
#define DMA1_S0M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 0 ) ) )
#define DMA1_S1M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 1 ) ) ) /* DMA1_SxM0AR */
#define DMA1_S2M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 2 ) ) )
#define DMA1_S3M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 3 ) ) )
#define DMA1_S4M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 4 ) ) )
#define DMA1_S5M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 5 ) ) )
#define DMA1_S6M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 6 ) ) )
#define DMA1_S7M0AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM0AR( 7 ) ) )
#define DMA1_S0M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 0 ) ) )
#define DMA1_S1M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 1 ) ) ) /* DMA1_SxM1AR*/
#define DMA1_S2M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 2 ) ) )
#define DMA1_S3M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 3 ) ) )
#define DMA1_S4M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 4 ) ) )
#define DMA1_S5M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 5 ) ) )
#define DMA1_S6M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 6 ) ) )
#define DMA1_S7M1AR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxM1AR( 7 ) ) )
#define DMA1_S0FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 0 ) ) )
#define DMA1_S1FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 1 ) ) ) /* DMA1_SxFCR */
#define DMA1_S2FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 2 ) ) )
#define DMA1_S3FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 3 ) ) )
#define DMA1_S4FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 4 ) ) )
#define DMA1_S5FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 5 ) ) )
#define DMA1_S6FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 6 ) ) )
#define DMA1_S7FCR ( *( volatile uint32_t * )( DMA1_BASE + DMA_OFFSET_SxFCR( 7 ) ) )

/*------------------------------------------------
DMA2 Register Address Accessors
------------------------------------------------*/
#define DMA2_LISR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_LISR ) )
#define DMA2_HISR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_HISR ) )
#define DMA2_LIFCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_LIFCR ) )
#define DMA2_HIFCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_HIFCR ) )
#define DMA2_S0CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 0 ) ) )
#define DMA2_S1CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 1 ) ) ) /* DMA2_SxCR */
#define DMA2_S2CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 2 ) ) )
#define DMA2_S3CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 3 ) ) )
#define DMA2_S4CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 4 ) ) )
#define DMA2_S5CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 5 ) ) )
#define DMA2_S6CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 6 ) ) )
#define DMA2_S7CR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxCR( 7 ) ) )
#define DMA2_S0NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 0 ) ) )
#define DMA2_S1NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 1 ) ) ) /* DMA2_SxNDTR */
#define DMA2_S2NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 2 ) ) )
#define DMA2_S3NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 3 ) ) )
#define DMA2_S4NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 4 ) ) )
#define DMA2_S5NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 5 ) ) )
#define DMA2_S6NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 6 ) ) )
#define DMA2_S7NDTR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxNDTR( 7 ) ) )
#define DMA2_S0PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 0 ) ) )
#define DMA2_S1PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 1 ) ) ) /* DMA2_SxPAR */
#define DMA2_S2PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 2 ) ) )
#define DMA2_S3PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 3 ) ) )
#define DMA2_S4PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 4 ) ) )
#define DMA2_S5PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 5 ) ) )
#define DMA2_S6PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 6 ) ) )
#define DMA2_S7PAR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxPAR( 7 ) ) )
#define DMA2_S0M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 0 ) ) )
#define DMA2_S1M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 1 ) ) ) /* DMA2_SxM0AR */
#define DMA2_S2M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 2 ) ) )
#define DMA2_S3M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 3 ) ) )
#define DMA2_S4M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 4 ) ) )
#define DMA2_S5M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 5 ) ) )
#define DMA2_S6M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 6 ) ) )
#define DMA2_S7M0AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM0AR( 7 ) ) )
#define DMA2_S0M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 0 ) ) )
#define DMA2_S1M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 1 ) ) ) /* DMA2_SxM1AR*/
#define DMA2_S2M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 2 ) ) )
#define DMA2_S3M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 3 ) ) )
#define DMA2_S4M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 4 ) ) )
#define DMA2_S5M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 5 ) ) )
#define DMA2_S6M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 6 ) ) )
#define DMA2_S7M1AR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxM1AR( 7 ) ) )
#define DMA2_S0FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 0 ) ) )
#define DMA2_S1FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 1 ) ) ) /* DMA2_SxFCR */
#define DMA2_S2FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 2 ) ) )
#define DMA2_S3FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 3 ) ) )
#define DMA2_S4FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 4 ) ) )
#define DMA2_S5FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 5 ) ) )
#define DMA2_S6FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 6 ) ) )
#define DMA2_S7FCR ( *( volatile uint32_t * )( DMA2_BASE + DMA_OFFSET_SxFCR( 7 ) ) )

  /*------------------------------------------------
  DMAx_SxCR Register
  ------------------------------------------------*/
  static constexpr uint32_t DMAx_SxCR_CHSEL_MSK = 0x07;
  static constexpr uint32_t DMAx_SxCR_CHSEL_POS = 25u;
  static constexpr uint32_t DMAx_SxCR_CHSEL     = DMAx_SxCR_CHSEL_MSK << DMAx_SxCR_CHSEL_POS;

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

    static constexpr uint8_t S_NUM_DMA_REQUESTORS = 79;

    /* clang-format off */
    #if defined( STM32F4 )
    /*------------------------------------------------
    Taken from tables 28 & 29 on RM0390 datasheet
    ------------------------------------------------*/
    constexpr std::array<std::array<uint8_t, 8>, 8> dma1RequestMapping = {{
      { S_SPI3_RX,    S_SPDIFRX_DT,   S_SPI3_RX,      S_SPI2_RX,    S_SPI2_TX,      S_SPI3_TX,      S_SPDIFRX_CS,   S_SPI3_TX   },
      { S_I2C1_RX,    S_I2C3_RX,      S_TIM7_UP,      NONE,         S_TIM7_UP,      S_I2C1_RX,      S_I2C1_TX,      S_I2C1_TX   },
      { S_TIM4_CH1,   NONE,           S_FMPI2C1_RX,   S_TIM4_CH2,   NONE,           S_FMPI2C1_RX,   S_TIM4_UP,      S_TIM4_CH3  },
      { NONE,         S_TIM2_CH3,     S_I2C3_RX,      NONE,         S_I2C3_TX,      S_TIM2_CH1,     S_TIM2_CH2,     S_TIM2_CH4  },
      { S_UART5_RX,   S_USART3_RX,    S_UART4_RX,     S_USART3_TX,  S_UART4_TX,     S_USART2_RX,    S_USART2_TX,    S_UART5_TX  },
      { NONE,         NONE,           S_TIM3_CH4,     NONE,         S_TIM3_CH1,     S_TIM3_CH2,     NONE,           S_TIM3_CH3  },
      { S_TIM5_CH3,   S_TIM5_CH4,     S_TIM5_CH1,     S_TIM5_TRIG,  S_TIM5_CH2,     NONE,           S_TIM5_UP,      NONE        },
      { NONE,         S_TIM6_UP,      S_I2C2_RX,      S_I2C2_RX,    S_USART3_TX,    S_DAC1,         S_DAC2,         S_I2C2_TX   }
    }};

    constexpr std::array<std::array<uint8_t, 8>, 8> dma2RequestMapping = {{
      { S_ADC1,       S_SAI1_A,       S_TIM8_CH1,     S_SAI1_A,     S_ADC1,         S_SAI1_B,       S_TIM1_CH1,     S_SAI2_B    },
      { NONE,         S_DCMI,         S_ADC2,         S_ADC2,       S_SAI1_B,       NONE,           NONE,           S_DCMI      },
      { S_ADC3,       S_ADC3,         NONE,           NONE,         NONE,           NONE,           NONE,           NONE        },
      { S_SPI1_RX,    NONE,           S_SPI1_RX,      S_SPI1_TX,    S_SAI2_A,       S_SPI1_TX,      S_SAI2_B,       S_QUADSPI   },
      { S_SPI4_RX,    S_SPI4_TX,      S_USART1_RX,    S_SDIO,       NONE,           S_USART1_RX,    S_SDIO,         S_USART1_TX },
      { NONE,         S_USART6_RX,    S_USART6_RX,    S_SPI4_RX,    S_SPI4_TX,      NONE,           S_USART6_TX,    S_USART6_TX },
      { S_TIM1_TRIG,  S_TIM1_CH1,     S_TIM1_CH2,     S_TIM1_CH1,   S_TIM1_CH4,     S_TIM1_UP,      S_TIM1_CH3,     NONE        },
      { NONE,         S_TIM8_UP,      S_TIM8_CH1,     S_TIM8_CH2,   S_TIM8_CH3,     NONE,           NONE,           S_TIM8_CH4  }
    }};
    #endif
    /* clang-format on */

  }    // namespace Source
}    // namespace Thor::DMA

#endif /* !THOR_DMA_DEFS_HPP */