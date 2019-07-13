/********************************************************************************
 *   File Name:
 *    hw_gpio_register_stm32f446xx.hpp
 *
 *   Description:
 *    GPIO register definitions for the STM32F446xx series chips.
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_GPIO_REGISTER_HPP
#define THOR_HW_GPIO_REGISTER_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/drivers/f4/system/sys_memory_map_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_GPIO == 1 )

namespace Thor::Driver::GPIO
{
  static constexpr uint32_t GPIOA_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x0000U;
  static constexpr uint32_t GPIOB_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x0400U;
  static constexpr uint32_t GPIOC_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x0800U;
  static constexpr uint32_t GPIOD_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x0C00U;
  static constexpr uint32_t GPIOE_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x1000U;
  static constexpr uint32_t GPIOF_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x1400U;
  static constexpr uint32_t GPIOG_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x1800U;
  static constexpr uint32_t GPIOH_BASE_ADDR = Thor::System::MemoryMap::AHB1PERIPH_BASE_ADDR + 0x1C00U;

  static constexpr size_t NUM_GPIO_PERIPHS = 8;

  static constexpr std::array<uint32_t, NUM_GPIO_PERIPHS> periphAddressList = { GPIOA_BASE_ADDR, GPIOB_BASE_ADDR,
                                                                                GPIOC_BASE_ADDR, GPIOD_BASE_ADDR,
                                                                                GPIOE_BASE_ADDR, GPIOF_BASE_ADDR,
                                                                                GPIOG_BASE_ADDR, GPIOH_BASE_ADDR };


  /*------------------------------------------------
  Alternate Functions
  ------------------------------------------------*/
  static constexpr uint8_t AF0_RTC_50Hz   = 0x00; /* RTC_50Hz Alternate Function mapping                       */
  static constexpr uint8_t AF0_MCO        = 0x00; /* MCO (MCO1 and MCO2) Alternate Function mapping            */
  static constexpr uint8_t AF0_TAMPER     = 0x00; /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
  static constexpr uint8_t AF0_SWJ        = 0x00; /* SWJ (SWD and JTAG) Alternate Function mapping             */
  static constexpr uint8_t AF0_TRACE      = 0x00; /* TRACE Alternate Function mapping                          */
  static constexpr uint8_t AF1_TIM1       = 0x01; /* TIM1 Alternate Function mapping */
  static constexpr uint8_t AF1_TIM2       = 0x01; /* TIM2 Alternate Function mapping */
  static constexpr uint8_t AF2_TIM3       = 0x02; /* TIM3 Alternate Function mapping */
  static constexpr uint8_t AF2_TIM4       = 0x02; /* TIM4 Alternate Function mapping */
  static constexpr uint8_t AF2_TIM5       = 0x02; /* TIM5 Alternate Function mapping */
  static constexpr uint8_t AF3_TIM8       = 0x03; /* TIM8 Alternate Function mapping  */
  static constexpr uint8_t AF3_TIM9       = 0x03; /* TIM9 Alternate Function mapping  */
  static constexpr uint8_t AF3_TIM10      = 0x03; /* TIM10 Alternate Function mapping */
  static constexpr uint8_t AF3_TIM11      = 0x03; /* TIM11 Alternate Function mapping */
  static constexpr uint8_t AF3_CEC        = 0x03; /* CEC Alternate Function mapping   */
  static constexpr uint8_t AF4_I2C1       = 0x04; /* I2C1 Alternate Function mapping */
  static constexpr uint8_t AF4_I2C2       = 0x04; /* I2C2 Alternate Function mapping */
  static constexpr uint8_t AF4_I2C3       = 0x04; /* I2C3 Alternate Function mapping */
  static constexpr uint8_t AF4_FMPI2C1    = 0x04; /* FMPI2C1 Alternate Function mapping */
  static constexpr uint8_t AF4_CEC        = 0x04; /* CEC Alternate Function mapping  */
  static constexpr uint8_t AF5_SPI1       = 0x05; /* SPI1/I2S1 Alternate Function mapping   */
  static constexpr uint8_t AF5_SPI2       = 0x05; /* SPI2/I2S2 Alternate Function mapping   */
  static constexpr uint8_t AF5_SPI3       = 0x05; /* SPI3/I2S3 Alternate Function mapping   */
  static constexpr uint8_t AF5_SPI4       = 0x05; /* SPI4 Alternate Function mapping        */
  static constexpr uint8_t AF6_SPI2       = 0x06; /* SPI2/I2S2 Alternate Function mapping  */
  static constexpr uint8_t AF6_SPI3       = 0x06; /* SPI3/I2S3 Alternate Function mapping  */
  static constexpr uint8_t AF6_SPI4       = 0x06; /* SPI4 Alternate Function mapping       */
  static constexpr uint8_t AF6_SAI1       = 0x06; /* SAI1 Alternate Function mapping       */
  static constexpr uint8_t AF7_USART1     = 0x07; /* USART1 Alternate Function mapping     */
  static constexpr uint8_t AF7_USART2     = 0x07; /* USART2 Alternate Function mapping     */
  static constexpr uint8_t AF7_USART3     = 0x07; /* USART3 Alternate Function mapping     */
  static constexpr uint8_t AF7_UART5      = 0x07; /* UART5 Alternate Function mapping      */
  static constexpr uint8_t AF7_SPI2       = 0x07; /* SPI2/I2S2 Alternate Function mapping  */
  static constexpr uint8_t AF7_SPI3       = 0x07; /* SPI3/I2S3 Alternate Function mapping  */
  static constexpr uint8_t AF7_SPDIFRX    = 0x07; /* SPDIFRX Alternate Function mapping      */
  static constexpr uint8_t AF8_UART4      = 0x08; /* UART4 Alternate Function mapping  */
  static constexpr uint8_t AF8_UART5      = 0x08; /* UART5 Alternate Function mapping  */
  static constexpr uint8_t AF8_USART6     = 0x08; /* USART6 Alternate Function mapping */
  static constexpr uint8_t AF8_SPDIFRX    = 0x08; /* SPDIFRX Alternate Function mapping  */
  static constexpr uint8_t AF8_SAI2       = 0x08; /* SAI2 Alternate Function mapping   */
  static constexpr uint8_t AF9_CAN1       = 0x09; /* CAN1 Alternate Function mapping  */
  static constexpr uint8_t AF9_CAN2       = 0x09; /* CAN2 Alternate Function mapping  */
  static constexpr uint8_t AF9_TIM12      = 0x09; /* TIM12 Alternate Function mapping */
  static constexpr uint8_t AF9_TIM13      = 0x09; /* TIM13 Alternate Function mapping */
  static constexpr uint8_t AF9_TIM14      = 0x09; /* TIM14 Alternate Function mapping */
  static constexpr uint8_t AF9_QSPI       = 0x09; /* QSPI Alternate Function mapping  */
  static constexpr uint8_t AF10_OTG_FS    = 0x0A; /* OTG_FS Alternate Function mapping */
  static constexpr uint8_t AF10_OTG_HS    = 0x0A; /* OTG_HS Alternate Function mapping */
  static constexpr uint8_t AF10_SAI2      = 0x0A; /* SAI2 Alternate Function mapping   */
  static constexpr uint8_t AF10_QSPI      = 0x0A; /* QSPI Alternate Function mapping  */
  static constexpr uint8_t AF11_ETH       = 0x0B; /* ETHERNET Alternate Function mapping */
  static constexpr uint8_t AF12_FMC       = 0x0C; /* FMC Alternate Function mapping                      */
  static constexpr uint8_t AF12_OTG_HS_FS = 0x0C; /* OTG HS configured in FS, Alternate Function mapping */
  static constexpr uint8_t AF12_SDIO      = 0x0C; /* SDIO Alternate Function mapping                     */
  static constexpr uint8_t AF13_DCMI      = 0x0D; /* DCMI Alternate Function mapping */
  static constexpr uint8_t AF15_EVENTOUT  = 0x0F; /* EVENTOUT Alternate Function mapping */

  static constexpr uint8_t AF_NONE = 0xFF;

  /*------------------------------------------------
  MODER
  ------------------------------------------------*/
  static constexpr uint32_t MODER_CFG_X_WID = 0x02u;
  static constexpr uint32_t MODER_CFG_X_MSK = 0x03u;

  static constexpr uint32_t MODER_MODE0_Pos  = ( 0U );
  static constexpr uint32_t MODER_MODE0_Msk  = ( 0x3U << MODER_MODE0_Pos );
  static constexpr uint32_t MODER_MODE0      = MODER_MODE0_Msk;
  static constexpr uint32_t MODER_MODE0_0    = ( 0x1U << MODER_MODE0_Pos );
  static constexpr uint32_t MODER_MODE0_1    = ( 0x2U << MODER_MODE0_Pos );
  static constexpr uint32_t MODER_MODE1_Pos  = ( 2U );
  static constexpr uint32_t MODER_MODE1_Msk  = ( 0x3U << MODER_MODE1_Pos );
  static constexpr uint32_t MODER_MODE1      = MODER_MODE1_Msk;
  static constexpr uint32_t MODER_MODE1_0    = ( 0x1U << MODER_MODE1_Pos );
  static constexpr uint32_t MODER_MODE1_1    = ( 0x2U << MODER_MODE1_Pos );
  static constexpr uint32_t MODER_MODE2_Pos  = ( 4U );
  static constexpr uint32_t MODER_MODE2_Msk  = ( 0x3U << MODER_MODE2_Pos );
  static constexpr uint32_t MODER_MODE2      = MODER_MODE2_Msk;
  static constexpr uint32_t MODER_MODE2_0    = ( 0x1U << MODER_MODE2_Pos );
  static constexpr uint32_t MODER_MODE2_1    = ( 0x2U << MODER_MODE2_Pos );
  static constexpr uint32_t MODER_MODE3_Pos  = ( 6U );
  static constexpr uint32_t MODER_MODE3_Msk  = ( 0x3U << MODER_MODE3_Pos );
  static constexpr uint32_t MODER_MODE3      = MODER_MODE3_Msk;
  static constexpr uint32_t MODER_MODE3_0    = ( 0x1U << MODER_MODE3_Pos );
  static constexpr uint32_t MODER_MODE3_1    = ( 0x2U << MODER_MODE3_Pos );
  static constexpr uint32_t MODER_MODE4_Pos  = ( 8U );
  static constexpr uint32_t MODER_MODE4_Msk  = ( 0x3U << MODER_MODE4_Pos );
  static constexpr uint32_t MODER_MODE4      = MODER_MODE4_Msk;
  static constexpr uint32_t MODER_MODE4_0    = ( 0x1U << MODER_MODE4_Pos );
  static constexpr uint32_t MODER_MODE4_1    = ( 0x2U << MODER_MODE4_Pos );
  static constexpr uint32_t MODER_MODE5_Pos  = ( 10U );
  static constexpr uint32_t MODER_MODE5_Msk  = ( 0x3U << MODER_MODE5_Pos );
  static constexpr uint32_t MODER_MODE5      = MODER_MODE5_Msk;
  static constexpr uint32_t MODER_MODE5_0    = ( 0x1U << MODER_MODE5_Pos );
  static constexpr uint32_t MODER_MODE5_1    = ( 0x2U << MODER_MODE5_Pos );
  static constexpr uint32_t MODER_MODE6_Pos  = ( 12U );
  static constexpr uint32_t MODER_MODE6_Msk  = ( 0x3U << MODER_MODE6_Pos );
  static constexpr uint32_t MODER_MODE6      = MODER_MODE6_Msk;
  static constexpr uint32_t MODER_MODE6_0    = ( 0x1U << MODER_MODE6_Pos );
  static constexpr uint32_t MODER_MODE6_1    = ( 0x2U << MODER_MODE6_Pos );
  static constexpr uint32_t MODER_MODE7_Pos  = ( 14U );
  static constexpr uint32_t MODER_MODE7_Msk  = ( 0x3U << MODER_MODE7_Pos );
  static constexpr uint32_t MODER_MODE7      = MODER_MODE7_Msk;
  static constexpr uint32_t MODER_MODE7_0    = ( 0x1U << MODER_MODE7_Pos );
  static constexpr uint32_t MODER_MODE7_1    = ( 0x2U << MODER_MODE7_Pos );
  static constexpr uint32_t MODER_MODE8_Pos  = ( 16U );
  static constexpr uint32_t MODER_MODE8_Msk  = ( 0x3U << MODER_MODE8_Pos );
  static constexpr uint32_t MODER_MODE8      = MODER_MODE8_Msk;
  static constexpr uint32_t MODER_MODE8_0    = ( 0x1U << MODER_MODE8_Pos );
  static constexpr uint32_t MODER_MODE8_1    = ( 0x2U << MODER_MODE8_Pos );
  static constexpr uint32_t MODER_MODE9_Pos  = ( 18U );
  static constexpr uint32_t MODER_MODE9_Msk  = ( 0x3U << MODER_MODE9_Pos );
  static constexpr uint32_t MODER_MODE9      = MODER_MODE9_Msk;
  static constexpr uint32_t MODER_MODE9_0    = ( 0x1U << MODER_MODE9_Pos );
  static constexpr uint32_t MODER_MODE9_1    = ( 0x2U << MODER_MODE9_Pos );
  static constexpr uint32_t MODER_MODE10_Pos = ( 20U );
  static constexpr uint32_t MODER_MODE10_Msk = ( 0x3U << MODER_MODE10_Pos );
  static constexpr uint32_t MODER_MODE10     = MODER_MODE10_Msk;
  static constexpr uint32_t MODER_MODE10_0   = ( 0x1U << MODER_MODE10_Pos );
  static constexpr uint32_t MODER_MODE10_1   = ( 0x2U << MODER_MODE10_Pos );
  static constexpr uint32_t MODER_MODE11_Pos = ( 22U );
  static constexpr uint32_t MODER_MODE11_Msk = ( 0x3U << MODER_MODE11_Pos );
  static constexpr uint32_t MODER_MODE11     = MODER_MODE11_Msk;
  static constexpr uint32_t MODER_MODE11_0   = ( 0x1U << MODER_MODE11_Pos );
  static constexpr uint32_t MODER_MODE11_1   = ( 0x2U << MODER_MODE11_Pos );
  static constexpr uint32_t MODER_MODE12_Pos = ( 24U );
  static constexpr uint32_t MODER_MODE12_Msk = ( 0x3U << MODER_MODE12_Pos );
  static constexpr uint32_t MODER_MODE12     = MODER_MODE12_Msk;
  static constexpr uint32_t MODER_MODE12_0   = ( 0x1U << MODER_MODE12_Pos );
  static constexpr uint32_t MODER_MODE12_1   = ( 0x2U << MODER_MODE12_Pos );
  static constexpr uint32_t MODER_MODE13_Pos = ( 26U );
  static constexpr uint32_t MODER_MODE13_Msk = ( 0x3U << MODER_MODE13_Pos );
  static constexpr uint32_t MODER_MODE13     = MODER_MODE13_Msk;
  static constexpr uint32_t MODER_MODE13_0   = ( 0x1U << MODER_MODE13_Pos );
  static constexpr uint32_t MODER_MODE13_1   = ( 0x2U << MODER_MODE13_Pos );
  static constexpr uint32_t MODER_MODE14_Pos = ( 28U );
  static constexpr uint32_t MODER_MODE14_Msk = ( 0x3U << MODER_MODE14_Pos );
  static constexpr uint32_t MODER_MODE14     = MODER_MODE14_Msk;
  static constexpr uint32_t MODER_MODE14_0   = ( 0x1U << MODER_MODE14_Pos );
  static constexpr uint32_t MODER_MODE14_1   = ( 0x2U << MODER_MODE14_Pos );
  static constexpr uint32_t MODER_MODE15_Pos = ( 30U );
  static constexpr uint32_t MODER_MODE15_Msk = ( 0x3U << MODER_MODE15_Pos );
  static constexpr uint32_t MODER_MODE15     = MODER_MODE15_Msk;
  static constexpr uint32_t MODER_MODE15_0   = ( 0x1U << MODER_MODE15_Pos );
  static constexpr uint32_t MODER_MODE15_1   = ( 0x2U << MODER_MODE15_Pos );

  /*------------------------------------------------
  OTYPER
  ------------------------------------------------*/
  static constexpr uint32_t OTYPER_OT0_Pos  = ( 0U );
  static constexpr uint32_t OTYPER_OT0_Msk  = ( 0x1U << OTYPER_OT0_Pos );
  static constexpr uint32_t OTYPER_OT0      = OTYPER_OT0_Msk;
  static constexpr uint32_t OTYPER_OT1_Pos  = ( 1U );
  static constexpr uint32_t OTYPER_OT1_Msk  = ( 0x1U << OTYPER_OT1_Pos );
  static constexpr uint32_t OTYPER_OT1      = OTYPER_OT1_Msk;
  static constexpr uint32_t OTYPER_OT2_Pos  = ( 2U );
  static constexpr uint32_t OTYPER_OT2_Msk  = ( 0x1U << OTYPER_OT2_Pos );
  static constexpr uint32_t OTYPER_OT2      = OTYPER_OT2_Msk;
  static constexpr uint32_t OTYPER_OT3_Pos  = ( 3U );
  static constexpr uint32_t OTYPER_OT3_Msk  = ( 0x1U << OTYPER_OT3_Pos );
  static constexpr uint32_t OTYPER_OT3      = OTYPER_OT3_Msk;
  static constexpr uint32_t OTYPER_OT4_Pos  = ( 4U );
  static constexpr uint32_t OTYPER_OT4_Msk  = ( 0x1U << OTYPER_OT4_Pos );
  static constexpr uint32_t OTYPER_OT4      = OTYPER_OT4_Msk;
  static constexpr uint32_t OTYPER_OT5_Pos  = ( 5U );
  static constexpr uint32_t OTYPER_OT5_Msk  = ( 0x1U << OTYPER_OT5_Pos );
  static constexpr uint32_t OTYPER_OT5      = OTYPER_OT5_Msk;
  static constexpr uint32_t OTYPER_OT6_Pos  = ( 6U );
  static constexpr uint32_t OTYPER_OT6_Msk  = ( 0x1U << OTYPER_OT6_Pos );
  static constexpr uint32_t OTYPER_OT6      = OTYPER_OT6_Msk;
  static constexpr uint32_t OTYPER_OT7_Pos  = ( 7U );
  static constexpr uint32_t OTYPER_OT7_Msk  = ( 0x1U << OTYPER_OT7_Pos );
  static constexpr uint32_t OTYPER_OT7      = OTYPER_OT7_Msk;
  static constexpr uint32_t OTYPER_OT8_Pos  = ( 8U );
  static constexpr uint32_t OTYPER_OT8_Msk  = ( 0x1U << OTYPER_OT8_Pos );
  static constexpr uint32_t OTYPER_OT8      = OTYPER_OT8_Msk;
  static constexpr uint32_t OTYPER_OT9_Pos  = ( 9U );
  static constexpr uint32_t OTYPER_OT9_Msk  = ( 0x1U << OTYPER_OT9_Pos );
  static constexpr uint32_t OTYPER_OT9      = OTYPER_OT9_Msk;
  static constexpr uint32_t OTYPER_OT10_Pos = ( 10U );
  static constexpr uint32_t OTYPER_OT10_Msk = ( 0x1U << OTYPER_OT10_Pos );
  static constexpr uint32_t OTYPER_OT10     = OTYPER_OT10_Msk;
  static constexpr uint32_t OTYPER_OT11_Pos = ( 11U );
  static constexpr uint32_t OTYPER_OT11_Msk = ( 0x1U << OTYPER_OT11_Pos );
  static constexpr uint32_t OTYPER_OT11     = OTYPER_OT11_Msk;
  static constexpr uint32_t OTYPER_OT12_Pos = ( 12U );
  static constexpr uint32_t OTYPER_OT12_Msk = ( 0x1U << OTYPER_OT12_Pos );
  static constexpr uint32_t OTYPER_OT12     = OTYPER_OT12_Msk;
  static constexpr uint32_t OTYPER_OT13_Pos = ( 13U );
  static constexpr uint32_t OTYPER_OT13_Msk = ( 0x1U << OTYPER_OT13_Pos );
  static constexpr uint32_t OTYPER_OT13     = OTYPER_OT13_Msk;
  static constexpr uint32_t OTYPER_OT14_Pos = ( 14U );
  static constexpr uint32_t OTYPER_OT14_Msk = ( 0x1U << OTYPER_OT14_Pos );
  static constexpr uint32_t OTYPER_OT14     = OTYPER_OT14_Msk;
  static constexpr uint32_t OTYPER_OT15_Pos = ( 15U );
  static constexpr uint32_t OTYPER_OT15_Msk = ( 0x1U << OTYPER_OT15_Pos );
  static constexpr uint32_t OTYPER_OT15     = OTYPER_OT15_Msk;

  /*------------------------------------------------
  OSPEEDR
  ------------------------------------------------*/
  static constexpr uint32_t OSPEEDR_CFG_X_WID = 0x02u;
  static constexpr uint32_t OSPEEDR_CFG_X_MSK = 0x03u;

  static constexpr uint32_t OSPEEDR_OSPEED0_Pos  = ( 0U );
  static constexpr uint32_t OSPEEDR_OSPEED0_Msk  = ( 0x3U << OSPEEDR_OSPEED0_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED0      = OSPEEDR_OSPEED0_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED0_0    = ( 0x1U << OSPEEDR_OSPEED0_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED0_1    = ( 0x2U << OSPEEDR_OSPEED0_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED1_Pos  = ( 2U );
  static constexpr uint32_t OSPEEDR_OSPEED1_Msk  = ( 0x3U << OSPEEDR_OSPEED1_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED1      = OSPEEDR_OSPEED1_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED1_0    = ( 0x1U << OSPEEDR_OSPEED1_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED1_1    = ( 0x2U << OSPEEDR_OSPEED1_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED2_Pos  = ( 4U );
  static constexpr uint32_t OSPEEDR_OSPEED2_Msk  = ( 0x3U << OSPEEDR_OSPEED2_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED2      = OSPEEDR_OSPEED2_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED2_0    = ( 0x1U << OSPEEDR_OSPEED2_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED2_1    = ( 0x2U << OSPEEDR_OSPEED2_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED3_Pos  = ( 6U );
  static constexpr uint32_t OSPEEDR_OSPEED3_Msk  = ( 0x3U << OSPEEDR_OSPEED3_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED3      = OSPEEDR_OSPEED3_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED3_0    = ( 0x1U << OSPEEDR_OSPEED3_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED3_1    = ( 0x2U << OSPEEDR_OSPEED3_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED4_Pos  = ( 8U );
  static constexpr uint32_t OSPEEDR_OSPEED4_Msk  = ( 0x3U << OSPEEDR_OSPEED4_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED4      = OSPEEDR_OSPEED4_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED4_0    = ( 0x1U << OSPEEDR_OSPEED4_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED4_1    = ( 0x2U << OSPEEDR_OSPEED4_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED5_Pos  = ( 10U );
  static constexpr uint32_t OSPEEDR_OSPEED5_Msk  = ( 0x3U << OSPEEDR_OSPEED5_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED5      = OSPEEDR_OSPEED5_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED5_0    = ( 0x1U << OSPEEDR_OSPEED5_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED5_1    = ( 0x2U << OSPEEDR_OSPEED5_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED6_Pos  = ( 12U );
  static constexpr uint32_t OSPEEDR_OSPEED6_Msk  = ( 0x3U << OSPEEDR_OSPEED6_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED6      = OSPEEDR_OSPEED6_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED6_0    = ( 0x1U << OSPEEDR_OSPEED6_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED6_1    = ( 0x2U << OSPEEDR_OSPEED6_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED7_Pos  = ( 14U );
  static constexpr uint32_t OSPEEDR_OSPEED7_Msk  = ( 0x3U << OSPEEDR_OSPEED7_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED7      = OSPEEDR_OSPEED7_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED7_0    = ( 0x1U << OSPEEDR_OSPEED7_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED7_1    = ( 0x2U << OSPEEDR_OSPEED7_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED8_Pos  = ( 16U );
  static constexpr uint32_t OSPEEDR_OSPEED8_Msk  = ( 0x3U << OSPEEDR_OSPEED8_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED8      = OSPEEDR_OSPEED8_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED8_0    = ( 0x1U << OSPEEDR_OSPEED8_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED8_1    = ( 0x2U << OSPEEDR_OSPEED8_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED9_Pos  = ( 18U );
  static constexpr uint32_t OSPEEDR_OSPEED9_Msk  = ( 0x3U << OSPEEDR_OSPEED9_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED9      = OSPEEDR_OSPEED9_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED9_0    = ( 0x1U << OSPEEDR_OSPEED9_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED9_1    = ( 0x2U << OSPEEDR_OSPEED9_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED10_Pos = ( 20U );
  static constexpr uint32_t OSPEEDR_OSPEED10_Msk = ( 0x3U << OSPEEDR_OSPEED10_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED10     = OSPEEDR_OSPEED10_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED10_0   = ( 0x1U << OSPEEDR_OSPEED10_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED10_1   = ( 0x2U << OSPEEDR_OSPEED10_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED11_Pos = ( 22U );
  static constexpr uint32_t OSPEEDR_OSPEED11_Msk = ( 0x3U << OSPEEDR_OSPEED11_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED11     = OSPEEDR_OSPEED11_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED11_0   = ( 0x1U << OSPEEDR_OSPEED11_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED11_1   = ( 0x2U << OSPEEDR_OSPEED11_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED12_Pos = ( 24U );
  static constexpr uint32_t OSPEEDR_OSPEED12_Msk = ( 0x3U << OSPEEDR_OSPEED12_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED12     = OSPEEDR_OSPEED12_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED12_0   = ( 0x1U << OSPEEDR_OSPEED12_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED12_1   = ( 0x2U << OSPEEDR_OSPEED12_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED13_Pos = ( 26U );
  static constexpr uint32_t OSPEEDR_OSPEED13_Msk = ( 0x3U << OSPEEDR_OSPEED13_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED13     = OSPEEDR_OSPEED13_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED13_0   = ( 0x1U << OSPEEDR_OSPEED13_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED13_1   = ( 0x2U << OSPEEDR_OSPEED13_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED14_Pos = ( 28U );
  static constexpr uint32_t OSPEEDR_OSPEED14_Msk = ( 0x3U << OSPEEDR_OSPEED14_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED14     = OSPEEDR_OSPEED14_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED14_0   = ( 0x1U << OSPEEDR_OSPEED14_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED14_1   = ( 0x2U << OSPEEDR_OSPEED14_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED15_Pos = ( 30U );
  static constexpr uint32_t OSPEEDR_OSPEED15_Msk = ( 0x3U << OSPEEDR_OSPEED15_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED15     = OSPEEDR_OSPEED15_Msk;
  static constexpr uint32_t OSPEEDR_OSPEED15_0   = ( 0x1U << OSPEEDR_OSPEED15_Pos );
  static constexpr uint32_t OSPEEDR_OSPEED15_1   = ( 0x2U << OSPEEDR_OSPEED15_Pos );

  /*------------------------------------------------
  PUPDR
  ------------------------------------------------*/
  static constexpr uint32_t PUPDR_CFG_X_WID = 0x02u;
  static constexpr uint32_t PUPDR_CFG_X_MSK = 0x03u;

  static constexpr uint32_t PUPDR_PUPD0_Pos  = ( 0U );
  static constexpr uint32_t PUPDR_PUPD0_Msk  = ( 0x3U << PUPDR_PUPD0_Pos );
  static constexpr uint32_t PUPDR_PUPD0      = PUPDR_PUPD0_Msk;
  static constexpr uint32_t PUPDR_PUPD0_0    = ( 0x1U << PUPDR_PUPD0_Pos );
  static constexpr uint32_t PUPDR_PUPD0_1    = ( 0x2U << PUPDR_PUPD0_Pos );
  static constexpr uint32_t PUPDR_PUPD1_Pos  = ( 2U );
  static constexpr uint32_t PUPDR_PUPD1_Msk  = ( 0x3U << PUPDR_PUPD1_Pos );
  static constexpr uint32_t PUPDR_PUPD1      = PUPDR_PUPD1_Msk;
  static constexpr uint32_t PUPDR_PUPD1_0    = ( 0x1U << PUPDR_PUPD1_Pos );
  static constexpr uint32_t PUPDR_PUPD1_1    = ( 0x2U << PUPDR_PUPD1_Pos );
  static constexpr uint32_t PUPDR_PUPD2_Pos  = ( 4U );
  static constexpr uint32_t PUPDR_PUPD2_Msk  = ( 0x3U << PUPDR_PUPD2_Pos );
  static constexpr uint32_t PUPDR_PUPD2      = PUPDR_PUPD2_Msk;
  static constexpr uint32_t PUPDR_PUPD2_0    = ( 0x1U << PUPDR_PUPD2_Pos );
  static constexpr uint32_t PUPDR_PUPD2_1    = ( 0x2U << PUPDR_PUPD2_Pos );
  static constexpr uint32_t PUPDR_PUPD3_Pos  = ( 6U );
  static constexpr uint32_t PUPDR_PUPD3_Msk  = ( 0x3U << PUPDR_PUPD3_Pos );
  static constexpr uint32_t PUPDR_PUPD3      = PUPDR_PUPD3_Msk;
  static constexpr uint32_t PUPDR_PUPD3_0    = ( 0x1U << PUPDR_PUPD3_Pos );
  static constexpr uint32_t PUPDR_PUPD3_1    = ( 0x2U << PUPDR_PUPD3_Pos );
  static constexpr uint32_t PUPDR_PUPD4_Pos  = ( 8U );
  static constexpr uint32_t PUPDR_PUPD4_Msk  = ( 0x3U << PUPDR_PUPD4_Pos );
  static constexpr uint32_t PUPDR_PUPD4      = PUPDR_PUPD4_Msk;
  static constexpr uint32_t PUPDR_PUPD4_0    = ( 0x1U << PUPDR_PUPD4_Pos );
  static constexpr uint32_t PUPDR_PUPD4_1    = ( 0x2U << PUPDR_PUPD4_Pos );
  static constexpr uint32_t PUPDR_PUPD5_Pos  = ( 10U );
  static constexpr uint32_t PUPDR_PUPD5_Msk  = ( 0x3U << PUPDR_PUPD5_Pos );
  static constexpr uint32_t PUPDR_PUPD5      = PUPDR_PUPD5_Msk;
  static constexpr uint32_t PUPDR_PUPD5_0    = ( 0x1U << PUPDR_PUPD5_Pos );
  static constexpr uint32_t PUPDR_PUPD5_1    = ( 0x2U << PUPDR_PUPD5_Pos );
  static constexpr uint32_t PUPDR_PUPD6_Pos  = ( 12U );
  static constexpr uint32_t PUPDR_PUPD6_Msk  = ( 0x3U << PUPDR_PUPD6_Pos );
  static constexpr uint32_t PUPDR_PUPD6      = PUPDR_PUPD6_Msk;
  static constexpr uint32_t PUPDR_PUPD6_0    = ( 0x1U << PUPDR_PUPD6_Pos );
  static constexpr uint32_t PUPDR_PUPD6_1    = ( 0x2U << PUPDR_PUPD6_Pos );
  static constexpr uint32_t PUPDR_PUPD7_Pos  = ( 14U );
  static constexpr uint32_t PUPDR_PUPD7_Msk  = ( 0x3U << PUPDR_PUPD7_Pos );
  static constexpr uint32_t PUPDR_PUPD7      = PUPDR_PUPD7_Msk;
  static constexpr uint32_t PUPDR_PUPD7_0    = ( 0x1U << PUPDR_PUPD7_Pos );
  static constexpr uint32_t PUPDR_PUPD7_1    = ( 0x2U << PUPDR_PUPD7_Pos );
  static constexpr uint32_t PUPDR_PUPD8_Pos  = ( 16U );
  static constexpr uint32_t PUPDR_PUPD8_Msk  = ( 0x3U << PUPDR_PUPD8_Pos );
  static constexpr uint32_t PUPDR_PUPD8      = PUPDR_PUPD8_Msk;
  static constexpr uint32_t PUPDR_PUPD8_0    = ( 0x1U << PUPDR_PUPD8_Pos );
  static constexpr uint32_t PUPDR_PUPD8_1    = ( 0x2U << PUPDR_PUPD8_Pos );
  static constexpr uint32_t PUPDR_PUPD9_Pos  = ( 18U );
  static constexpr uint32_t PUPDR_PUPD9_Msk  = ( 0x3U << PUPDR_PUPD9_Pos );
  static constexpr uint32_t PUPDR_PUPD9      = PUPDR_PUPD9_Msk;
  static constexpr uint32_t PUPDR_PUPD9_0    = ( 0x1U << PUPDR_PUPD9_Pos );
  static constexpr uint32_t PUPDR_PUPD9_1    = ( 0x2U << PUPDR_PUPD9_Pos );
  static constexpr uint32_t PUPDR_PUPD10_Pos = ( 20U );
  static constexpr uint32_t PUPDR_PUPD10_Msk = ( 0x3U << PUPDR_PUPD10_Pos );
  static constexpr uint32_t PUPDR_PUPD10     = PUPDR_PUPD10_Msk;
  static constexpr uint32_t PUPDR_PUPD10_0   = ( 0x1U << PUPDR_PUPD10_Pos );
  static constexpr uint32_t PUPDR_PUPD10_1   = ( 0x2U << PUPDR_PUPD10_Pos );
  static constexpr uint32_t PUPDR_PUPD11_Pos = ( 22U );
  static constexpr uint32_t PUPDR_PUPD11_Msk = ( 0x3U << PUPDR_PUPD11_Pos );
  static constexpr uint32_t PUPDR_PUPD11     = PUPDR_PUPD11_Msk;
  static constexpr uint32_t PUPDR_PUPD11_0   = ( 0x1U << PUPDR_PUPD11_Pos );
  static constexpr uint32_t PUPDR_PUPD11_1   = ( 0x2U << PUPDR_PUPD11_Pos );
  static constexpr uint32_t PUPDR_PUPD12_Pos = ( 24U );
  static constexpr uint32_t PUPDR_PUPD12_Msk = ( 0x3U << PUPDR_PUPD12_Pos );
  static constexpr uint32_t PUPDR_PUPD12     = PUPDR_PUPD12_Msk;
  static constexpr uint32_t PUPDR_PUPD12_0   = ( 0x1U << PUPDR_PUPD12_Pos );
  static constexpr uint32_t PUPDR_PUPD12_1   = ( 0x2U << PUPDR_PUPD12_Pos );
  static constexpr uint32_t PUPDR_PUPD13_Pos = ( 26U );
  static constexpr uint32_t PUPDR_PUPD13_Msk = ( 0x3U << PUPDR_PUPD13_Pos );
  static constexpr uint32_t PUPDR_PUPD13     = PUPDR_PUPD13_Msk;
  static constexpr uint32_t PUPDR_PUPD13_0   = ( 0x1U << PUPDR_PUPD13_Pos );
  static constexpr uint32_t PUPDR_PUPD13_1   = ( 0x2U << PUPDR_PUPD13_Pos );
  static constexpr uint32_t PUPDR_PUPD14_Pos = ( 28U );
  static constexpr uint32_t PUPDR_PUPD14_Msk = ( 0x3U << PUPDR_PUPD14_Pos );
  static constexpr uint32_t PUPDR_PUPD14     = PUPDR_PUPD14_Msk;
  static constexpr uint32_t PUPDR_PUPD14_0   = ( 0x1U << PUPDR_PUPD14_Pos );
  static constexpr uint32_t PUPDR_PUPD14_1   = ( 0x2U << PUPDR_PUPD14_Pos );
  static constexpr uint32_t PUPDR_PUPD15_Pos = ( 30U );
  static constexpr uint32_t PUPDR_PUPD15_Msk = ( 0x3U << PUPDR_PUPD15_Pos );
  static constexpr uint32_t PUPDR_PUPD15     = PUPDR_PUPD15_Msk;
  static constexpr uint32_t PUPDR_PUPD15_0   = ( 0x1U << PUPDR_PUPD15_Pos );
  static constexpr uint32_t PUPDR_PUPD15_1   = ( 0x2U << PUPDR_PUPD15_Pos );

  /*------------------------------------------------
  IDR
  ------------------------------------------------*/
  static constexpr uint32_t IDR_ID0_Pos  = ( 0U );
  static constexpr uint32_t IDR_ID0_Msk  = ( 0x1U << IDR_ID0_Pos );
  static constexpr uint32_t IDR_ID0      = IDR_ID0_Msk;
  static constexpr uint32_t IDR_ID1_Pos  = ( 1U );
  static constexpr uint32_t IDR_ID1_Msk  = ( 0x1U << IDR_ID1_Pos );
  static constexpr uint32_t IDR_ID1      = IDR_ID1_Msk;
  static constexpr uint32_t IDR_ID2_Pos  = ( 2U );
  static constexpr uint32_t IDR_ID2_Msk  = ( 0x1U << IDR_ID2_Pos );
  static constexpr uint32_t IDR_ID2      = IDR_ID2_Msk;
  static constexpr uint32_t IDR_ID3_Pos  = ( 3U );
  static constexpr uint32_t IDR_ID3_Msk  = ( 0x1U << IDR_ID3_Pos );
  static constexpr uint32_t IDR_ID3      = IDR_ID3_Msk;
  static constexpr uint32_t IDR_ID4_Pos  = ( 4U );
  static constexpr uint32_t IDR_ID4_Msk  = ( 0x1U << IDR_ID4_Pos );
  static constexpr uint32_t IDR_ID4      = IDR_ID4_Msk;
  static constexpr uint32_t IDR_ID5_Pos  = ( 5U );
  static constexpr uint32_t IDR_ID5_Msk  = ( 0x1U << IDR_ID5_Pos );
  static constexpr uint32_t IDR_ID5      = IDR_ID5_Msk;
  static constexpr uint32_t IDR_ID6_Pos  = ( 6U );
  static constexpr uint32_t IDR_ID6_Msk  = ( 0x1U << IDR_ID6_Pos );
  static constexpr uint32_t IDR_ID6      = IDR_ID6_Msk;
  static constexpr uint32_t IDR_ID7_Pos  = ( 7U );
  static constexpr uint32_t IDR_ID7_Msk  = ( 0x1U << IDR_ID7_Pos );
  static constexpr uint32_t IDR_ID7      = IDR_ID7_Msk;
  static constexpr uint32_t IDR_ID8_Pos  = ( 8U );
  static constexpr uint32_t IDR_ID8_Msk  = ( 0x1U << IDR_ID8_Pos );
  static constexpr uint32_t IDR_ID8      = IDR_ID8_Msk;
  static constexpr uint32_t IDR_ID9_Pos  = ( 9U );
  static constexpr uint32_t IDR_ID9_Msk  = ( 0x1U << IDR_ID9_Pos );
  static constexpr uint32_t IDR_ID9      = IDR_ID9_Msk;
  static constexpr uint32_t IDR_ID10_Pos = ( 10U );
  static constexpr uint32_t IDR_ID10_Msk = ( 0x1U << IDR_ID10_Pos );
  static constexpr uint32_t IDR_ID10     = IDR_ID10_Msk;
  static constexpr uint32_t IDR_ID11_Pos = ( 11U );
  static constexpr uint32_t IDR_ID11_Msk = ( 0x1U << IDR_ID11_Pos );
  static constexpr uint32_t IDR_ID11     = IDR_ID11_Msk;
  static constexpr uint32_t IDR_ID12_Pos = ( 12U );
  static constexpr uint32_t IDR_ID12_Msk = ( 0x1U << IDR_ID12_Pos );
  static constexpr uint32_t IDR_ID12     = IDR_ID12_Msk;
  static constexpr uint32_t IDR_ID13_Pos = ( 13U );
  static constexpr uint32_t IDR_ID13_Msk = ( 0x1U << IDR_ID13_Pos );
  static constexpr uint32_t IDR_ID13     = IDR_ID13_Msk;
  static constexpr uint32_t IDR_ID14_Pos = ( 14U );
  static constexpr uint32_t IDR_ID14_Msk = ( 0x1U << IDR_ID14_Pos );
  static constexpr uint32_t IDR_ID14     = IDR_ID14_Msk;
  static constexpr uint32_t IDR_ID15_Pos = ( 15U );
  static constexpr uint32_t IDR_ID15_Msk = ( 0x1U << IDR_ID15_Pos );
  static constexpr uint32_t IDR_ID15     = IDR_ID15_Msk;

  /*------------------------------------------------
  ODR
  ------------------------------------------------*/
  static constexpr uint32_t ODR_OD0_Pos  = ( 0U );
  static constexpr uint32_t ODR_OD0_Msk  = ( 0x1U << ODR_OD0_Pos );
  static constexpr uint32_t ODR_OD0      = ODR_OD0_Msk;
  static constexpr uint32_t ODR_OD1_Pos  = ( 1U );
  static constexpr uint32_t ODR_OD1_Msk  = ( 0x1U << ODR_OD1_Pos );
  static constexpr uint32_t ODR_OD1      = ODR_OD1_Msk;
  static constexpr uint32_t ODR_OD2_Pos  = ( 2U );
  static constexpr uint32_t ODR_OD2_Msk  = ( 0x1U << ODR_OD2_Pos );
  static constexpr uint32_t ODR_OD2      = ODR_OD2_Msk;
  static constexpr uint32_t ODR_OD3_Pos  = ( 3U );
  static constexpr uint32_t ODR_OD3_Msk  = ( 0x1U << ODR_OD3_Pos );
  static constexpr uint32_t ODR_OD3      = ODR_OD3_Msk;
  static constexpr uint32_t ODR_OD4_Pos  = ( 4U );
  static constexpr uint32_t ODR_OD4_Msk  = ( 0x1U << ODR_OD4_Pos );
  static constexpr uint32_t ODR_OD4      = ODR_OD4_Msk;
  static constexpr uint32_t ODR_OD5_Pos  = ( 5U );
  static constexpr uint32_t ODR_OD5_Msk  = ( 0x1U << ODR_OD5_Pos );
  static constexpr uint32_t ODR_OD5      = ODR_OD5_Msk;
  static constexpr uint32_t ODR_OD6_Pos  = ( 6U );
  static constexpr uint32_t ODR_OD6_Msk  = ( 0x1U << ODR_OD6_Pos );
  static constexpr uint32_t ODR_OD6      = ODR_OD6_Msk;
  static constexpr uint32_t ODR_OD7_Pos  = ( 7U );
  static constexpr uint32_t ODR_OD7_Msk  = ( 0x1U << ODR_OD7_Pos );
  static constexpr uint32_t ODR_OD7      = ODR_OD7_Msk;
  static constexpr uint32_t ODR_OD8_Pos  = ( 8U );
  static constexpr uint32_t ODR_OD8_Msk  = ( 0x1U << ODR_OD8_Pos );
  static constexpr uint32_t ODR_OD8      = ODR_OD8_Msk;
  static constexpr uint32_t ODR_OD9_Pos  = ( 9U );
  static constexpr uint32_t ODR_OD9_Msk  = ( 0x1U << ODR_OD9_Pos );
  static constexpr uint32_t ODR_OD9      = ODR_OD9_Msk;
  static constexpr uint32_t ODR_OD10_Pos = ( 10U );
  static constexpr uint32_t ODR_OD10_Msk = ( 0x1U << ODR_OD10_Pos );
  static constexpr uint32_t ODR_OD10     = ODR_OD10_Msk;
  static constexpr uint32_t ODR_OD11_Pos = ( 11U );
  static constexpr uint32_t ODR_OD11_Msk = ( 0x1U << ODR_OD11_Pos );
  static constexpr uint32_t ODR_OD11     = ODR_OD11_Msk;
  static constexpr uint32_t ODR_OD12_Pos = ( 12U );
  static constexpr uint32_t ODR_OD12_Msk = ( 0x1U << ODR_OD12_Pos );
  static constexpr uint32_t ODR_OD12     = ODR_OD12_Msk;
  static constexpr uint32_t ODR_OD13_Pos = ( 13U );
  static constexpr uint32_t ODR_OD13_Msk = ( 0x1U << ODR_OD13_Pos );
  static constexpr uint32_t ODR_OD13     = ODR_OD13_Msk;
  static constexpr uint32_t ODR_OD14_Pos = ( 14U );
  static constexpr uint32_t ODR_OD14_Msk = ( 0x1U << ODR_OD14_Pos );
  static constexpr uint32_t ODR_OD14     = ODR_OD14_Msk;
  static constexpr uint32_t ODR_OD15_Pos = ( 15U );
  static constexpr uint32_t ODR_OD15_Msk = ( 0x1U << ODR_OD15_Pos );
  static constexpr uint32_t ODR_OD15     = ODR_OD15_Msk;

  /*------------------------------------------------
  BSRR
  ------------------------------------------------*/
  static constexpr uint32_t BSRR_BS0_Pos  = ( 0U );
  static constexpr uint32_t BSRR_BS0_Msk  = ( 0x1U << BSRR_BS0_Pos );
  static constexpr uint32_t BSRR_BS0      = BSRR_BS0_Msk;
  static constexpr uint32_t BSRR_BS1_Pos  = ( 1U );
  static constexpr uint32_t BSRR_BS1_Msk  = ( 0x1U << BSRR_BS1_Pos );
  static constexpr uint32_t BSRR_BS1      = BSRR_BS1_Msk;
  static constexpr uint32_t BSRR_BS2_Pos  = ( 2U );
  static constexpr uint32_t BSRR_BS2_Msk  = ( 0x1U << BSRR_BS2_Pos );
  static constexpr uint32_t BSRR_BS2      = BSRR_BS2_Msk;
  static constexpr uint32_t BSRR_BS3_Pos  = ( 3U );
  static constexpr uint32_t BSRR_BS3_Msk  = ( 0x1U << BSRR_BS3_Pos );
  static constexpr uint32_t BSRR_BS3      = BSRR_BS3_Msk;
  static constexpr uint32_t BSRR_BS4_Pos  = ( 4U );
  static constexpr uint32_t BSRR_BS4_Msk  = ( 0x1U << BSRR_BS4_Pos );
  static constexpr uint32_t BSRR_BS4      = BSRR_BS4_Msk;
  static constexpr uint32_t BSRR_BS5_Pos  = ( 5U );
  static constexpr uint32_t BSRR_BS5_Msk  = ( 0x1U << BSRR_BS5_Pos );
  static constexpr uint32_t BSRR_BS5      = BSRR_BS5_Msk;
  static constexpr uint32_t BSRR_BS6_Pos  = ( 6U );
  static constexpr uint32_t BSRR_BS6_Msk  = ( 0x1U << BSRR_BS6_Pos );
  static constexpr uint32_t BSRR_BS6      = BSRR_BS6_Msk;
  static constexpr uint32_t BSRR_BS7_Pos  = ( 7U );
  static constexpr uint32_t BSRR_BS7_Msk  = ( 0x1U << BSRR_BS7_Pos );
  static constexpr uint32_t BSRR_BS7      = BSRR_BS7_Msk;
  static constexpr uint32_t BSRR_BS8_Pos  = ( 8U );
  static constexpr uint32_t BSRR_BS8_Msk  = ( 0x1U << BSRR_BS8_Pos );
  static constexpr uint32_t BSRR_BS8      = BSRR_BS8_Msk;
  static constexpr uint32_t BSRR_BS9_Pos  = ( 9U );
  static constexpr uint32_t BSRR_BS9_Msk  = ( 0x1U << BSRR_BS9_Pos );
  static constexpr uint32_t BSRR_BS9      = BSRR_BS9_Msk;
  static constexpr uint32_t BSRR_BS10_Pos = ( 10U );
  static constexpr uint32_t BSRR_BS10_Msk = ( 0x1U << BSRR_BS10_Pos );
  static constexpr uint32_t BSRR_BS10     = BSRR_BS10_Msk;
  static constexpr uint32_t BSRR_BS11_Pos = ( 11U );
  static constexpr uint32_t BSRR_BS11_Msk = ( 0x1U << BSRR_BS11_Pos );
  static constexpr uint32_t BSRR_BS11     = BSRR_BS11_Msk;
  static constexpr uint32_t BSRR_BS12_Pos = ( 12U );
  static constexpr uint32_t BSRR_BS12_Msk = ( 0x1U << BSRR_BS12_Pos );
  static constexpr uint32_t BSRR_BS12     = BSRR_BS12_Msk;
  static constexpr uint32_t BSRR_BS13_Pos = ( 13U );
  static constexpr uint32_t BSRR_BS13_Msk = ( 0x1U << BSRR_BS13_Pos );
  static constexpr uint32_t BSRR_BS13     = BSRR_BS13_Msk;
  static constexpr uint32_t BSRR_BS14_Pos = ( 14U );
  static constexpr uint32_t BSRR_BS14_Msk = ( 0x1U << BSRR_BS14_Pos );
  static constexpr uint32_t BSRR_BS14     = BSRR_BS14_Msk;
  static constexpr uint32_t BSRR_BS15_Pos = ( 15U );
  static constexpr uint32_t BSRR_BS15_Msk = ( 0x1U << BSRR_BS15_Pos );
  static constexpr uint32_t BSRR_BS15     = BSRR_BS15_Msk;
  static constexpr uint32_t BSRR_BR0_Pos  = ( 16U );
  static constexpr uint32_t BSRR_BR0_Msk  = ( 0x1U << BSRR_BR0_Pos );
  static constexpr uint32_t BSRR_BR0      = BSRR_BR0_Msk;
  static constexpr uint32_t BSRR_BR1_Pos  = ( 17U );
  static constexpr uint32_t BSRR_BR1_Msk  = ( 0x1U << BSRR_BR1_Pos );
  static constexpr uint32_t BSRR_BR1      = BSRR_BR1_Msk;
  static constexpr uint32_t BSRR_BR2_Pos  = ( 18U );
  static constexpr uint32_t BSRR_BR2_Msk  = ( 0x1U << BSRR_BR2_Pos );
  static constexpr uint32_t BSRR_BR2      = BSRR_BR2_Msk;
  static constexpr uint32_t BSRR_BR3_Pos  = ( 19U );
  static constexpr uint32_t BSRR_BR3_Msk  = ( 0x1U << BSRR_BR3_Pos );
  static constexpr uint32_t BSRR_BR3      = BSRR_BR3_Msk;
  static constexpr uint32_t BSRR_BR4_Pos  = ( 20U );
  static constexpr uint32_t BSRR_BR4_Msk  = ( 0x1U << BSRR_BR4_Pos );
  static constexpr uint32_t BSRR_BR4      = BSRR_BR4_Msk;
  static constexpr uint32_t BSRR_BR5_Pos  = ( 21U );
  static constexpr uint32_t BSRR_BR5_Msk  = ( 0x1U << BSRR_BR5_Pos );
  static constexpr uint32_t BSRR_BR5      = BSRR_BR5_Msk;
  static constexpr uint32_t BSRR_BR6_Pos  = ( 22U );
  static constexpr uint32_t BSRR_BR6_Msk  = ( 0x1U << BSRR_BR6_Pos );
  static constexpr uint32_t BSRR_BR6      = BSRR_BR6_Msk;
  static constexpr uint32_t BSRR_BR7_Pos  = ( 23U );
  static constexpr uint32_t BSRR_BR7_Msk  = ( 0x1U << BSRR_BR7_Pos );
  static constexpr uint32_t BSRR_BR7      = BSRR_BR7_Msk;
  static constexpr uint32_t BSRR_BR8_Pos  = ( 24U );
  static constexpr uint32_t BSRR_BR8_Msk  = ( 0x1U << BSRR_BR8_Pos );
  static constexpr uint32_t BSRR_BR8      = BSRR_BR8_Msk;
  static constexpr uint32_t BSRR_BR9_Pos  = ( 25U );
  static constexpr uint32_t BSRR_BR9_Msk  = ( 0x1U << BSRR_BR9_Pos );
  static constexpr uint32_t BSRR_BR9      = BSRR_BR9_Msk;
  static constexpr uint32_t BSRR_BR10_Pos = ( 26U );
  static constexpr uint32_t BSRR_BR10_Msk = ( 0x1U << BSRR_BR10_Pos );
  static constexpr uint32_t BSRR_BR10     = BSRR_BR10_Msk;
  static constexpr uint32_t BSRR_BR11_Pos = ( 27U );
  static constexpr uint32_t BSRR_BR11_Msk = ( 0x1U << BSRR_BR11_Pos );
  static constexpr uint32_t BSRR_BR11     = BSRR_BR11_Msk;
  static constexpr uint32_t BSRR_BR12_Pos = ( 28U );
  static constexpr uint32_t BSRR_BR12_Msk = ( 0x1U << BSRR_BR12_Pos );
  static constexpr uint32_t BSRR_BR12     = BSRR_BR12_Msk;
  static constexpr uint32_t BSRR_BR13_Pos = ( 29U );
  static constexpr uint32_t BSRR_BR13_Msk = ( 0x1U << BSRR_BR13_Pos );
  static constexpr uint32_t BSRR_BR13     = BSRR_BR13_Msk;
  static constexpr uint32_t BSRR_BR14_Pos = ( 30U );
  static constexpr uint32_t BSRR_BR14_Msk = ( 0x1U << BSRR_BR14_Pos );
  static constexpr uint32_t BSRR_BR14     = BSRR_BR14_Msk;
  static constexpr uint32_t BSRR_BR15_Pos = ( 31U );
  static constexpr uint32_t BSRR_BR15_Msk = ( 0x1U << BSRR_BR15_Pos );
  static constexpr uint32_t BSRR_BR15     = BSRR_BR15_Msk;

  /*------------------------------------------------
  LCKR
  ------------------------------------------------*/
  static constexpr uint32_t LCKR_LCK0_Pos  = ( 0U );
  static constexpr uint32_t LCKR_LCK0_Msk  = ( 0x1U << LCKR_LCK0_Pos );
  static constexpr uint32_t LCKR_LCK0      = LCKR_LCK0_Msk;
  static constexpr uint32_t LCKR_LCK1_Pos  = ( 1U );
  static constexpr uint32_t LCKR_LCK1_Msk  = ( 0x1U << LCKR_LCK1_Pos );
  static constexpr uint32_t LCKR_LCK1      = LCKR_LCK1_Msk;
  static constexpr uint32_t LCKR_LCK2_Pos  = ( 2U );
  static constexpr uint32_t LCKR_LCK2_Msk  = ( 0x1U << LCKR_LCK2_Pos );
  static constexpr uint32_t LCKR_LCK2      = LCKR_LCK2_Msk;
  static constexpr uint32_t LCKR_LCK3_Pos  = ( 3U );
  static constexpr uint32_t LCKR_LCK3_Msk  = ( 0x1U << LCKR_LCK3_Pos );
  static constexpr uint32_t LCKR_LCK3      = LCKR_LCK3_Msk;
  static constexpr uint32_t LCKR_LCK4_Pos  = ( 4U );
  static constexpr uint32_t LCKR_LCK4_Msk  = ( 0x1U << LCKR_LCK4_Pos );
  static constexpr uint32_t LCKR_LCK4      = LCKR_LCK4_Msk;
  static constexpr uint32_t LCKR_LCK5_Pos  = ( 5U );
  static constexpr uint32_t LCKR_LCK5_Msk  = ( 0x1U << LCKR_LCK5_Pos );
  static constexpr uint32_t LCKR_LCK5      = LCKR_LCK5_Msk;
  static constexpr uint32_t LCKR_LCK6_Pos  = ( 6U );
  static constexpr uint32_t LCKR_LCK6_Msk  = ( 0x1U << LCKR_LCK6_Pos );
  static constexpr uint32_t LCKR_LCK6      = LCKR_LCK6_Msk;
  static constexpr uint32_t LCKR_LCK7_Pos  = ( 7U );
  static constexpr uint32_t LCKR_LCK7_Msk  = ( 0x1U << LCKR_LCK7_Pos );
  static constexpr uint32_t LCKR_LCK7      = LCKR_LCK7_Msk;
  static constexpr uint32_t LCKR_LCK8_Pos  = ( 8U );
  static constexpr uint32_t LCKR_LCK8_Msk  = ( 0x1U << LCKR_LCK8_Pos );
  static constexpr uint32_t LCKR_LCK8      = LCKR_LCK8_Msk;
  static constexpr uint32_t LCKR_LCK9_Pos  = ( 9U );
  static constexpr uint32_t LCKR_LCK9_Msk  = ( 0x1U << LCKR_LCK9_Pos );
  static constexpr uint32_t LCKR_LCK9      = LCKR_LCK9_Msk;
  static constexpr uint32_t LCKR_LCK10_Pos = ( 10U );
  static constexpr uint32_t LCKR_LCK10_Msk = ( 0x1U << LCKR_LCK10_Pos );
  static constexpr uint32_t LCKR_LCK10     = LCKR_LCK10_Msk;
  static constexpr uint32_t LCKR_LCK11_Pos = ( 11U );
  static constexpr uint32_t LCKR_LCK11_Msk = ( 0x1U << LCKR_LCK11_Pos );
  static constexpr uint32_t LCKR_LCK11     = LCKR_LCK11_Msk;
  static constexpr uint32_t LCKR_LCK12_Pos = ( 12U );
  static constexpr uint32_t LCKR_LCK12_Msk = ( 0x1U << LCKR_LCK12_Pos );
  static constexpr uint32_t LCKR_LCK12     = LCKR_LCK12_Msk;
  static constexpr uint32_t LCKR_LCK13_Pos = ( 13U );
  static constexpr uint32_t LCKR_LCK13_Msk = ( 0x1U << LCKR_LCK13_Pos );
  static constexpr uint32_t LCKR_LCK13     = LCKR_LCK13_Msk;
  static constexpr uint32_t LCKR_LCK14_Pos = ( 14U );
  static constexpr uint32_t LCKR_LCK14_Msk = ( 0x1U << LCKR_LCK14_Pos );
  static constexpr uint32_t LCKR_LCK14     = LCKR_LCK14_Msk;
  static constexpr uint32_t LCKR_LCK15_Pos = ( 15U );
  static constexpr uint32_t LCKR_LCK15_Msk = ( 0x1U << LCKR_LCK15_Pos );
  static constexpr uint32_t LCKR_LCK15     = LCKR_LCK15_Msk;
  static constexpr uint32_t LCKR_LCKK_Pos  = ( 16U );
  static constexpr uint32_t LCKR_LCKK_Msk  = ( 0x1U << LCKR_LCKK_Pos );
  static constexpr uint32_t LCKR_LCKK      = LCKR_LCKK_Msk;

  /*------------------------------------------------
  AFRL
  ------------------------------------------------*/
  static constexpr uint32_t AFR_CFG_X_WID = 0x04u;
  static constexpr uint32_t AFR_CFG_X_MSK = 0x0Fu;

  static constexpr uint32_t AFRL_AFSEL0_Pos = ( 0U );
  static constexpr uint32_t AFRL_AFSEL0_Msk = ( 0xFU << AFRL_AFSEL0_Pos );
  static constexpr uint32_t AFRL_AFSEL0     = AFRL_AFSEL0_Msk;
  static constexpr uint32_t AFRL_AFSEL0_0   = ( 0x1U << AFRL_AFSEL0_Pos );
  static constexpr uint32_t AFRL_AFSEL0_1   = ( 0x2U << AFRL_AFSEL0_Pos );
  static constexpr uint32_t AFRL_AFSEL0_2   = ( 0x4U << AFRL_AFSEL0_Pos );
  static constexpr uint32_t AFRL_AFSEL0_3   = ( 0x8U << AFRL_AFSEL0_Pos );
  static constexpr uint32_t AFRL_AFSEL1_Pos = ( 4U );
  static constexpr uint32_t AFRL_AFSEL1_Msk = ( 0xFU << AFRL_AFSEL1_Pos );
  static constexpr uint32_t AFRL_AFSEL1     = AFRL_AFSEL1_Msk;
  static constexpr uint32_t AFRL_AFSEL1_0   = ( 0x1U << AFRL_AFSEL1_Pos );
  static constexpr uint32_t AFRL_AFSEL1_1   = ( 0x2U << AFRL_AFSEL1_Pos );
  static constexpr uint32_t AFRL_AFSEL1_2   = ( 0x4U << AFRL_AFSEL1_Pos );
  static constexpr uint32_t AFRL_AFSEL1_3   = ( 0x8U << AFRL_AFSEL1_Pos );
  static constexpr uint32_t AFRL_AFSEL2_Pos = ( 8U );
  static constexpr uint32_t AFRL_AFSEL2_Msk = ( 0xFU << AFRL_AFSEL2_Pos );
  static constexpr uint32_t AFRL_AFSEL2     = AFRL_AFSEL2_Msk;
  static constexpr uint32_t AFRL_AFSEL2_0   = ( 0x1U << AFRL_AFSEL2_Pos );
  static constexpr uint32_t AFRL_AFSEL2_1   = ( 0x2U << AFRL_AFSEL2_Pos );
  static constexpr uint32_t AFRL_AFSEL2_2   = ( 0x4U << AFRL_AFSEL2_Pos );
  static constexpr uint32_t AFRL_AFSEL2_3   = ( 0x8U << AFRL_AFSEL2_Pos );
  static constexpr uint32_t AFRL_AFSEL3_Pos = ( 12U );
  static constexpr uint32_t AFRL_AFSEL3_Msk = ( 0xFU << AFRL_AFSEL3_Pos );
  static constexpr uint32_t AFRL_AFSEL3     = AFRL_AFSEL3_Msk;
  static constexpr uint32_t AFRL_AFSEL3_0   = ( 0x1U << AFRL_AFSEL3_Pos );
  static constexpr uint32_t AFRL_AFSEL3_1   = ( 0x2U << AFRL_AFSEL3_Pos );
  static constexpr uint32_t AFRL_AFSEL3_2   = ( 0x4U << AFRL_AFSEL3_Pos );
  static constexpr uint32_t AFRL_AFSEL3_3   = ( 0x8U << AFRL_AFSEL3_Pos );
  static constexpr uint32_t AFRL_AFSEL4_Pos = ( 16U );
  static constexpr uint32_t AFRL_AFSEL4_Msk = ( 0xFU << AFRL_AFSEL4_Pos );
  static constexpr uint32_t AFRL_AFSEL4     = AFRL_AFSEL4_Msk;
  static constexpr uint32_t AFRL_AFSEL4_0   = ( 0x1U << AFRL_AFSEL4_Pos );
  static constexpr uint32_t AFRL_AFSEL4_1   = ( 0x2U << AFRL_AFSEL4_Pos );
  static constexpr uint32_t AFRL_AFSEL4_2   = ( 0x4U << AFRL_AFSEL4_Pos );
  static constexpr uint32_t AFRL_AFSEL4_3   = ( 0x8U << AFRL_AFSEL4_Pos );
  static constexpr uint32_t AFRL_AFSEL5_Pos = ( 20U );
  static constexpr uint32_t AFRL_AFSEL5_Msk = ( 0xFU << AFRL_AFSEL5_Pos );
  static constexpr uint32_t AFRL_AFSEL5     = AFRL_AFSEL5_Msk;
  static constexpr uint32_t AFRL_AFSEL5_0   = ( 0x1U << AFRL_AFSEL5_Pos );
  static constexpr uint32_t AFRL_AFSEL5_1   = ( 0x2U << AFRL_AFSEL5_Pos );
  static constexpr uint32_t AFRL_AFSEL5_2   = ( 0x4U << AFRL_AFSEL5_Pos );
  static constexpr uint32_t AFRL_AFSEL5_3   = ( 0x8U << AFRL_AFSEL5_Pos );
  static constexpr uint32_t AFRL_AFSEL6_Pos = ( 24U );
  static constexpr uint32_t AFRL_AFSEL6_Msk = ( 0xFU << AFRL_AFSEL6_Pos );
  static constexpr uint32_t AFRL_AFSEL6     = AFRL_AFSEL6_Msk;
  static constexpr uint32_t AFRL_AFSEL6_0   = ( 0x1U << AFRL_AFSEL6_Pos );
  static constexpr uint32_t AFRL_AFSEL6_1   = ( 0x2U << AFRL_AFSEL6_Pos );
  static constexpr uint32_t AFRL_AFSEL6_2   = ( 0x4U << AFRL_AFSEL6_Pos );
  static constexpr uint32_t AFRL_AFSEL6_3   = ( 0x8U << AFRL_AFSEL6_Pos );
  static constexpr uint32_t AFRL_AFSEL7_Pos = ( 28U );
  static constexpr uint32_t AFRL_AFSEL7_Msk = ( 0xFU << AFRL_AFSEL7_Pos );
  static constexpr uint32_t AFRL_AFSEL7     = AFRL_AFSEL7_Msk;
  static constexpr uint32_t AFRL_AFSEL7_0   = ( 0x1U << AFRL_AFSEL7_Pos );
  static constexpr uint32_t AFRL_AFSEL7_1   = ( 0x2U << AFRL_AFSEL7_Pos );
  static constexpr uint32_t AFRL_AFSEL7_2   = ( 0x4U << AFRL_AFSEL7_Pos );
  static constexpr uint32_t AFRL_AFSEL7_3   = ( 0x8U << AFRL_AFSEL7_Pos );

  /*------------------------------------------------
  AFRH
  ------------------------------------------------*/
  static constexpr uint32_t AFRH_AFSEL8_Pos  = ( 0U );
  static constexpr uint32_t AFRH_AFSEL8_Msk  = ( 0xFU << AFRH_AFSEL8_Pos );
  static constexpr uint32_t AFRH_AFSEL8      = AFRH_AFSEL8_Msk;
  static constexpr uint32_t AFRH_AFSEL8_0    = ( 0x1U << AFRH_AFSEL8_Pos );
  static constexpr uint32_t AFRH_AFSEL8_1    = ( 0x2U << AFRH_AFSEL8_Pos );
  static constexpr uint32_t AFRH_AFSEL8_2    = ( 0x4U << AFRH_AFSEL8_Pos );
  static constexpr uint32_t AFRH_AFSEL8_3    = ( 0x8U << AFRH_AFSEL8_Pos );
  static constexpr uint32_t AFRH_AFSEL9_Pos  = ( 4U );
  static constexpr uint32_t AFRH_AFSEL9_Msk  = ( 0xFU << AFRH_AFSEL9_Pos );
  static constexpr uint32_t AFRH_AFSEL9      = AFRH_AFSEL9_Msk;
  static constexpr uint32_t AFRH_AFSEL9_0    = ( 0x1U << AFRH_AFSEL9_Pos );
  static constexpr uint32_t AFRH_AFSEL9_1    = ( 0x2U << AFRH_AFSEL9_Pos );
  static constexpr uint32_t AFRH_AFSEL9_2    = ( 0x4U << AFRH_AFSEL9_Pos );
  static constexpr uint32_t AFRH_AFSEL9_3    = ( 0x8U << AFRH_AFSEL9_Pos );
  static constexpr uint32_t AFRH_AFSEL10_Pos = ( 8U );
  static constexpr uint32_t AFRH_AFSEL10_Msk = ( 0xFU << AFRH_AFSEL10_Pos );
  static constexpr uint32_t AFRH_AFSEL10     = AFRH_AFSEL10_Msk;
  static constexpr uint32_t AFRH_AFSEL10_0   = ( 0x1U << AFRH_AFSEL10_Pos );
  static constexpr uint32_t AFRH_AFSEL10_1   = ( 0x2U << AFRH_AFSEL10_Pos );
  static constexpr uint32_t AFRH_AFSEL10_2   = ( 0x4U << AFRH_AFSEL10_Pos );
  static constexpr uint32_t AFRH_AFSEL10_3   = ( 0x8U << AFRH_AFSEL10_Pos );
  static constexpr uint32_t AFRH_AFSEL11_Pos = ( 12U );
  static constexpr uint32_t AFRH_AFSEL11_Msk = ( 0xFU << AFRH_AFSEL11_Pos );
  static constexpr uint32_t AFRH_AFSEL11     = AFRH_AFSEL11_Msk;
  static constexpr uint32_t AFRH_AFSEL11_0   = ( 0x1U << AFRH_AFSEL11_Pos );
  static constexpr uint32_t AFRH_AFSEL11_1   = ( 0x2U << AFRH_AFSEL11_Pos );
  static constexpr uint32_t AFRH_AFSEL11_2   = ( 0x4U << AFRH_AFSEL11_Pos );
  static constexpr uint32_t AFRH_AFSEL11_3   = ( 0x8U << AFRH_AFSEL11_Pos );
  static constexpr uint32_t AFRH_AFSEL12_Pos = ( 16U );
  static constexpr uint32_t AFRH_AFSEL12_Msk = ( 0xFU << AFRH_AFSEL12_Pos );
  static constexpr uint32_t AFRH_AFSEL12     = AFRH_AFSEL12_Msk;
  static constexpr uint32_t AFRH_AFSEL12_0   = ( 0x1U << AFRH_AFSEL12_Pos );
  static constexpr uint32_t AFRH_AFSEL12_1   = ( 0x2U << AFRH_AFSEL12_Pos );
  static constexpr uint32_t AFRH_AFSEL12_2   = ( 0x4U << AFRH_AFSEL12_Pos );
  static constexpr uint32_t AFRH_AFSEL12_3   = ( 0x8U << AFRH_AFSEL12_Pos );
  static constexpr uint32_t AFRH_AFSEL13_Pos = ( 20U );
  static constexpr uint32_t AFRH_AFSEL13_Msk = ( 0xFU << AFRH_AFSEL13_Pos );
  static constexpr uint32_t AFRH_AFSEL13     = AFRH_AFSEL13_Msk;
  static constexpr uint32_t AFRH_AFSEL13_0   = ( 0x1U << AFRH_AFSEL13_Pos );
  static constexpr uint32_t AFRH_AFSEL13_1   = ( 0x2U << AFRH_AFSEL13_Pos );
  static constexpr uint32_t AFRH_AFSEL13_2   = ( 0x4U << AFRH_AFSEL13_Pos );
  static constexpr uint32_t AFRH_AFSEL13_3   = ( 0x8U << AFRH_AFSEL13_Pos );
  static constexpr uint32_t AFRH_AFSEL14_Pos = ( 24U );
  static constexpr uint32_t AFRH_AFSEL14_Msk = ( 0xFU << AFRH_AFSEL14_Pos );
  static constexpr uint32_t AFRH_AFSEL14     = AFRH_AFSEL14_Msk;
  static constexpr uint32_t AFRH_AFSEL14_0   = ( 0x1U << AFRH_AFSEL14_Pos );
  static constexpr uint32_t AFRH_AFSEL14_1   = ( 0x2U << AFRH_AFSEL14_Pos );
  static constexpr uint32_t AFRH_AFSEL14_2   = ( 0x4U << AFRH_AFSEL14_Pos );
  static constexpr uint32_t AFRH_AFSEL14_3   = ( 0x8U << AFRH_AFSEL14_Pos );
  static constexpr uint32_t AFRH_AFSEL15_Pos = ( 28U );
  static constexpr uint32_t AFRH_AFSEL15_Msk = ( 0xFU << AFRH_AFSEL15_Pos );
  static constexpr uint32_t AFRH_AFSEL15     = AFRH_AFSEL15_Msk;
  static constexpr uint32_t AFRH_AFSEL15_0   = ( 0x1U << AFRH_AFSEL15_Pos );
  static constexpr uint32_t AFRH_AFSEL15_1   = ( 0x2U << AFRH_AFSEL15_Pos );
  static constexpr uint32_t AFRH_AFSEL15_2   = ( 0x4U << AFRH_AFSEL15_Pos );
  static constexpr uint32_t AFRH_AFSEL15_3   = ( 0x8U << AFRH_AFSEL15_Pos );

  /*------------------------------------------------
  BRR
  ------------------------------------------------*/
  static constexpr uint32_t BRR_BR0_Pos  = ( 0U );
  static constexpr uint32_t BRR_BR0_Msk  = ( 0x1U << BRR_BR0_Pos );
  static constexpr uint32_t BRR_BR0      = BRR_BR0_Msk;
  static constexpr uint32_t BRR_BR1_Pos  = ( 1U );
  static constexpr uint32_t BRR_BR1_Msk  = ( 0x1U << BRR_BR1_Pos );
  static constexpr uint32_t BRR_BR1      = BRR_BR1_Msk;
  static constexpr uint32_t BRR_BR2_Pos  = ( 2U );
  static constexpr uint32_t BRR_BR2_Msk  = ( 0x1U << BRR_BR2_Pos );
  static constexpr uint32_t BRR_BR2      = BRR_BR2_Msk;
  static constexpr uint32_t BRR_BR3_Pos  = ( 3U );
  static constexpr uint32_t BRR_BR3_Msk  = ( 0x1U << BRR_BR3_Pos );
  static constexpr uint32_t BRR_BR3      = BRR_BR3_Msk;
  static constexpr uint32_t BRR_BR4_Pos  = ( 4U );
  static constexpr uint32_t BRR_BR4_Msk  = ( 0x1U << BRR_BR4_Pos );
  static constexpr uint32_t BRR_BR4      = BRR_BR4_Msk;
  static constexpr uint32_t BRR_BR5_Pos  = ( 5U );
  static constexpr uint32_t BRR_BR5_Msk  = ( 0x1U << BRR_BR5_Pos );
  static constexpr uint32_t BRR_BR5      = BRR_BR5_Msk;
  static constexpr uint32_t BRR_BR6_Pos  = ( 6U );
  static constexpr uint32_t BRR_BR6_Msk  = ( 0x1U << BRR_BR6_Pos );
  static constexpr uint32_t BRR_BR6      = BRR_BR6_Msk;
  static constexpr uint32_t BRR_BR7_Pos  = ( 7U );
  static constexpr uint32_t BRR_BR7_Msk  = ( 0x1U << BRR_BR7_Pos );
  static constexpr uint32_t BRR_BR7      = BRR_BR7_Msk;
  static constexpr uint32_t BRR_BR8_Pos  = ( 8U );
  static constexpr uint32_t BRR_BR8_Msk  = ( 0x1U << BRR_BR8_Pos );
  static constexpr uint32_t BRR_BR8      = BRR_BR8_Msk;
  static constexpr uint32_t BRR_BR9_Pos  = ( 9U );
  static constexpr uint32_t BRR_BR9_Msk  = ( 0x1U << BRR_BR9_Pos );
  static constexpr uint32_t BRR_BR9      = BRR_BR9_Msk;
  static constexpr uint32_t BRR_BR10_Pos = ( 10U );
  static constexpr uint32_t BRR_BR10_Msk = ( 0x1U << BRR_BR10_Pos );
  static constexpr uint32_t BRR_BR10     = BRR_BR10_Msk;
  static constexpr uint32_t BRR_BR11_Pos = ( 11U );
  static constexpr uint32_t BRR_BR11_Msk = ( 0x1U << BRR_BR11_Pos );
  static constexpr uint32_t BRR_BR11     = BRR_BR11_Msk;
  static constexpr uint32_t BRR_BR12_Pos = ( 12U );
  static constexpr uint32_t BRR_BR12_Msk = ( 0x1U << BRR_BR12_Pos );
  static constexpr uint32_t BRR_BR12     = BRR_BR12_Msk;
  static constexpr uint32_t BRR_BR13_Pos = ( 13U );
  static constexpr uint32_t BRR_BR13_Msk = ( 0x1U << BRR_BR13_Pos );
  static constexpr uint32_t BRR_BR13     = BRR_BR13_Msk;
  static constexpr uint32_t BRR_BR14_Pos = ( 14U );
  static constexpr uint32_t BRR_BR14_Msk = ( 0x1U << BRR_BR14_Pos );
  static constexpr uint32_t BRR_BR14     = BRR_BR14_Msk;
  static constexpr uint32_t BRR_BR15_Pos = ( 15U );
  static constexpr uint32_t BRR_BR15_Msk = ( 0x1U << BRR_BR15_Pos );
  static constexpr uint32_t BRR_BR15     = BRR_BR15_Msk;
}    // namespace Thor::Driver::GPIO

#endif /* TARGET_STM32F4 && THOR_DRIVER_GPIO */
#endif /* !THOR_HW_GPIO_REGISTER_HPP */