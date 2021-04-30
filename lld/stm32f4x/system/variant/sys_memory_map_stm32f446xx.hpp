/********************************************************************************
 *  File Name:
 *    sys_memory_map_stm32f446xx.hpp
 *
 *  Description:
 *    System level memory map definitions for the STM32F446xx series chips
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_MEMORY_MAP_HPP
#define THOR_SYSTEM_MEMORY_MAP_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f4xxxx.hpp>

namespace Thor::System::MemoryMap
{
  /*-------------------------------------------------------------------------------
  AHB3
  -------------------------------------------------------------------------------*/
  static constexpr uint32_t FMC_PERIPH_START_ADDRESS     = AHB3_RGN_START_ADDR;
  static constexpr uint32_t QUADSPI_PERIPH_START_ADDRESS = AHB3_RGN_START_ADDR + 0x1000;

  /*-------------------------------------------------------------------------------
  AHB2
  -------------------------------------------------------------------------------*/
  static constexpr uint32_t DCMI_PERIPH_START_ADDRESS = AHB2_RGN_START_ADDR + 0x00050000;
  static constexpr uint32_t USB_OTG_FS_START_ADDRESS  = AHB2_RGN_START_ADDR;

  /*-------------------------------------------------------------------------------
  AHB1
  -------------------------------------------------------------------------------*/
  static constexpr uint32_t USB_OTG_HS_START_ADDRESS   = 0x40040000;
  static constexpr uint32_t DMA2_PERIPH_START_ADDRESS  = AHB1_RGN_START_ADDR + 0x6400;
  static constexpr uint32_t DMA1_PERIPH_START_ADDRESS  = AHB1_RGN_START_ADDR + 0x6000;
  static constexpr uint32_t FLASH_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x3C00;
  static constexpr uint32_t RCC_PERIPH_START_ADDRESS   = AHB1_RGN_START_ADDR + 0x3800;
  static constexpr uint32_t CRC_PERIPH_START_ADDRESS   = AHB1_RGN_START_ADDR + 0x3000;
  static constexpr uint32_t GPIOH_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x1C00;
  static constexpr uint32_t GPIOG_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x1800;
  static constexpr uint32_t GPIOF_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x1400;
  static constexpr uint32_t GPIOE_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x1000;
  static constexpr uint32_t GPIOD_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x0C00;
  static constexpr uint32_t GPIOC_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x0800;
  static constexpr uint32_t GPIOB_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x0400;
  static constexpr uint32_t GPIOA_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDR + 0x0000;

  /*-------------------------------------------------------------------------------
  APB2
  -------------------------------------------------------------------------------*/
  static constexpr uint32_t SAI2_PERIPH_START_ADDRESS   = APB2_RGN_START_ADDR + 0x5C00;
  static constexpr uint32_t SAI1_PERIPH_START_ADDRESS   = APB2_RGN_START_ADDR + 0x5800;
  static constexpr uint32_t TIM11_PERIPH_START_ADDRESS  = APB2_RGN_START_ADDR + 0x4800;
  static constexpr uint32_t TIM10_PERIPH_START_ADDRESS  = APB2_RGN_START_ADDR + 0x4400;
  static constexpr uint32_t TIM9_PERIPH_START_ADDRESS   = APB2_RGN_START_ADDR + 0x4000;
  static constexpr uint32_t EXTI_PERIPH_START_ADDRESS   = APB2_RGN_START_ADDR + 0x3C00;
  static constexpr uint32_t SYSCFG_PERIPH_START_ADDRESS = APB2_RGN_START_ADDR + 0x3800;
  static constexpr uint32_t SPI4_PERIPH_START_ADDRESS   = APB2_RGN_START_ADDR + 0x3400;
  static constexpr uint32_t SPI1_PERIPH_START_ADDRESS   = APB2_RGN_START_ADDR + 0x3000;
  static constexpr uint32_t SDMMC_PERIPH_START_ADDRESS  = APB2_RGN_START_ADDR + 0x2C00;
  static constexpr uint32_t ADC123_COMMON_BASE          = APB2_RGN_START_ADDR + 0x2300;
  static constexpr uint32_t ADC3_BASE                   = APB2_RGN_START_ADDR + 0x2200;
  static constexpr uint32_t ADC2_BASE                   = APB2_RGN_START_ADDR + 0x2100;
  static constexpr uint32_t ADC1_BASE                   = APB2_RGN_START_ADDR + 0x2000;
  static constexpr uint32_t USART6_PERIPH_START_ADDRESS = APB2_RGN_START_ADDR + 0x1400;
  static constexpr uint32_t USART1_PERIPH_START_ADDRESS = APB2_RGN_START_ADDR + 0x1000;
  static constexpr uint32_t TIM8_PERIPH_START_ADDRESS   = APB2_RGN_START_ADDR + 0x0400;
  static constexpr uint32_t TIM1_PERIPH_START_ADDRESS   = APB2_RGN_START_ADDR + 0x0000;

  /*-------------------------------------------------------------------------------
  APB1
  -------------------------------------------------------------------------------*/
  static constexpr uint32_t DAC_PERIPH_START_ADDRESS       = APB1_RGN_START_ADDR + 0x7400;
  static constexpr uint32_t PWR_PERIPH_START_ADDRESS       = APB1_RGN_START_ADDR + 0x7000;
  static constexpr uint32_t HDMI_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x6C00;
  static constexpr uint32_t CAN2_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x6800;
  static constexpr uint32_t CAN1_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x6400;
  static constexpr uint32_t I2C3_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x5C00;
  static constexpr uint32_t I2C2_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x5800;
  static constexpr uint32_t I2C1_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x5400;
  static constexpr uint32_t UART5_PERIPH_START_ADDRESS     = APB1_RGN_START_ADDR + 0x5000;
  static constexpr uint32_t UART4_PERIPH_START_ADDRESS     = APB1_RGN_START_ADDR + 0x4C00;
  static constexpr uint32_t USART3_PERIPH_START_ADDRESS    = APB1_RGN_START_ADDR + 0x4800;
  static constexpr uint32_t USART2_PERIPH_START_ADDRESS    = APB1_RGN_START_ADDR + 0x4400;
  static constexpr uint32_t SPDIF_PERIPH_START_ADDRESS     = APB1_RGN_START_ADDR + 0x4000;
  static constexpr uint32_t SPI3_I2S3_PERIPH_START_ADDRESS = APB1_RGN_START_ADDR + 0x3C00;
  static constexpr uint32_t SPI2_I2S2_PERIPH_START_ADDRESS = APB1_RGN_START_ADDR + 0x3800;
  static constexpr uint32_t IWDG_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x3000;
  static constexpr uint32_t WWDG_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x2C00;
  static constexpr uint32_t RTC_BKP_PERIPH_START_ADDRESS   = APB1_RGN_START_ADDR + 0x2800;
  static constexpr uint32_t TIM14_PERIPH_START_ADDRESS     = APB1_RGN_START_ADDR + 0x2000;
  static constexpr uint32_t TIM13_PERIPH_START_ADDRESS     = APB1_RGN_START_ADDR + 0x1C00;
  static constexpr uint32_t TIM12_PERIPH_START_ADDRESS     = APB1_RGN_START_ADDR + 0x1800;
  static constexpr uint32_t TIM7_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x1400;
  static constexpr uint32_t TIM6_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x1000;
  static constexpr uint32_t TIM5_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x0C00;
  static constexpr uint32_t TIM4_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x0800;
  static constexpr uint32_t TIM3_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x0400;
  static constexpr uint32_t TIM2_PERIPH_START_ADDRESS      = APB1_RGN_START_ADDR + 0x0000;

}    // namespace Thor::System::MemoryMap

#endif /* !THOR_SYSTEM_MEMORY_MAP_HPP */
