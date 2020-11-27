/********************************************************************************
 *  File Name:
 *    sys_memory_map_stm32l432xx.hpp
 *
 *  Description:
 *    System level memory map definitions for the STM32L432xx series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_MEMORY_MAP_HPP
#define THOR_SYSTEM_MEMORY_MAP_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/lld/stm32l4x/system/variant/sys_memory_map_stm32l4xxxx.hpp>

namespace Thor::System::MemoryMap
{
  /*------------------------------------------------
  Flash Memory (256kB)
  ------------------------------------------------*/
  static constexpr uint32_t MAIN_FLASH_RGN_END_ADDRESS = MAIN_FLASH_RGN_END_ADDRESS_256K;

  /*-------------------------------------------------
  SRAM 1 (48kB)
  -------------------------------------------------*/
  static constexpr uint32_t SRAM1_RGN_END_ADDRESS = SRAM1_RGN_END_ADDRESS_48KB;

  /*-------------------------------------------------
  SRAM 2 (16kB)
  -------------------------------------------------*/
  static constexpr uint32_t SRAM2_RGN_END_ADDRESS = SRAM2_RGN_END_ADDRESS_16KB;

  /*-------------------------------------------------
  APB1
  -------------------------------------------------*/
  static constexpr uint32_t CAN1_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x6400;
  static constexpr uint32_t CAN1_PERIPH_END_ADDRESS   = APB1_RGN_START_ADDRESS + 0x67FF;

  static constexpr uint32_t PWR_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x7000;
  static constexpr uint32_t PWR_PERIPH_END_ADDRESS   = APB1_RGN_START_ADDRESS + 0x73FF;

  static constexpr uint32_t SPI2_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x3800;
  static constexpr uint32_t SPI2_PERIPH_END_ADDRESS   = APB1_RGN_START_ADDRESS + 0x3BFF;

  static constexpr uint32_t SPI3_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x3C00;
  static constexpr uint32_t SPI3_PERIPH_END_ADDRESS   = APB1_RGN_START_ADDRESS + 0x3FFF;

  static constexpr uint32_t LPTIMER2_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x9400;
  static constexpr uint32_t LPTIMER2_PERIPH_END_ADDRESS   = APB1_RGN_END_ADDRESS + 0x97FF;

  static constexpr uint32_t LPTIMER1_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x7C00;
  static constexpr uint32_t LPTIMER1_PERIPH_END_ADDRESS   = APB1_RGN_END_ADDRESS + 0x7FFF;

  static constexpr uint32_t TIMER7_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x1400;
  static constexpr uint32_t TIMER7_PERIPH_END_ADDRESS   = APB1_RGN_END_ADDRESS + 0x17FF;

  static constexpr uint32_t TIMER6_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x1000;
  static constexpr uint32_t TIMER6_PERIPH_END_ADDRESS   = APB1_RGN_END_ADDRESS + 0x13FF;

  static constexpr uint32_t TIMER3_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x0400;
  static constexpr uint32_t TIMER3_PERIPH_END_ADDRESS   = APB1_RGN_END_ADDRESS + 0x07FF;

  static constexpr uint32_t TIMER2_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x0000;
  static constexpr uint32_t TIMER2_PERIPH_END_ADDRESS   = APB1_RGN_END_ADDRESS + 0x03FF;

  static constexpr uint32_t USART2_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x4400;
  static constexpr uint32_t USART2_PERIPH_END_ADDRESS   = APB1_RGN_START_ADDRESS + 0x47FF;

  static constexpr uint32_t USART3_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x4800;
  static constexpr uint32_t USART3_PERIPH_END_ADDRESS   = APB1_RGN_START_ADDRESS + 0x4BFF;

  static constexpr uint32_t USB1_FS_SRAM_START_ADDRESS   = APB1_RGN_START_ADDRESS + 0x6C00;
  static constexpr uint32_t USB1_FS_SRAM_END_ADDRESS     = APB1_RGN_START_ADDRESS + 0x6FFF;
  static constexpr uint32_t USB1_FS_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x6800;
  static constexpr uint32_t USB1_FS_PERIPH_END_ADDRESS   = APB1_RGN_START_ADDRESS + 0x6BFF;

  /*-------------------------------------------------
  APB2
  -------------------------------------------------*/
  static constexpr uint32_t SPI1_PERIPH_START_ADDRESS = APB2_RGN_START_ADDRESS + 0x3000;
  static constexpr uint32_t SPI1_PERIPH_END_ADDRESS   = APB2_RGN_START_ADDRESS + 0x33FF;

  static constexpr uint32_t TIMER16_PERIPH_START_ADDRESS = APB2_RGN_START_ADDRESS + 0x4400;
  static constexpr uint32_t TIMER16_PERIPH_END_ADDRESS   = APB2_RGN_END_ADDRESS + 0x47FF;

  static constexpr uint32_t TIMER15_PERIPH_START_ADDRESS = APB2_RGN_START_ADDRESS + 0x4000;
  static constexpr uint32_t TIMER15_PERIPH_END_ADDRESS   = APB2_RGN_END_ADDRESS + 0x43FF;

  static constexpr uint32_t TIMER1_PERIPH_START_ADDRESS = APB2_RGN_START_ADDRESS + 0x2C00;
  static constexpr uint32_t TIMER1_PERIPH_END_ADDRESS   = APB2_RGN_END_ADDRESS + 0x2FFF;

  static constexpr uint32_t USART1_PERIPH_START_ADDRESS = APB2_RGN_START_ADDRESS + 0x3800;
  static constexpr uint32_t USART1_PERIPH_END_ADDRESS   = APB2_RGN_START_ADDRESS + 0x3BFF;

  static constexpr uint32_t EXTI_PERIPH_START_ADDRESS = APB2_RGN_START_ADDRESS + 0x0400;
  static constexpr uint32_t EXTI_PERIPH_END_ADDRESS   = APB2_RGN_END_ADDRESS + 0x07FF;

  static constexpr uint32_t SYSCFG_PERIPH_START_ADDRESS = APB2_RGN_START_ADDRESS + 0x0000;
  static constexpr uint32_t SYSCFG_PERIPH_END_ADDRESS   = APB2_RGN_END_ADDRESS + 0x002F;

  /*-------------------------------------------------
  AHB1
  -------------------------------------------------*/
  static constexpr uint32_t RCC_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDRESS + 0x1000;
  static constexpr uint32_t RCC_PERIPH_END_ADDRESS   = AHB1_RGN_START_ADDRESS + 0x13FF;

  static constexpr uint32_t FLASH_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDRESS + 0x2000;
  static constexpr uint32_t FLASH_PERIPH_END_ADDRESS   = AHB1_RGN_START_ADDRESS + 0x23FF;

  /*-------------------------------------------------
  AHB2
  -------------------------------------------------*/
  static constexpr uint32_t GPIOA_PERIPH_START_ADDRESS = AHB2_RGN_START_ADDRESS + 0x0000U;
  static constexpr uint32_t GPIOA_PERIPH_END_ADDRESS   = AHB2_RGN_START_ADDRESS + 0x03FFU;

  static constexpr uint32_t GPIOB_PERIPH_START_ADDRESS = AHB2_RGN_START_ADDRESS + 0x0400U;
  static constexpr uint32_t GPIOB_PERIPH_END_ADDRESS   = AHB2_RGN_START_ADDRESS + 0x07FFU;

  static constexpr uint32_t GPIOC_PERIPH_START_ADDRESS = AHB2_RGN_START_ADDRESS + 0x0800U;
  static constexpr uint32_t GPIOC_PERIPH_END_ADDRESS   = AHB2_RGN_START_ADDRESS + 0x0BFFU;

  static constexpr uint32_t GPIOD_PERIPH_START_ADDRESS = AHB2_RGN_START_ADDRESS + 0x0C00U;
  static constexpr uint32_t GPIOD_PERIPH_END_ADDRESS   = AHB2_RGN_START_ADDRESS + 0x0FFFU;

  static constexpr uint32_t GPIOE_PERIPH_START_ADDRESS = AHB2_RGN_START_ADDRESS + 0x1000U;
  static constexpr uint32_t GPIOE_PERIPH_END_ADDRESS   = AHB2_RGN_START_ADDRESS + 0x13FFU;

  static constexpr uint32_t GPIOH_PERIPH_START_ADDRESS = AHB2_RGN_START_ADDRESS + 0x1C00U;
  static constexpr uint32_t GPIOH_PERIPH_END_ADDRESS   = AHB2_RGN_START_ADDRESS + 0x1FFFU;

  /*------------------------------------------------
  Vector Table Offset Configuration
  ------------------------------------------------*/
  static constexpr uint32_t FLASH_BASE_ADDR = MAIN_FLASH_RGN_START_ADDRESS;
  static constexpr uint32_t VECT_TAB_OFFSET = 0x00;


}    // namespace Thor::System::MemoryMap

#endif /* !THOR_SYSTEM_MEMORY_MAP_HPP */
