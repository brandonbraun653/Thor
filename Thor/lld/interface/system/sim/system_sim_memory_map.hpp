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
#ifndef THOR_LLD_SIM_SYSTEM_MEMORY_MAP_HPP
#define THOR_LLD_SIM_SYSTEM_MEMORY_MAP_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::System::MemoryMap
{
  /*-------------------------------------------------
  Boot Configuration Memory Region
  -------------------------------------------------*/
  static constexpr uint32_t BOOT_RGN_START_ADDRESS = 0x00000000;
  static constexpr uint32_t BOOT_RGN_END_ADDRESS   = 0x00080000;

  /*------------------------------------------------
  Main Flash Memory (128kB-512kB): Actual end address defined in chip's header
  ------------------------------------------------*/
  static constexpr uint32_t MAIN_FLASH_RGN_START_ADDRESS    = 0x08000000;
  static constexpr uint32_t MAIN_FLASH_RGN_END_ADDRESS_128K = 0x08020000;
  static constexpr uint32_t MAIN_FLASH_RGN_END_ADDRESS_256K = 0x08040000;
  static constexpr uint32_t MAIN_FLASH_RGN_END_ADDRESS_512K = 0x08080000;

  /*-------------------------------------------------
  System Flash Memory (28kB)
  -------------------------------------------------*/
  static constexpr uint32_t SYS_FLASH_RGN_START_ADDRESS = 0x1FFF0000;
  static constexpr uint32_t SYS_FLASH_RGN_END_ADDRESS   = 0x1FFF7000;

  /*-------------------------------------------------
  One Time Programmable Flash (1kB)
  -------------------------------------------------*/
  static constexpr uint32_t OTP_FLASH_RGN_START_ADDRESS = 0x1FFF7000;
  static constexpr uint32_t OTP_FLASH_RGN_END_ADDRESS   = 0x1FFF7400;

  /*-------------------------------------------------
  Option Bytes Flash (16)
  -------------------------------------------------*/
  static constexpr uint32_t OPTIONS_FLASH_RGN_START_ADDRESS = 0x1FFF7800;
  static constexpr uint32_t OPTIONS_FLASH_RGN_END_ADDRESS   = 0x1FFF7810;

  /*-------------------------------------------------
  SRAM 1 (32kB-128kB): Actual end address defined in chip's header
  -------------------------------------------------*/
  static constexpr uint32_t SRAM1_RGN_START_ADDRESS     = 0x20000000;
  static constexpr uint32_t SRAM1_RGN_END_ADDRESS_32KB  = 0x20008000;
  static constexpr uint32_t SRAM1_RGN_END_ADDRESS_48KB  = 0x2000C000;
  static constexpr uint32_t SRAM1_RGN_END_ADDRESS_128KB = 0x20020000;

  /*-------------------------------------------------
  SRAM 2 (8kB-32kB): Actual end address defined in chip's header
  -------------------------------------------------*/
  static constexpr uint32_t SRAM2_RGN_START_ADDRESS    = 0x10000000;
  static constexpr uint32_t SRAM2_RGN_END_ADDRESS_8KB  = 0x10002000;
  static constexpr uint32_t SRAM2_RGN_END_ADDRESS_16KB = 0x10004000;
  static constexpr uint32_t SRAM2_RGN_END_ADDRESS_32KB = 0x10008000;

  /*-------------------------------------------------
  APB1
  -------------------------------------------------*/
  static constexpr uint32_t APB1_RGN_START_ADDRESS = 0x40000000;
  static constexpr uint32_t APB1_RGN_END_ADDRESS   = 0x40009800;

  /*-------------------------------------------------
  APB2
  -------------------------------------------------*/
  static constexpr uint32_t APB2_RGN_START_ADDRESS = 0x40010000;
  static constexpr uint32_t APB2_RGN_END_ADDRESS   = 0x40016400;

  /*-------------------------------------------------
  AHB1
  -------------------------------------------------*/
  static constexpr uint32_t AHB1_RGN_START_ADDRESS = 0x40020000;
  static constexpr uint32_t AHB1_RGN_END_ADDRESS   = 0x40024400;

  /*-------------------------------------------------
  AHB2
  -------------------------------------------------*/
  static constexpr uint32_t AHB2_RGN_START_ADDRESS = 0x48000000;
  static constexpr uint32_t AHB2_RGN_END_ADDRESS   = 0x50060C00;

  /*-------------------------------------------------
  Quad SPI Registers
  -------------------------------------------------*/
  static constexpr uint32_t QUADSPI_REG_RGN_START_ADDRESS = 0xA0001000;
  static constexpr uint32_t QUADSPI_REG_RGN_END_ADDRESS   = 0xA0001400;

  /*-------------------------------------------------
  Quad SPI Flash Bank
  -------------------------------------------------*/
  static constexpr uint32_t QUADSPI_FLASH_RGN_START_ADDRESS = 0x90000000;
  static constexpr uint32_t QUADSPI_FLASH_RGN_END_ADDRESS   = 0xA0000000;

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
  static constexpr uint32_t USART2_PERIPH_END_ADDRESS = APB1_RGN_START_ADDRESS + 0x47FF;

  static constexpr uint32_t USART3_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x4800;
  static constexpr uint32_t USART3_PERIPH_END_ADDRESS = APB1_RGN_START_ADDRESS + 0x4BFF;

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
  static constexpr uint32_t USART1_PERIPH_END_ADDRESS = APB2_RGN_START_ADDRESS + 0x3BFF;

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

#endif  /* !THOR_LLD_SIM_SYSTEM_MEMORY_MAP_HPP */
