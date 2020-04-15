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
  static constexpr uint32_t PWR_PERIPH_START_ADDRESS = APB1_RGN_START_ADDRESS + 0x7000;
  static constexpr uint32_t PWR_PERIPH_END_ADDRESS   = APB1_RGN_START_ADDRESS + 0x73FF;

  /*-------------------------------------------------
  APB2
  -------------------------------------------------*/

  /*-------------------------------------------------
  AHB1
  -------------------------------------------------*/
  static constexpr uint32_t RCC_PERIPH_START_ADDRESS = AHB1_RGN_START_ADDRESS + 0x00001000;
  static constexpr uint32_t RCC_PERIPH_END_ADDRESS   = AHB1_RGN_START_ADDRESS + 0x000013FF;

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

#endif  /* !THOR_SYSTEM_MEMORY_MAP_HPP */
