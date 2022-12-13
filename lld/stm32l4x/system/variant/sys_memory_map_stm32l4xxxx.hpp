/******************************************************************************
 *  File Name:
 *    sys_memory_map_stm32l432xx.hpp
 *
 *  Description:
 *    System level memory map definitions for the STM32L4xxxx series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_MEMORY_MAP_STM32L4XXXX_HPP
#define THOR_SYSTEM_MEMORY_MAP_STM32L4XXXX_HPP

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

}    // namespace Thor::System::MemoryMap

#endif /* !THOR_SYSTEM_MEMORY_MAP_STM32L4XXXX_HPP */
