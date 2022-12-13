/******************************************************************************
 *  File Name:
 *    sys_memory_map_stm32f4xxxx.hpp
 *
 *  Description:
 *    System level memory map definitions for the STM32F4xxxx series chips
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_MEMORY_MAP_STM32F4XXXX_HPP
#define THOR_SYSTEM_MEMORY_MAP_STM32F4XXXX_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::System::MemoryMap
{
  /*---------------------------------------------------------------------------
  Flash
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t FLASH_RGN_START_ADDR     = 0x08000000u; /**< FLASH(up to 1 MB) BASE address in the alias region  */
  static constexpr uint32_t FLASH_MAX_END_ADDR       = 0x0807FFFFu; /**< FLASH end address */
  static constexpr uint32_t FLASH_OTP_RGN_START_ADDR = 0x1FFF7800u; /**< RGN_START_ADDR address of embedded FLASH OTP Area  */
  static constexpr uint32_t FLASH_OTP_END_ADDR       = 0x1FFF7A0Fu; /**< End address of embedded FLASH OTP Area  */

  /*---------------------------------------------------------------------------
  SRAM
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t SRAM1_RGN_START_ADDR   = 0x20000000u; /**< SRAM1(112 KB) BASE address in the alias region  */
  static constexpr uint32_t SRAM2_RGN_START_ADDR   = 0x2001C000u; /**< SRAM2(16 KB) BASE address in the alias region  */
  static constexpr uint32_t BKPSRAM_RGN_START_ADDR = 0x40024000u; /**< Backup SRAM(4 KB) BASE address in the alias region  */

  /*---------------------------------------------------------------------------
  Memory Bus
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t APB1_RGN_START_ADDR = 0x40000000u; /**< Peripheral BASE address in the alias region  */
  static constexpr uint32_t APB2_RGN_START_ADDR = 0x40010000u;

  static constexpr uint32_t AHB1_RGN_START_ADDR = 0x40020000u;
  static constexpr uint32_t AHB2_RGN_START_ADDR = 0x50000000u;
  static constexpr uint32_t AHB3_RGN_START_ADDR = 0xA0000000u;

  static constexpr uint32_t VECT_TAB_OFFSET =
      0x00; /*!< Vector Table base offset field. This value must be a multiple of 0x200. */

}    // namespace Thor::System::MemoryMap

#endif /* !THOR_SYSTEM_MEMORY_MAP_STM32F4XXXX_HPP */
