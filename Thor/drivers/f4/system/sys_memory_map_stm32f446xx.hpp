/********************************************************************************
 *   File Name:
 *    sys_memory_map_stm32f446xx.hpp
 *
 *   Description:
 *    System level memory map definitions for the STM32F446xx series chips
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_MEMORY_MAP_HPP
#define THOR_SYSTEM_MEMORY_MAP_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::System::MemoryMap
{
  static constexpr uint32_t FLASH_BASE      = 0x08000000U;  /**< FLASH(up to 1 MB) base address in the alias region  */
  static constexpr uint32_t SRAM1_BASE      = 0x20000000U;  /**< SRAM1(112 KB) base address in the alias region  */
  static constexpr uint32_t SRAM2_BASE      = 0x2001C000U;  /**< SRAM2(16 KB) base address in the alias region  */
  static constexpr uint32_t PERIPH_BASE     = 0x40000000U;  /**< Peripheral base address in the alias region  */
  static constexpr uint32_t BKPSRAM_BASE    = 0x40024000U;  /**< Backup SRAM(4 KB) base address in the alias region  */
  static constexpr uint32_t FMC_R_BASE      = 0xA0000000U;  /**< FMC registers base address  */
  static constexpr uint32_t QSPI_R_BASE     = 0xA0001000U;  /**< QuadSPI registers base address */
  static constexpr uint32_t SRAM1_BB_BASE   = 0x22000000U;  /**< SRAM1(112 KB) base address in the bit-band region */
  static constexpr uint32_t SRAM2_BB_BASE   = 0x22380000U;  /**< SRAM2(16 KB) base address in the bit-band region */
  static constexpr uint32_t PERIPH_BB_BASE  = 0x42000000U;  /**< Peripheral base address in the bit-band region */
  static constexpr uint32_t BKPSRAM_BB_BASE = 0x42480000U;  /**< Backup SRAM(4 KB) base address in the bit-band region */
  static constexpr uint32_t FLASH_END       = 0x0807FFFFU;  /**< FLASH end address */
  static constexpr uint32_t FLASH_OTP_BASE  = 0x1FFF7800U;  /**< Base address of : (up to 528 Bytes) embedded FLASH OTP Area  */
  static constexpr uint32_t FLASH_OTP_END   = 0x1FFF7A0FU;  /**< End address of : (up to 528 Bytes) embedded FLASH OTP Area  */


  static constexpr uint32_t APB1PERIPH_BASE = PERIPH_BASE;
  static constexpr uint32_t APB2PERIPH_BASE = ( PERIPH_BASE + 0x00010000U );
  static constexpr uint32_t AHB1PERIPH_BASE = ( PERIPH_BASE + 0x00020000U );
  static constexpr uint32_t AHB2PERIPH_BASE = ( PERIPH_BASE + 0x10000000U );
}

#endif /* !THOR_SYSTEM_MEMORY_MAP_HPP */