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
  static constexpr uint32_t FLASH_BASE_ADDR      = 0x08000000U; /**< FLASH(up to 1 MB) BASE address in the alias region  */
  static constexpr uint32_t SRAM1_BASE_ADDR      = 0x20000000U; /**< SRAM1(112 KB) BASE address in the alias region  */
  static constexpr uint32_t SRAM2_BASE_ADDR      = 0x2001C000U; /**< SRAM2(16 KB) BASE address in the alias region  */
  static constexpr uint32_t PERIPH_BASE_ADDR     = 0x40000000U; /**< Peripheral BASE address in the alias region  */
  static constexpr uint32_t BKPSRAM_BASE_ADDR    = 0x40024000U; /**< Backup SRAM(4 KB) BASE address in the alias region  */
  static constexpr uint32_t FMC_R_BASE_ADDR      = 0xA0000000U; /**< FMC registers BASE address  */
  static constexpr uint32_t QSPI_R_BASE_ADDR     = 0xA0001000U; /**< QuadSPI registers BASE address */
  static constexpr uint32_t SRAM1_BB_BASE_ADDR   = 0x22000000U; /**< SRAM1(112 KB) BASE address in the bit-band region */
  static constexpr uint32_t SRAM2_BB_BASE_ADDR   = 0x22380000U; /**< SRAM2(16 KB) BASE address in the bit-band region */
  static constexpr uint32_t PERIPH_BB_BASE_ADDR  = 0x42000000U; /**< Peripheral BASE address in the bit-band region */
  static constexpr uint32_t BKPSRAM_BB_BASE_ADDR = 0x42480000U; /**< Backup SRAM(4 KB) BASE in the bit-band region */
  static constexpr uint32_t FLASH_END_ADDR       = 0x0807FFFFU; /**< FLASH end address */
  static constexpr uint32_t FLASH_OTP_BASE_ADDR  = 0x1FFF7800U; /**< BASE_ADDR address of embedded FLASH OTP Area  */
  static constexpr uint32_t FLASH_OTP_END_ADDR   = 0x1FFF7A0FU; /**< End address of embedded FLASH OTP Area  */

  static constexpr uint32_t APB1PERIPH_BASE_ADDR = PERIPH_BASE_ADDR;
  static constexpr uint32_t APB2PERIPH_BASE_ADDR = ( PERIPH_BASE_ADDR + 0x00010000U );
  static constexpr uint32_t AHB1PERIPH_BASE_ADDR = ( PERIPH_BASE_ADDR + 0x00020000U );
  static constexpr uint32_t AHB2PERIPH_BASE_ADDR = ( PERIPH_BASE_ADDR + 0x10000000U );

  static constexpr uint32_t VECT_TAB_OFFSET = 0x00; /*!< Vector Table base offset field. This value must be a multiple of 0x200. */

}    // namespace Thor::System::MemoryMap

#endif /* !THOR_SYSTEM_MEMORY_MAP_HPP */
