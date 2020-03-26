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
}    // namespace Thor::System::MemoryMap

#endif  /* !THOR_SYSTEM_MEMORY_MAP_HPP */
