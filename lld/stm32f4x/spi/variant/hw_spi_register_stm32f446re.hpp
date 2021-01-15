/********************************************************************************
 *  File Name:
 *    hw_spi_register_stm32f446re.hpp
 *
 *  Description:
 *    Device specific definitions
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_REGISTERS_STM32F446RE_HPP
#define THOR_LLD_SPI_REGISTERS_STM32F446RE_HPP

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/system/variant/sys_memory_map_stm32f446xx.hpp>

#define STM32_SPI1_PERIPH_AVAILABLE
#define STM32_SPI2_PERIPH_AVAILABLE
#define STM32_SPI3_PERIPH_AVAILABLE
#define STM32_SPI4_PERIPH_AVAILABLE

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr Reg32_t SPI1_BASE_ADDR = Thor::System::MemoryMap::SPI1_PERIPH_START_ADDRESS;
  static constexpr Reg32_t SPI2_BASE_ADDR = Thor::System::MemoryMap::SPI2_I2S2_PERIPH_START_ADDRESS;
  static constexpr Reg32_t SPI3_BASE_ADDR = Thor::System::MemoryMap::SPI3_I2S3_PERIPH_START_ADDRESS;
  static constexpr Reg32_t SPI4_BASE_ADDR = Thor::System::MemoryMap::SPI4_PERIPH_START_ADDRESS;

  static constexpr Reg32_t NUM_SPI_PERIPHS = 4;

  /*------------------------------------------------
  Resource Lookup Indexes
  ------------------------------------------------*/
  static constexpr uint32_t SPI1_RESOURCE_INDEX = 0u;
  static constexpr uint32_t SPI2_RESOURCE_INDEX = 1u;
  static constexpr uint32_t SPI3_RESOURCE_INDEX = 2u;
  static constexpr uint32_t SPI4_RESOURCE_INDEX = 3u;

  /*------------------------------------------------
  Supported Hardware Channels
  ------------------------------------------------*/
  static constexpr uint8_t SPI1_CHANNEL_NUMBER = 1u;
  static constexpr uint8_t SPI2_CHANNEL_NUMBER = 2u;
  static constexpr uint8_t SPI3_CHANNEL_NUMBER = 3u;
  static constexpr uint8_t SPI4_CHANNEL_NUMBER = 4u;

}  // namespace Thor::LLD::SPI

#endif  /* !THOR_LLD_SPI_REGISTERS_STM32F446RE_HPP */
