/********************************************************************************
 *  File Name:
 *    hw_spi_register_stm32l432kc.hpp
 *
 *  Description:
 *    SPI definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_REGISTER_STM32L432KC_HPP
#define THOR_LLD_SPI_REGISTER_STM32L432KC_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_SPI1_PERIPH_AVAILABLE
#define STM32_SPI3_PERIPH_AVAILABLE


namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_SPI_PERIPHS = 2;

  static constexpr RIndex_t SPI1_RESOURCE_INDEX = 0u;
  static constexpr RIndex_t SPI2_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;
  static constexpr RIndex_t SPI3_RESOURCE_INDEX = 1u;
  static constexpr RIndex_t SPI4_RESOURCE_INDEX = INVALID_RESOURCE_INDEX;

}    // namespace Thor::LLD::SPI

#endif /* !THOR_LLD_SPI_REGISTER_STM32L432KC_HPP */
