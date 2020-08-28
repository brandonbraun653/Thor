/********************************************************************************
 *  File Name:
 *    spi_mock_variant.hpp
 *
 *  Description:
 *    Mock variant of the SPI hardware
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_SPI_MOCK_VARIANT_HPP
#define THOR_LLD_SPI_MOCK_VARIANT_HPP

/* Chimera Includes */
#include <Chimera/container>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/spi/spi_intf.hpp>
#include <Thor/lld/interface/spi/spi_types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_SPI1_PERIPH_AVAILABLE
#define STM32_SPI2_PERIPH_AVAILABLE
#define STM32_SPI3_PERIPH_AVAILABLE
#define STM32_SPI4_PERIPH_AVAILABLE

namespace Thor::LLD::SPI
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_SPI_PERIPHS = 4;

  static constexpr uint32_t SPI1_RESOURCE_INDEX = 0u;
  static constexpr uint32_t SPI2_RESOURCE_INDEX = 1u;
  static constexpr uint32_t SPI3_RESOURCE_INDEX = 2u;
  static constexpr uint32_t SPI4_RESOURCE_INDEX = 2u;

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    uint32_t placeHolderReg;
  };

  /*-------------------------------------------------------------------------------
  Variables
  -------------------------------------------------------------------------------*/
  extern std::array<RegisterMap*, NUM_SPI_PERIPHS> PeripheralRegisterMaps;
  extern Thor::LLD::RIndexMap InstanceToResourceIndex;
  extern Chimera::Container::LightFlatMap<Chimera::SPI::Channel, RegisterMap*> ChannelToInstance;


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeMapping();
  void initializeRegisters();

}  // namespace Thor::LLD::SPI

#endif  /* !THOR_LLD_SPI_MOCK_VARIANT_HPP */
