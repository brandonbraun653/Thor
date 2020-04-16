/********************************************************************************
 *  File Name:
 *    hw_spi_mapping.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/spi/hw_spi_mapping.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_SPI )

namespace Thor::LLD::SPI
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;
  DriverInstanceList spiObjects;

  /*-------------------------------------------------
  Module Functions 
  -------------------------------------------------*/
  void initializeMapping()
  {
#if defined( STM32_SPI1_PERIPH_AVAILABLE )
    PeripheralList[ SPI1_RESOURCE_INDEX ] = SPI1_PERIPH;
#endif

#if defined( STM32_SPI2_PERIPH_AVAILABLE )
    PeripheralList[ SPI2_RESOURCE_INDEX ] = SPI2_PERIPH;
#endif 

#if defined( STM32_SPI3_PERIPH_AVAILABLE )
    PeripheralList[ SPI3_RESOURCE_INDEX ] = SPI3_PERIPH;
#endif

    spiObjects.fill( nullptr );
  }

  /*-------------------------------------------------
  Initialize the Chimera Option to Register Config Mappings
  -------------------------------------------------*/
  /* clang-format off */
   std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::BitOrder::NUM_OPTIONS )> BitOrderToRegConfig = {
    Configuration::DataFormat::MSB, Configuration::DataFormat::LSB
  };

  std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::ClockMode::NUM_OPTIONS )> ClockModeToRegConfig = {
    Configuration::ClockFormat::MODE0, Configuration::ClockFormat::MODE1, Configuration::ClockFormat::MODE2,
    Configuration::ClockFormat::MODE3
  };

  std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::ControlMode::NUM_OPTIONS )> ControlModeToRegConfig = {
    Configuration::Mode::MASTER,
    Configuration::Mode::SLAVE
  };

  std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::DataSize::NUM_OPTIONS )> DataSizeToRegConfig = {
    Configuration::Width::WIDTH_8_BIT,
    Configuration::Width::WIDTH_16_BIT,
    Configuration::Width::WIDTH_16_BIT,
    Configuration::Width::WIDTH_16_BIT, 
    Configuration::Width::WIDTH_16_BIT,
    Configuration::Width::WIDTH_16_BIT,
    Configuration::Width::WIDTH_16_BIT, 
    Configuration::Width::WIDTH_16_BIT, 
    Configuration::Width::WIDTH_16_BIT
  };
  /* clang-format on */

}    // namespace Thor::LLD::SPI

#endif /* TARGET_STM32L4 && THOR_LLD_SPI */
