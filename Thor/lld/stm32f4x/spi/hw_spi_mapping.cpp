/********************************************************************************
 *  File Name:
 *    hw_spi_mapping.cpp
 *
 *  Description:
 *    Useful maps for the SPI peripherals
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/spi/hw_spi_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_LLD_SPI )

namespace Thor::LLD::SPI
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;
  DriverInstanceList spiObjects;

  /*------------------------------------------------
  Chimera Configuration
  ------------------------------------------------*/
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


  void initializeMapping()
  {
    spiObjects.fill( nullptr );
  }

  bool isSPI( std::uintptr_t address )
  {
    bool result = false;

    for ( auto &val : periphAddressList )
    {
      if ( val == address )
      {
        result = true;
        break;
      }
    }

    return result;
  }

}    // namespace Thor::LLD::SPI

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
