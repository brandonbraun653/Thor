/********************************************************************************
 *   File Name:
 *    hw_spi_mapping.cpp
 *
 *   Description:
 *    Useful maps for the SPI peripherals
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/spi/hw_spi_mapping.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  /*------------------------------------------------
  Chip Specific Resources
  ------------------------------------------------*/
  PeriphRegisterList PeripheralList;
  DMASignalList RXDMASignals;
  DMASignalList TXDMASignals;
  DriverInstanceList spiObjects;

  /*------------------------------------------------
  Chimera Configuration
  ------------------------------------------------*/
  const std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::BitOrder::NUM_OPTIONS )> BitOrderToRegConfig = {
    Configuration::DataFormat::MSB, Configuration::DataFormat::LSB
  };

  const std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::ClockMode::NUM_OPTIONS )> ClockModeToRegConfig = {
    Configuration::ClockFormat::MODE0, Configuration::ClockFormat::MODE1, Configuration::ClockFormat::MODE2,
    Configuration::ClockFormat::MODE3
  };

  const std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::ControlMode::NUM_OPTIONS )> ControlModeToRegConfig = {
    Configuration::Mode::MASTER,
    Configuration::Mode::SLAVE
  };

  const std::array<Reg32_t, static_cast<size_t>( Chimera::SPI::DataSize::NUM_OPTIONS )> DataSizeToRegConfig = {
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

  bool isSPI( const std::uintptr_t address )
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

}    // namespace Thor::Driver::SPI

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
