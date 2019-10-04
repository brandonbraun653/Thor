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
#include <Thor/drivers/f4/spi/hw_spi_prj.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  /*------------------------------------------------
  Peripheral DMA Signals
  ------------------------------------------------*/
  DMASignalList RXDMASignals;
  DMASignalList TXDMASignals;

  /*------------------------------------------------
  Low Level Driver Instances
  ------------------------------------------------*/
  DriverInstanceList spiObjects;

  /*------------------------------------------------
  Maps a SPI peripheral into the corresponding resource index
  ------------------------------------------------*/
  const Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{ 
    { reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ), SPI1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ), SPI2_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ), SPI3_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI4_PERIPH ), SPI4_RESOURCE_INDEX }
  };

}    // namespace Thor::Driver::SPI

#endif  /* TARGET_STM32F4 && THOR_DRIVER_SPI */
