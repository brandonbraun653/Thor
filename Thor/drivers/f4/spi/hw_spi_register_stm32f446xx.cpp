/********************************************************************************
 *   File Name:
 *    hw_spi_register_stm32f446xx.cpp
 *
 *   Description:
 *    Explicit STM32F446xx SPI data and routines
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/dma.hpp>
#include <Thor/drivers/f4/spi/hw_spi_driver.hpp>
#include <Thor/drivers/f4/spi/hw_spi_mapping.hpp>
#include <Thor/drivers/f4/spi/hw_spi_register_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 ) && defined( STM32F446xx )

namespace Thor::Driver::SPI
{
  void initializeSystemDriver()
  {
    spiObjects.fill( nullptr );

    /*------------------------------------------------
    Initialize RX DMA Signals
    ------------------------------------------------*/
    RXDMASignals[ SPI1_RESOURCE_INDEX ] = Thor::DMA::Source::S_SPI1_RX;
    RXDMASignals[ SPI2_RESOURCE_INDEX ] = Thor::DMA::Source::S_SPI2_RX;
    RXDMASignals[ SPI3_RESOURCE_INDEX ] = Thor::DMA::Source::S_SPI3_RX;
    RXDMASignals[ SPI4_RESOURCE_INDEX ] = Thor::DMA::Source::S_SPI4_RX;

    /*------------------------------------------------
    Initialize TX DMA Signals
    ------------------------------------------------*/
    TXDMASignals[ SPI1_RESOURCE_INDEX ] = Thor::DMA::Source::S_SPI1_TX;
    TXDMASignals[ SPI2_RESOURCE_INDEX ] = Thor::DMA::Source::S_SPI2_TX;
    TXDMASignals[ SPI3_RESOURCE_INDEX ] = Thor::DMA::Source::S_SPI3_TX;
    TXDMASignals[ SPI4_RESOURCE_INDEX ] = Thor::DMA::Source::S_SPI4_TX;

  }
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
