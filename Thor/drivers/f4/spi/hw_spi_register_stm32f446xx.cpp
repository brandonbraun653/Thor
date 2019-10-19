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
#if defined( _EMBEDDED )
  RegisterMap * SPI1_PERIPH = reinterpret_cast<RegisterMap *>( SPI1_BASE_ADDR );
  RegisterMap * SPI2_PERIPH = reinterpret_cast<RegisterMap *>( SPI2_BASE_ADDR );
  RegisterMap * SPI3_PERIPH = reinterpret_cast<RegisterMap *>( SPI3_BASE_ADDR );
  RegisterMap * SPI4_PERIPH = reinterpret_cast<RegisterMap *>( SPI4_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ), SPI1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ), SPI2_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ), SPI3_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI4_PERIPH ), SPI4_RESOURCE_INDEX }
  };

#elif defined( _SIM )
  RegisterMap * SPI1_PERIPH = nullptr;
  RegisterMap * SPI2_PERIPH = nullptr;
  RegisterMap * SPI3_PERIPH = nullptr;
  RegisterMap * SPI4_PERIPH = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
#endif

  void initializeRegisters()
  {
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

    #if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    SPI1_PERIPH = new RegisterMap;
    SPI2_PERIPH = new RegisterMap;
    SPI3_PERIPH = new RegisterMap;
    SPI4_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ SPI1_RESOURCE_INDEX ] = SPI1_PERIPH;
    PeripheralList[ SPI2_RESOURCE_INDEX ] = SPI2_PERIPH;
    PeripheralList[ SPI3_RESOURCE_INDEX ] = SPI3_PERIPH;
    PeripheralList[ SPI4_RESOURCE_INDEX ] = SPI4_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ), SPI1_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ), SPI2_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ), SPI3_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI4_PERIPH ), SPI4_RESOURCE_INDEX );
    #endif 
  }
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
