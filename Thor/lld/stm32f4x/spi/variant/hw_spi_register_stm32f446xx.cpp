/********************************************************************************
 *  File Name:
 *    hw_spi_register_stm32f446xx.cpp
 *
 *  Description:
 *    Explicit STM32F446xx SPI data and routines
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/hld/dma/dma_definitions.hpp>
#include <Thor/lld/stm32f4x/interrupt/hw_it_prj.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_driver.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_mapping.hpp>
#include <Thor/lld/stm32f4x/spi/hw_spi_types.hpp>
#include <Thor/lld/stm32f4x/spi/variant/hw_spi_register_stm32f446xx.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_LLD_SPI ) && defined( STM32F446xx )

namespace Thor::Driver::SPI
{
#if defined( EMBEDDED )
  RegisterMap *SPI1_PERIPH = reinterpret_cast<RegisterMap *>( SPI1_BASE_ADDR );
  RegisterMap *SPI2_PERIPH = reinterpret_cast<RegisterMap *>( SPI2_BASE_ADDR );
  RegisterMap *SPI3_PERIPH = reinterpret_cast<RegisterMap *>( SPI3_BASE_ADDR );
  RegisterMap *SPI4_PERIPH = reinterpret_cast<RegisterMap *>( SPI4_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ), SPI1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ), SPI2_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ), SPI3_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI4_PERIPH ), SPI4_RESOURCE_INDEX }
  };

  Chimera::Container::LightFlatMap<size_t, RegisterMap *> ChannelToInstance{ { SPI1_CHANNEL_NUMBER, SPI1_PERIPH },
                                                                             { SPI2_CHANNEL_NUMBER, SPI2_PERIPH },
                                                                             { SPI3_CHANNEL_NUMBER, SPI3_PERIPH },
                                                                             { SPI4_CHANNEL_NUMBER, SPI4_PERIPH } };

#elif defined( _SIM )
  RegisterMap *SPI1_PERIPH = nullptr;
  RegisterMap *SPI2_PERIPH = nullptr;
  RegisterMap *SPI3_PERIPH = nullptr;
  RegisterMap *SPI4_PERIPH = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
  Chimera::Container::LightFlatMap<size_t, RegisterMap *> ChannelToInstance;
#endif

  const std::array<uint8_t, NUM_SPI_PERIPHS> supportedChannels = { SPI1_CHANNEL_NUMBER, SPI2_CHANNEL_NUMBER,
                                                                   SPI3_CHANNEL_NUMBER, SPI4_CHANNEL_NUMBER };

  const DMASignalList RXDMASignals = {
    Thor::DMA::Source::S_SPI1_RX,
    Thor::DMA::Source::S_SPI2_RX,
    Thor::DMA::Source::S_SPI3_RX,
    Thor::DMA::Source::S_SPI4_RX,
  };

  const DMASignalList TXDMASignals = {
    Thor::DMA::Source::S_SPI1_TX,
    Thor::DMA::Source::S_SPI2_TX,
    Thor::DMA::Source::S_SPI3_TX,
    Thor::DMA::Source::S_SPI4_TX,
  };

  const IRQSignalList IRQSignals = { SPI1_IRQn, SPI2_IRQn, SPI3_IRQn, SPI4_IRQn };

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    SPI1_PERIPH = new RegisterMap;
    SPI2_PERIPH = new RegisterMap;
    SPI3_PERIPH = new RegisterMap;
    SPI4_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ), SPI1_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ), SPI2_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ), SPI3_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI4_PERIPH ), SPI4_RESOURCE_INDEX );

    ChannelToInstance.append( SPI1_CHANNEL_NUMBER, SPI1_PERIPH );
    ChannelToInstance.append( SPI2_CHANNEL_NUMBER, SPI2_PERIPH );
    ChannelToInstance.append( SPI3_CHANNEL_NUMBER, SPI3_PERIPH );
    ChannelToInstance.append( SPI4_CHANNEL_NUMBER, SPI4_PERIPH );
#endif

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ SPI1_RESOURCE_INDEX ] = SPI1_PERIPH;
    PeripheralList[ SPI2_RESOURCE_INDEX ] = SPI2_PERIPH;
    PeripheralList[ SPI3_RESOURCE_INDEX ] = SPI3_PERIPH;
    PeripheralList[ SPI4_RESOURCE_INDEX ] = SPI4_PERIPH;
  }
}    // namespace Thor::Driver::SPI


namespace Thor::Driver::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_spi_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig SPI_ClockConfig[ spiTableSize ];
  RegisterConfig SPI_ClockConfigLP[ spiTableSize ];
  RegisterConfig SPI_ResetConfig[ spiTableSize ];

  Configuration::ClockType_t SPI_SourceClock[ spiTableSize ];

  const PCC SPILookup = {
    SPI_ClockConfig, SPI_ClockConfigLP, SPI_ResetConfig, SPI_SourceClock, &Thor::Driver::SPI::InstanceToResourceIndex,
    gpioTableSize
  };

  void SPIInit()
  {
    using namespace Thor::Driver::SPI;

    /*------------------------------------------------
    SPI clock enable register access lookup table
    ------------------------------------------------*/
    SPI_ClockConfig[ SPI1_RESOURCE_INDEX ].mask = APB2ENR_SPI1EN;
    SPI_ClockConfig[ SPI1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;

    SPI_ClockConfig[ SPI2_RESOURCE_INDEX ].mask = APB1ENR_SPI2EN;
    SPI_ClockConfig[ SPI2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR;

    SPI_ClockConfig[ SPI3_RESOURCE_INDEX ].mask = APB1ENR_SPI3EN;
    SPI_ClockConfig[ SPI3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR;

    SPI_ClockConfig[ SPI4_RESOURCE_INDEX ].mask = APB2ENR_SPI4EN;
    SPI_ClockConfig[ SPI4_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;

    /*------------------------------------------------
    SPI low power clock enable register access lookup table
    ------------------------------------------------*/
    SPI_ClockConfigLP[ SPI1_RESOURCE_INDEX ].mask = APB2LPENR_SPI1LPEN;
    SPI_ClockConfigLP[ SPI1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2LPENR;

    SPI_ClockConfigLP[ SPI2_RESOURCE_INDEX ].mask = APB1LPENR_SPI2LPEN;
    SPI_ClockConfigLP[ SPI2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1LPENR;

    SPI_ClockConfigLP[ SPI3_RESOURCE_INDEX ].mask = APB1LPENR_SPI3LPEN;
    SPI_ClockConfigLP[ SPI3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1LPENR;

    SPI_ClockConfigLP[ SPI4_RESOURCE_INDEX ].mask = APB2LPENR_SPI4LPEN;
    SPI_ClockConfigLP[ SPI4_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2LPENR;

    /*------------------------------------------------
    SPI reset register access lookup table
    ------------------------------------------------*/
    SPI_ResetConfig[ SPI1_RESOURCE_INDEX ].mask = APB2RSTR_SPI1RST;
    SPI_ResetConfig[ SPI1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;

    SPI_ResetConfig[ SPI2_RESOURCE_INDEX ].mask = APB1RSTR_SPI2RST;
    SPI_ResetConfig[ SPI2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR;

    SPI_ResetConfig[ SPI3_RESOURCE_INDEX ].mask = APB1RSTR_SPI3RST;
    SPI_ResetConfig[ SPI3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR;

    SPI_ResetConfig[ SPI4_RESOURCE_INDEX ].mask = APB2RSTR_SPI4RST;
    SPI_ResetConfig[ SPI4_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;

    /*------------------------------------------------
    SPI clocking bus source identifier
    ------------------------------------------------*/
    SPI_SourceClock[ SPI1_RESOURCE_INDEX ] = Configuration::ClockType::PCLK2;
    SPI_SourceClock[ SPI2_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    SPI_SourceClock[ SPI3_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    SPI_SourceClock[ SPI4_RESOURCE_INDEX ] = Configuration::ClockType::PCLK2;
  }

}    // namespace Thor::Driver::RCC::LookupTables

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
