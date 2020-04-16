/********************************************************************************
 *  File Name:
 *    hw_spi_register_stm32l4xxxx.cpp
 *
 *  Description:
 *    SPI register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_driver.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_mapping.hpp>
#include <Thor/lld/stm32l4x/spi/variant/hw_spi_register_stm32l4xxxx.hpp>
#include <Thor/lld/stm32l4x/spi/hw_spi_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_SPI )

namespace Thor::LLD::SPI
{
  const std::array<uint8_t, NUM_SPI_PERIPHS> supportedChannels = { SPI1_CHANNEL_NUMBER, SPI2_CHANNEL_NUMBER,
                                                                   SPI3_CHANNEL_NUMBER };

  const DMASignalList RXDMASignals = { Thor::DMA::Source::S_SPI1_RX, Thor::DMA::Source::S_SPI2_RX,
                                       Thor::DMA::Source::S_SPI3_RX };

  const DMASignalList TXDMASignals = { Thor::DMA::Source::S_SPI1_TX, Thor::DMA::Source::S_SPI2_TX,
                                       Thor::DMA::Source::S_SPI3_TX };

  const IRQSignalList IRQSignals = { SPI1_IRQn, SPI2_IRQn, SPI3_IRQn };


#if defined( EMBEDDED )
  /*-------------------------------------------------
  Memory Mapped Structs to Peripherals
  -------------------------------------------------*/
  RegisterMap *SPI1_PERIPH = reinterpret_cast<RegisterMap *>( SPI1_BASE_ADDR );
  RegisterMap *SPI2_PERIPH = reinterpret_cast<RegisterMap *>( SPI2_BASE_ADDR );
  RegisterMap *SPI3_PERIPH = reinterpret_cast<RegisterMap *>( SPI3_BASE_ADDR );

  /*-------------------------------------------------
  Lookup Tables Defintions
  -------------------------------------------------*/
  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ), SPI1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ), SPI2_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ), SPI3_RESOURCE_INDEX }
  };

  Chimera::Container::LightFlatMap<size_t, RegisterMap *> ChannelToInstance{ { SPI1_CHANNEL_NUMBER, SPI1_PERIPH },
                                                                             { SPI2_CHANNEL_NUMBER, SPI2_PERIPH },
                                                                             { SPI3_CHANNEL_NUMBER, SPI3_PERIPH }};

#elif defined( _SIM )
  /*-------------------------------------------------
  Memory Mapped Structs to Virtual Peripherals
  -------------------------------------------------*/
  RegisterMap *SPI1_PERIPH = nullptr;
  RegisterMap *SPI2_PERIPH = nullptr;
  RegisterMap *SPI3_PERIPH = nullptr;

  /*-------------------------------------------------
  Lookup Tables Definitions
  -------------------------------------------------*/
  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
  Chimera::Container::LightFlatMap<size_t, RegisterMap *> ChannelToInstance;
#endif

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    SPI1_PERIPH = new RegisterMap;
    SPI2_PERIPH = new RegisterMap;
    SPI3_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ SPI1_RESOURCE_INDEX ] = SPI1_PERIPH;
    PeripheralList[ SPI2_RESOURCE_INDEX ] = SPI2_PERIPH;
    PeripheralList[ SPI3_RESOURCE_INDEX ] = SPI3_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI1_PERIPH ), SPI1_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI2_PERIPH ), SPI2_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( SPI3_PERIPH ), SPI3_RESOURCE_INDEX );

    ChannelToInstance.append( SPI1_CHANNEL_NUMBER, SPI1_PERIPH );
    ChannelToInstance.append( SPI2_CHANNEL_NUMBER, SPI2_PERIPH );
    ChannelToInstance.append( SPI3_CHANNEL_NUMBER, SPI3_PERIPH );
#endif
  }
}    // namespace Thor::LLD::GPIO

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_spi_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig SPI_ClockConfig[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
  RegisterConfig SPI_ResetConfig[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];
  Chimera::Clock::Bus SPI_SourceClock[ Thor::LLD::SPI::NUM_SPI_PERIPHS ];

  PCC SPILookup = { SPI_ClockConfig,
                    nullptr,
                    SPI_ResetConfig,
                    SPI_SourceClock,
                    &Thor::LLD::SPI::InstanceToResourceIndex,
                    Thor::LLD::SPI::NUM_SPI_PERIPHS };

  void SPIInit()
  {
    using namespace Thor::LLD::SPI;

    /*------------------------------------------------
    SPI clock enable register access lookup table
    ------------------------------------------------*/
    #if defined ( STM32_SPI1_PERIPH_AVAILABLE )
    SPI_ClockConfig[ SPI1_RESOURCE_INDEX ].mask = APB2ENR_SPI1EN;
    SPI_ClockConfig[ SPI1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;
    #endif
    
    #if defined ( STM32_SPI2_PERIPH_AVAILABLE )
    SPI_ClockConfig[ SPI2_RESOURCE_INDEX ].mask = APB1ENR1_SPI2EN;
    SPI_ClockConfig[ SPI2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
    #endif 

    #if defined ( STM32_SPI3_PERIPH_AVAILABLE )
    SPI_ClockConfig[ SPI3_RESOURCE_INDEX ].mask = APB1ENR1_SPI3EN;
    SPI_ClockConfig[ SPI3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;
    #endif 

    /*------------------------------------------------
    SPI reset register access lookup table
    ------------------------------------------------*/
    #if defined ( STM32_SPI1_PERIPH_AVAILABLE )
    SPI_ResetConfig[ SPI1_RESOURCE_INDEX ].mask = APB2RSTR_SPI1RST;
    SPI_ResetConfig[ SPI1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;
    #endif

    #if defined ( STM32_SPI2_PERIPH_AVAILABLE )
    #pragma message( "NEED SPI 2 DEFINITIONS" )
    #endif 
    
    #if defined ( STM32_SPI3_PERIPH_AVAILABLE )
    SPI_ResetConfig[ SPI1_RESOURCE_INDEX ].mask = APB1RSTR1_SPI3RST;
    SPI_ResetConfig[ SPI1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;
    #endif

    /*------------------------------------------------
    SPI clocking bus source identifier
    ------------------------------------------------*/
    #if defined ( STM32_SPI1_PERIPH_AVAILABLE )
    SPI_SourceClock[ SPI1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB2;
    #endif
    
    #if defined ( STM32_SPI2_PERIPH_AVAILABLE )
    SPI_SourceClock[ SPI1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
    #endif 

    #if defined ( STM32_SPI3_PERIPH_AVAILABLE )
    SPI_SourceClock[ SPI1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
    #endif 
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_SPI */
