/********************************************************************************
 *  File Name:
 *    hw_usart_register_stm32l432kc.cpp
 *
 *  Description:
 *    USART register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_driver.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_mapping.hpp>
#include <Thor/lld/stm32l4x/usart/variant/hw_usart_register_stm32l4xxxx.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{
  /* clang-format off */
  const std::array<Chimera::Serial::Channel, NUM_USART_PERIPHS> supportedChannels = {
#if defined ( STM32_USART1_PERIPH_AVAILABLE )
    Chimera::Serial::Channel::SERIAL1,
#else
    Chimera::Serial::Channel::NOT_SUPPORTED,
#endif

#if defined ( STM32_USART2_PERIPH_AVAILABLE )
    Chimera::Serial::Channel::SERIAL2,
#else
    Chimera::Serial::Channel::NOT_SUPPORTED,
#endif

#if defined ( STM32_USART3_PERIPH_AVAILABLE )
    Chimera::Serial::Channel::SERIAL3,
#else
    Chimera::Serial::Channel::NOT_SUPPORTED,
#endif
  };

  DMASignalList RXDMASignals = {
    Thor::DMA::Source::S_USART1_RX,
    Thor::DMA::Source::S_USART2_RX,
    Thor::DMA::Source::S_USART3_RX
  };

  DMASignalList TXDMASignals = {
    Thor::DMA::Source::S_USART1_TX,
    Thor::DMA::Source::S_USART2_TX,
    Thor::DMA::Source::S_USART3_TX
  };

  IRQSignalList IRQSignals = {
    USART1_IRQn,
    USART2_IRQn,
    USART3_IRQn
  };

#if defined( EMBEDDED )
  /*-------------------------------------------------
  Memory Mapped Structs to Peripherals
  -------------------------------------------------*/
  RegisterMap *USART1_PERIPH = reinterpret_cast<RegisterMap *>( USART1_BASE_ADDR );
  RegisterMap *USART2_PERIPH = reinterpret_cast<RegisterMap *>( USART2_BASE_ADDR );
  RegisterMap *USART3_PERIPH = reinterpret_cast<RegisterMap *>( USART3_BASE_ADDR );

  /*-------------------------------------------------
  Lookup Tables Defintions
  -------------------------------------------------*/
  Thor::LLD::RIndexMap InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( USART1_PERIPH ), USART1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( USART2_PERIPH ), USART2_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( USART3_PERIPH ), USART3_RESOURCE_INDEX }
  };

  Chimera::Container::LightFlatMap<Chimera::Serial::Channel, RegisterMap *> ChannelToInstance{
    { Chimera::Serial::Channel::SERIAL1, USART1_PERIPH },
    { Chimera::Serial::Channel::SERIAL2, USART2_PERIPH },
    { Chimera::Serial::Channel::SERIAL3, USART3_PERIPH }
  };
  /* clang-format on */

#elif defined( _SIM )
  /*-------------------------------------------------
  Memory Mapped Structs to Virtual Peripherals
  -------------------------------------------------*/
  RegisterMap *USART1_PERIPH = nullptr;
  RegisterMap *USART2_PERIPH = nullptr;
  RegisterMap *USART3_PERIPH = nullptr;

  /*-------------------------------------------------
  Lookup Tables Definitions
  -------------------------------------------------*/
  Thor::LLD::RIndexMap InstanceToResourceIndex;
  Chimera::Container::LightFlatMap<size_t, RegisterMap *> ChannelToInstance;
#endif

  void initializeRegisters()
  {
#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    USART1_PERIPH = new RegisterMap;
    USART2_PERIPH = new RegisterMap;
    USART3_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ USART1_RESOURCE_INDEX ] = USART1_PERIPH;
    PeripheralList[ USART2_RESOURCE_INDEX ] = USART2_PERIPH;
    PeripheralList[ USART3_RESOURCE_INDEX ] = USART3_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( USART1_PERIPH ), USART1_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( USART2_PERIPH ), USART2_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( USART3_PERIPH ), USART3_RESOURCE_INDEX );

    ChannelToInstance.append( Chimera::Serial::Channel::SERIAL1, USART1_PERIPH );
    ChannelToInstance.append( Chimera::Serial::Channel::SERIAL2, USART2_PERIPH );
    ChannelToInstance.append( Chimera::Serial::Channel::SERIAL3, USART3_PERIPH );
#endif
  }
}    // namespace Thor::LLD::USART

namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_usart_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig USART_ClockConfig[ Thor::LLD::USART::NUM_USART_PERIPHS ];
  RegisterConfig USART_ClockConfigLP[ Thor::LLD::USART::NUM_USART_PERIPHS ];
  RegisterConfig USART_ResetConfig[ Thor::LLD::USART::NUM_USART_PERIPHS ];
  Chimera::Clock::Bus USART_SourceClock[ Thor::LLD::USART::NUM_USART_PERIPHS ];

  PCC USARTLookup = { USART_ClockConfig,
                      nullptr,
                      USART_ResetConfig,
                      USART_SourceClock,
                      &Thor::LLD::USART::InstanceToResourceIndex,
                      Thor::LLD::USART::NUM_USART_PERIPHS };

  void USARTInit()
  {
    using namespace Thor::LLD::USART;

    /*------------------------------------------------
    USART clock enable register access lookup table
    ------------------------------------------------*/
    #if defined ( STM32_USART1_PERIPH_AVAILABLE )
    USART_ClockConfig[ USART1_RESOURCE_INDEX ].mask = APB2ENR_USART1EN;
    USART_ClockConfig[ USART1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2ENR;
    #endif

    #if defined ( STM32_USART2_PERIPH_AVAILABLE )
    USART_ClockConfig[ USART2_RESOURCE_INDEX ].mask = APB1ENR1_USART2EN;
    USART_ClockConfig[ USART2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
    #endif

    #if defined ( STM32_USART3_PERIPH_AVAILABLE )
    USART_ClockConfig[ USART3_RESOURCE_INDEX ].mask = APB1ENR1_USART3EN;
    USART_ClockConfig[ USART3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1ENR1;
    #endif

    /*------------------------------------------------
    USART reset register access lookup table
    ------------------------------------------------*/
    #if defined ( STM32_USART1_PERIPH_AVAILABLE )
    USART_ResetConfig[ USART1_RESOURCE_INDEX ].mask = APB2RSTR_USART1RST;
    USART_ResetConfig[ USART1_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB2RSTR;
    #endif

    #if defined ( STM32_USART2_PERIPH_AVAILABLE )
    USART_ResetConfig[ USART2_RESOURCE_INDEX ].mask = APB1RSTR1_USART2RST;
    USART_ResetConfig[ USART2_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;
    #endif

    #if defined ( STM32_USART3_PERIPH_AVAILABLE )
    USART_ResetConfig[ USART3_RESOURCE_INDEX ].mask = APB1RSTR1_USART3RST;
    USART_ResetConfig[ USART3_RESOURCE_INDEX ].reg  = &RCC1_PERIPH->APB1RSTR1;
    #endif

    /*------------------------------------------------
    USART clocking bus source identifier
    ------------------------------------------------*/
    #if defined ( STM32_USART1_PERIPH_AVAILABLE )
    USART_SourceClock[ USART1_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB2;
    #endif

    #if defined ( STM32_USART2_PERIPH_AVAILABLE )
    USART_SourceClock[ USART2_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
    #endif

    #if defined ( STM32_USART3_PERIPH_AVAILABLE )
    USART_SourceClock[ USART3_RESOURCE_INDEX ] = Chimera::Clock::Bus::APB1;
    #endif
  };

}    // namespace Thor::LLD::RCC::LookupTables

#endif /* STM32L432xx && THOR_LLD_USART */
