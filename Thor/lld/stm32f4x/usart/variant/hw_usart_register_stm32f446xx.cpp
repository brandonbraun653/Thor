/********************************************************************************
 *   File Name:
 *    hw_usart_register_stm32f446xx.cpp
 *
 *   Description:
 *    Explicit STM32F446xx USART data and routines
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/dma.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_mapping.hpp>
#include <Thor/drivers/f4/usart/hw_usart_driver.hpp>
#include <Thor/drivers/f4/usart/hw_usart_mapping.hpp>
#include <Thor/drivers/f4/usart/hw_usart_register_stm32f446xx.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_USART == 1 ) && defined( STM32F446xx )

namespace Thor::Driver::USART
{
#if defined( EMBEDDED )
  RegisterMap *USART1_PERIPH = reinterpret_cast<RegisterMap *>( USART1_BASE_ADDR );
  RegisterMap *USART2_PERIPH = reinterpret_cast<RegisterMap *>( USART2_BASE_ADDR );
  RegisterMap *USART3_PERIPH = reinterpret_cast<RegisterMap *>( USART3_BASE_ADDR );
  RegisterMap *USART6_PERIPH = reinterpret_cast<RegisterMap *>( USART6_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( USART1_PERIPH ), USART1_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( USART2_PERIPH ), USART2_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( USART3_PERIPH ), USART3_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( USART6_PERIPH ), USART6_RESOURCE_INDEX }
  };

  Chimera::Container::LightFlatMap<size_t, RegisterMap *> ChanneltoInstance{
    { 1, USART1_PERIPH }, { 2, USART2_PERIPH }, { 3, USART3_PERIPH }, { 6, USART6_PERIPH }
  };

#elif defined( _SIM )
  RegisterMap *USART1_PERIPH = nullptr;
  RegisterMap *USART2_PERIPH = nullptr;
  RegisterMap *USART3_PERIPH = nullptr;
  RegisterMap *USART6_PERIPH = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
  Chimera::Container::LightFlatMap<size_t, RegisterMap *> ChanneltoInstance;
#endif

  void initializeRegisters()
  {
    /*------------------------------------------------
    Initialize RX DMA Signals
    ------------------------------------------------*/
    RXDMASignals[ USART1_RESOURCE_INDEX ] = Thor::DMA::Source::S_USART1_RX;
    RXDMASignals[ USART2_RESOURCE_INDEX ] = Thor::DMA::Source::S_USART2_RX;
    RXDMASignals[ USART3_RESOURCE_INDEX ] = Thor::DMA::Source::S_USART3_RX;
    RXDMASignals[ USART6_RESOURCE_INDEX ] = Thor::DMA::Source::S_USART6_RX;

    /*------------------------------------------------
    Initialize TX DMA Signals
    ------------------------------------------------*/
    TXDMASignals[ USART1_RESOURCE_INDEX ] = Thor::DMA::Source::S_USART1_TX;
    TXDMASignals[ USART2_RESOURCE_INDEX ] = Thor::DMA::Source::S_USART2_TX;
    TXDMASignals[ USART3_RESOURCE_INDEX ] = Thor::DMA::Source::S_USART3_TX;
    TXDMASignals[ USART6_RESOURCE_INDEX ] = Thor::DMA::Source::S_USART6_TX;

#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    USART1_PERIPH = new RegisterMap;
    USART2_PERIPH = new RegisterMap;
    USART3_PERIPH = new RegisterMap;
    USART6_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ USART1_RESOURCE_INDEX ] = USART1_PERIPH;
    PeripheralList[ USART2_RESOURCE_INDEX ] = USART2_PERIPH;
    PeripheralList[ USART3_RESOURCE_INDEX ] = USART3_PERIPH;
    PeripheralList[ USART6_RESOURCE_INDEX ] = USART6_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( USART1_PERIPH ), USART1_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( USART2_PERIPH ), USART2_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( USART3_PERIPH ), USART3_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( USART6_PERIPH ), USART6_RESOURCE_INDEX );
#endif
  }
}    // namespace Thor::Driver::USART


namespace Thor::Driver::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_usart_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig USART_ClockConfig[ usartTableSize ];
  RegisterConfig USART_ClockConfigLP[ usartTableSize ];
  RegisterConfig USART_ResetConfig[ usartTableSize ];
  
  Configuration::ClockType_t USART_SourceClock[ usartTableSize ];

  const PCC USARTLookup = {
    USART_ClockConfig, USART_ClockConfigLP, USART_ResetConfig, USART_SourceClock, &Thor::Driver::USART::InstanceToResourceIndex,
    gpioTableSize
  };

  void USARTInit()
  {
    using namespace Thor::Driver::USART;

    /*------------------------------------------------
    USART clock enable register access lookup table
    ------------------------------------------------*/
    USART_ClockConfig[ USART1_RESOURCE_INDEX ].mask = APB2ENR_USART1EN;
    USART_ClockConfig[ USART1_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB2ENR;

    USART_ClockConfig[ USART2_RESOURCE_INDEX ].mask = APB1ENR_USART2EN;
    USART_ClockConfig[ USART2_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1ENR;

    USART_ClockConfig[ USART3_RESOURCE_INDEX ].mask = APB1ENR_USART3EN;
    USART_ClockConfig[ USART3_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1ENR;

    USART_ClockConfig[ USART6_RESOURCE_INDEX ].mask = APB2ENR_USART6EN;
    USART_ClockConfig[ USART6_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB2ENR;

    /*------------------------------------------------
    USART low power clock enable register access lookup table
    ------------------------------------------------*/
    USART_ClockConfigLP[ USART1_RESOURCE_INDEX ].mask = APB2LPENR_USART1LPEN;
    USART_ClockConfigLP[ USART1_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB2LPENR;

    USART_ClockConfigLP[ USART2_RESOURCE_INDEX ].mask = APB1LPENR_USART2LPEN;
    USART_ClockConfigLP[ USART2_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1LPENR;

    USART_ClockConfigLP[ USART3_RESOURCE_INDEX ].mask = APB1LPENR_USART3LPEN;
    USART_ClockConfigLP[ USART3_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1LPENR;

    USART_ClockConfigLP[ USART6_RESOURCE_INDEX ].mask = APB2LPENR_USART6LPEN;
    USART_ClockConfigLP[ USART6_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB2LPENR;

    /*------------------------------------------------
    USART reset register access lookup table
    ------------------------------------------------*/
    USART_ResetConfig[ USART1_RESOURCE_INDEX ].mask = APB2RSTR_USART1RST;
    USART_ResetConfig[ USART1_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB2RSTR;

    USART_ResetConfig[ USART2_RESOURCE_INDEX ].mask = APB1RSTR_USART2RST;
    USART_ResetConfig[ USART2_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1RSTR;

    USART_ResetConfig[ USART3_RESOURCE_INDEX ].mask = APB1RSTR_USART3RST;
    USART_ResetConfig[ USART3_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1RSTR;

    USART_ResetConfig[ USART6_RESOURCE_INDEX ].mask = APB2RSTR_USART6RST;
    USART_ResetConfig[ USART6_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB2RSTR;

    /*------------------------------------------------
    USART clocking bus source identifier
    ------------------------------------------------*/
    USART_SourceClock[ USART1_RESOURCE_INDEX ] = Configuration::ClockType::PCLK2;
    USART_SourceClock[ USART2_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    USART_SourceClock[ USART3_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    USART_SourceClock[ USART6_RESOURCE_INDEX ] = Configuration::ClockType::PCLK2;
  }

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
