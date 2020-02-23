/********************************************************************************
 *  File Name:
 *    hw_uart_register_stm32f446xx.cpp
 *
 *  Description:
 *    Explicit STM32F446xx UART data and routines
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/lld/stm32f4x/rcc/hw_rcc_mapping.hpp>
#include <Thor/uart>

#if defined( TARGET_STM32F4 ) && ( THOR_LLD_UART ) && defined( STM32F446xx )

namespace Thor::LLD::UART
{
#if defined( EMBEDDED )
  RegisterMap *UART4_PERIPH = reinterpret_cast<RegisterMap *>( UART4_BASE_ADDR );
  RegisterMap *UART5_PERIPH = reinterpret_cast<RegisterMap *>( UART5_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( UART4_PERIPH ), UART4_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( UART5_PERIPH ), UART5_RESOURCE_INDEX }
  };

#elif defined( _SIM )
  RegisterMap *UART4_PERIPH = nullptr;
  RegisterMap *UART5_PERIPH = nullptr;

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex;
  Chimera::Container::LightFlatMap<size_t, RegisterMap *> ChanneltoInstance;
#endif

  void initializeRegisters()
  {
    /*------------------------------------------------
    Initialize RX DMA Signals
    ------------------------------------------------*/
    RXDMASignals[ UART4_RESOURCE_INDEX ] = Thor::DMA::Source::S_UART4_RX;
    RXDMASignals[ UART5_RESOURCE_INDEX ] = Thor::DMA::Source::S_UART5_RX;

    /*------------------------------------------------
    Initialize TX DMA Signals
    ------------------------------------------------*/
    TXDMASignals[ UART4_RESOURCE_INDEX ] = Thor::DMA::Source::S_UART4_TX;
    TXDMASignals[ UART5_RESOURCE_INDEX ] = Thor::DMA::Source::S_UART5_TX;

#if defined( _SIM )
    /*------------------------------------------------
    Allocate some memory to simulate the register blocks
    ------------------------------------------------*/
    UART4_PERIPH = new RegisterMap;
    UART5_PERIPH = new RegisterMap;

    /*------------------------------------------------
    Update the memory listing
    ------------------------------------------------*/
    PeripheralList[ UART4_RESOURCE_INDEX ] = UART4_PERIPH;
    PeripheralList[ UART5_RESOURCE_INDEX ] = UART5_PERIPH;

    /*------------------------------------------------
    Update the resource indexer now that the registers actually exist
    ------------------------------------------------*/
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( UART4_PERIPH ), UART4_RESOURCE_INDEX );
    InstanceToResourceIndex.append( reinterpret_cast<std::uintptr_t>( UART5_PERIPH ), UART5_RESOURCE_INDEX );
#endif
  }
}    // namespace Thor::LLD::UART


namespace Thor::LLD::RCC::LookupTables
{
  /*------------------------------------------------
  Lookup tables for register access on a peripheral by peripheral basis.
  Indexing must match the lookup table hw_uart_mapping.hpp
  ------------------------------------------------*/
  RegisterConfig UART_ClockConfig[ uartTableSize ];
  RegisterConfig UART_ClockConfigLP[ uartTableSize ];
  RegisterConfig UART_ResetConfig[ uartTableSize ];
  
  Configuration::ClockType_t UART_SourceClock[ uartTableSize ];

  const PCC UARTLookup = {
    UART_ClockConfig, UART_ClockConfigLP, UART_ResetConfig, UART_SourceClock, &Thor::LLD::UART::InstanceToResourceIndex,
    gpioTableSize
  };

  void UARTInit()
  {
    using namespace Thor::LLD::UART;

    /*------------------------------------------------
    UART clock enable register access lookup table
    ------------------------------------------------*/
    UART_ClockConfig[ UART4_RESOURCE_INDEX ].mask = APB1ENR_UART4EN;
    UART_ClockConfig[ UART4_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1ENR;

    UART_ClockConfig[ UART5_RESOURCE_INDEX ].mask = APB1ENR_UART5EN;
    UART_ClockConfig[ UART5_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1ENR;

    /*------------------------------------------------
    UART low power clock enable register access lookup table
    ------------------------------------------------*/
    UART_ClockConfigLP[ UART4_RESOURCE_INDEX ].mask = APB1LPENR_UART4LPEN;
    UART_ClockConfigLP[ UART4_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1LPENR;

    UART_ClockConfigLP[ UART5_RESOURCE_INDEX ].mask = APB1LPENR_UART5LPEN;
    UART_ClockConfigLP[ UART5_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1LPENR;

    /*------------------------------------------------
    UART reset register access lookup table
    ------------------------------------------------*/
    UART_ResetConfig[ UART4_RESOURCE_INDEX ].mask = APB1RSTR_UART4RST;
    UART_ResetConfig[ UART4_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1RSTR;

    UART_ResetConfig[ UART5_RESOURCE_INDEX ].mask = APB1RSTR_UART5RST;
    UART_ResetConfig[ UART5_RESOURCE_INDEX ].reg = &RCC1_PERIPH->APB1RSTR;

    /*------------------------------------------------
    UART clocking bus source identifier
    ------------------------------------------------*/
    UART_SourceClock[ UART4_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
    UART_SourceClock[ UART5_RESOURCE_INDEX ] = Configuration::ClockType::PCLK1;
  }

}

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
