/********************************************************************************
 *  File Name:
 *    hw_usart_register_stm32f446xx.cpp
 *
 *  Description:
 *    Explicit STM32F446xx USART data and routines
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_USART ) && defined( STM32F446xx )

namespace Thor::LLD::USART
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
  }
}    // namespace Thor::LLD::USART


#endif /* TARGET_STM32F4 && THOR_DRIVER_USART */
