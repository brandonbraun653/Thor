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
#include <Thor/cfg>
#include <Thor/dma>
#include <Thor/uart>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_UART ) && defined( STM32F446xx )

namespace Thor::LLD::UART
{
#if defined( EMBEDDED )
  RegisterMap *UART4_PERIPH = reinterpret_cast<RegisterMap *>( UART4_BASE_ADDR );
  RegisterMap *UART5_PERIPH = reinterpret_cast<RegisterMap *>( UART5_BASE_ADDR );

  Chimera::Container::LightFlatMap<std::uintptr_t, size_t> InstanceToResourceIndex{
    { reinterpret_cast<std::uintptr_t>( UART4_PERIPH ), UART4_RESOURCE_INDEX },
    { reinterpret_cast<std::uintptr_t>( UART5_PERIPH ), UART5_RESOURCE_INDEX }
  };


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
  }
}    // namespace Thor::LLD::UART

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
