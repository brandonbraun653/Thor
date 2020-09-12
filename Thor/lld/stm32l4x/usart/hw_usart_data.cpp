/********************************************************************************
 *  File Name:
 *    hw_usart_data.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/interface/usart/usart_prv_data.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_prj.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------------------------------------*/
  RegisterMap *USART1_PERIPH = reinterpret_cast<RegisterMap *>( USART1_BASE_ADDR );
  RegisterMap *USART2_PERIPH = reinterpret_cast<RegisterMap *>( USART2_BASE_ADDR );
  RegisterMap *USART3_PERIPH = reinterpret_cast<RegisterMap *>( USART3_BASE_ADDR );


  /*-------------------------------------------------------------------------------
  Configuration Maps
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  { /* clang-format off */
    LLD_CONST Reg32_t CharWidth[ static_cast<size_t>( Chimera::Serial::CharWid::NUM_OPTIONS ) ] = {
      Configuration::WordLength::LEN_8BIT,
      Configuration::WordLength::LEN_9BIT
    };

    LLD_CONST Reg32_t Parity[ static_cast<size_t>( Chimera::Serial::Parity::NUM_OPTIONS ) ] = {
      Configuration::Parity::NONE,
      Configuration::Parity::EVEN,
      Configuration::Parity::ODD
    };

    LLD_CONST Reg32_t StopBits[ static_cast<size_t>( Chimera::Serial::StopBits::NUM_OPTIONS ) ] = {
      Configuration::Stop::BIT_1,
      Configuration::Stop::BIT_1_5,
      Configuration::Stop::BIT_2
    };
  } /* clang-format on */

  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  { /* clang-format off */
    LLD_CONST Reg32_t RXDMASignals[ NUM_USART_PERIPHS ] = {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
      Thor::DMA::Source::S_USART1_RX,
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
      Thor::DMA::Source::S_USART2_RX,
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
      Thor::DMA::Source::S_USART3_RX
#endif
    };


    LLD_CONST Reg32_t TXDMASignals[ NUM_USART_PERIPHS ] = {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
      Thor::DMA::Source::S_USART1_TX,
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
      Thor::DMA::Source::S_USART2_TX,
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
      Thor::DMA::Source::S_USART3_TX
#endif 
    };


    LLD_CONST IRQn_Type IRQSignals[ NUM_USART_PERIPHS ] = {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
      USART1_IRQn,
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
      USART2_IRQn,
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
      USART3_IRQn
#endif
    };
  } /* clang-format on */

}    // namespace Thor::LLD::USART

#endif /* TARGET_STM32L4 && THOR_LLD_USART */
