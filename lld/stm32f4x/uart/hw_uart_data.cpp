/********************************************************************************
 *  File Name:
 *    hw_uart_data.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/hld/dma/hld_dma_intf.hpp>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/uart>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_UART )

namespace Thor::LLD::UART
{
  /*-------------------------------------------------------------------------------
  Public Data
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------*/
#if defined( STM32_UART4_PERIPH_AVAILABLE )
  RegisterMap *UART4_PERIPH = reinterpret_cast<RegisterMap *>( UART4_BASE_ADDR );
#endif
#if defined( STM32_UART5_PERIPH_AVAILABLE )
  RegisterMap *UART5_PERIPH = reinterpret_cast<RegisterMap *>( UART5_BASE_ADDR );
#endif

  /*-------------------------------------------------
  Configuration Maps
  -------------------------------------------------*/
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

  /*-------------------------------------------------
  Peripheral Resources
  -------------------------------------------------*/
  namespace Resource
  { /* clang-format off */
    LLD_CONST Thor::LLD::DMA::Source RXDMASignals[ NUM_UART_PERIPHS ] = {
#if defined( STM32_UART4_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::UART4_RX,
#endif
#if defined( STM32_UART5_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::UART5_RX,
#endif
    };


    LLD_CONST Thor::LLD::DMA::Source TXDMASignals[ NUM_UART_PERIPHS ] = {
#if defined( STM32_UART4_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::UART4_TX,
#endif
#if defined( STM32_UART5_PERIPH_AVAILABLE )
      Thor::LLD::DMA::Source::UART5_TX,
#endif
    };


    LLD_CONST IRQn_Type IRQSignals[ NUM_UART_PERIPHS ] = {
#if defined( STM32_UART4_PERIPH_AVAILABLE )
      UART4_IRQn,
#endif
#if defined( STM32_UART5_PERIPH_AVAILABLE )
      UART5_IRQn,
#endif
    };
  } /* clang-format on */


  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static Driver s_usart_drivers[ NUM_UART_PERIPHS ];


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_usart_drivers, ARRAY_COUNT( s_usart_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }


  bool isChannelSupported( const Chimera::Serial::Channel channel )
  {
    if ( channel < Chimera::Serial::Channel::NUM_OPTIONS )
    {
      return ( getResourceIndex( channel ) != INVALID_RESOURCE_INDEX );
    }
    else
    {
      return false;
    }
  }


  Driver_rPtr getDriver( const Chimera::Serial::Channel channel )
  {
    if ( isChannelSupported( channel ) )
    {
      return &s_usart_drivers[ static_cast<size_t>( channel ) ];
    }

    return nullptr;
  }

}

#endif /* TARGET_STM32F4 && THOR_LLD_UART */
