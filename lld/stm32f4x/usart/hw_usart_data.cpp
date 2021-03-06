/********************************************************************************
 *  File Name:
 *    hw_usart_data.cpp
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
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/usart>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_USART )

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Public Data
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Peripheral Memory Maps
  -------------------------------------------------*/
  RegisterMap *USART1_PERIPH = reinterpret_cast<RegisterMap *>( USART1_BASE_ADDR );
  RegisterMap *USART2_PERIPH = reinterpret_cast<RegisterMap *>( USART2_BASE_ADDR );
  RegisterMap *USART3_PERIPH = reinterpret_cast<RegisterMap *>( USART3_BASE_ADDR );
  RegisterMap *USART6_PERIPH = reinterpret_cast<RegisterMap *>( USART6_BASE_ADDR );

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
    LLD_CONST DMA::Source RXDMASignals[ NUM_USART_PERIPHS ] = {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
      DMA::Source::USART1_RX,
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
      DMA::Source::USART2_RX,
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
      DMA::Source::USART3_RX,
#endif
#if defined( STM32_USART6_PERIPH_AVAILABLE )
      DMA::Source::USART6_RX,
#endif
    };


    LLD_CONST DMA::Source TXDMASignals[ NUM_USART_PERIPHS ] = {
#if defined( STM32_USART1_PERIPH_AVAILABLE )
      DMA::Source::USART1_TX,
#endif
#if defined( STM32_USART2_PERIPH_AVAILABLE )
      DMA::Source::USART2_TX,
#endif
#if defined( STM32_USART3_PERIPH_AVAILABLE )
      DMA::Source::USART3_TX,
#endif
#if defined( STM32_USART6_PERIPH_AVAILABLE )
      DMA::Source::USART6_TX,
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
      USART3_IRQn,
#endif
#if defined( STM32_USART6_PERIPH_AVAILABLE )
      USART6_IRQn,
#endif
    };
  } /* clang-format on */


  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static Driver s_usart_drivers[ NUM_USART_PERIPHS ];


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
      return &s_usart_drivers[ getResourceIndex( channel ) ];
    }

    return nullptr;
  }

}    // namespace Thor::LLD::USART


/*-------------------------------------------------------------------------------
IRQ Handlers
-------------------------------------------------------------------------------*/
#if defined( STM32_USART1_PERIPH_AVAILABLE )
void USART1_IRQHandler( void )
{
  using namespace Thor::LLD::USART;
  s_usart_drivers[ USART1_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART1_PERIPH_AVAILABLE */


#if defined( STM32_USART2_PERIPH_AVAILABLE )
void USART2_IRQHandler( void )
{
  using namespace Thor::LLD::USART;
  s_usart_drivers[ USART2_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART2_PERIPH_AVAILABLE */


#if defined( STM32_USART3_PERIPH_AVAILABLE )
void USART3_IRQHandler( void )
{
  using namespace Thor::LLD::USART;
  s_usart_drivers[ USART3_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART3_PERIPH_AVAILABLE */


#if defined( STM32_USART6_PERIPH_AVAILABLE )
void USART6_IRQHandler( void )
{
  using namespace Thor::LLD::USART;
  s_usart_drivers[ USART6_RESOURCE_INDEX ].IRQHandler();
}
#endif /* STM32_USART6_PERIPH_AVAILABLE */


#endif /* TARGET_STM32F4 && THOR_LLD_USART */
