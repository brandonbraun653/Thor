#include <Thor/interrupt.hpp>

/* This macro becomes a problem if a chip doesn't have some of these SPI channels.... */
#if defined( STM32F767xx )
#define IS_SPI_INSTANCE( __INSTANCE__ )                                                             \
  ( ( ( __INSTANCE__ ) == SPI1 ) || ( ( __INSTANCE__ ) == SPI2 ) || ( ( __INSTANCE__ ) == SPI3 ) || \
    ( ( __INSTANCE__ ) == SPI4 ) || ( ( __INSTANCE__ ) == SPI5 ) || ( ( __INSTANCE__ ) == SPI6 ) )
#endif

#if defined( STM32F446xx )
#define IS_SPI_INSTANCE( __INSTANCE__ )                                                             \
  ( ( ( __INSTANCE__ ) == SPI1 ) || ( ( __INSTANCE__ ) == SPI2 ) || ( ( __INSTANCE__ ) == SPI3 ) || \
    ( ( __INSTANCE__ ) == SPI4 ) )
#endif

Thor::Interrupt::DMA::DMAHandler dmaHandler;
Thor::Interrupt::Serial::Serial_DMAHandlerManager serialDMAManager;
Thor::Interrupt::SPI::SPI_DMAHandlerManager spiDMAManager;

namespace Thor
{
  namespace Interrupt
  {
    namespace DMA
    {
      void DMAHandler::updateDMASource( uint32_t periph, uint32_t stream )
      {
        DMA_Periph = periph;
        DMA_Stream = stream;
      }

      void DMAHandler::copyRegisters( uint32_t REG_LISR, uint32_t REG_HISR, uint32_t REG_SxCR, uint32_t REG_SxPAR )
      {
        REG_DMA_LISR  = REG_LISR;
        REG_DMA_HISR  = REG_HISR;
        REG_DMA_SxCR  = REG_SxCR;
        REG_DMA_SxPAR = REG_SxPAR;
      }

      bool DMAHandler::calculatePeriphSource( PeriphConfig &output )
      {
        bool source_found = true;

        return source_found;
      }

      void DMAHandler::IRQHandler()
      {
        PeriphConfig config;

        if ( calculatePeriphSource( config ) )
        {
          switch ( config.peripheral_type )
          {
            case Thor::Interrupt::SRC_UART:
            case Thor::Interrupt::SRC_USART:
              serialDMAManager.requestCallback( config );
              break;

            case Thor::Interrupt::SRC_SPI:
              spiDMAManager.requestCallback( config );

            default:
              break;
          };
        }
      }


      DMAManagerBase::DMAManagerBase( const size_t numCallbacks )
      {
        txdmaCallbacks.resize( numCallbacks );
        rxdmaCallbacks.resize( numCallbacks );
      }

      void DMAManagerBase::attachCallback_TXDMA( size_t periphNum, func_void func )
      {
        if ( periphNum < txdmaCallbacks.size() )
          txdmaCallbacks[ periphNum ] = func;
      }

      void DMAManagerBase::attachCallback_RXDMA( size_t periphNum, func_void func )
      {
        if ( periphNum < rxdmaCallbacks.size() )
          rxdmaCallbacks[ periphNum ] = func;
      }

      void DMAManagerBase::removeCallback_TXDMA( size_t periphNum )
      {
        if ( periphNum < txdmaCallbacks.size() )
          txdmaCallbacks[ periphNum ].clear();
      }

      void DMAManagerBase::removeCallback_RXDMA( size_t periphNum )
      {
        if ( periphNum < rxdmaCallbacks.size() )
          rxdmaCallbacks[ periphNum ].clear();
      }

      void DMAManagerBase::executeCallback_TXDMA( size_t periphNum )
      {
        if ( ( periphNum < txdmaCallbacks.size() ) && !txdmaCallbacks[ periphNum ].empty() )
          txdmaCallbacks[ periphNum ]();
      }

      void DMAManagerBase::executeCallback_RXDMA( size_t periphNum )
      {
        if ( ( periphNum < rxdmaCallbacks.size() ) && !rxdmaCallbacks[ periphNum ].empty() )
          rxdmaCallbacks[ periphNum ]();
      }
    };    // namespace DMA

    namespace Serial
    {
      void Serial_DMAHandlerManager::requestCallback( Thor::Interrupt::DMA::PeriphConfig pConfig )
      {
        switch ( pConfig.peripheral_instance )
        {
#if defined( USART1 )
          case Thor::Interrupt::SRC_USART1:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 1 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 1 );
            }
            break;
#endif

#if defined( USART2 )
          case Thor::Interrupt::SRC_USART2:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 2 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 2 );
            }
            break;
#endif

#if defined( USART3 )
          case Thor::Interrupt::SRC_USART3:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 3 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 3 );
            }
            break;
#endif

#if defined( UART4 )
          case Thor::Interrupt::SRC_UART4:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 4 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 4 );
            }
            break;
#endif

#if defined( UART5 )
          case Thor::Interrupt::SRC_UART5:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 5 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 5 );
            }
            break;
#endif

#if defined( USART6 )
          case Thor::Interrupt::SRC_USART6:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 6 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 6 );
            }
            break;
#endif

#if defined( UART7 )
          case Thor::Interrupt::SRC_UART7:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 7 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 7 );
            }
            break;
#endif

#if defined( UART8 )
          case Thor::Interrupt::SRC_UART8:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 8 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 8 );
            }
            break;
#endif

          default:
            break;
        }
      }
    };    // namespace Serial

    namespace SPI
    {
      void SPI_DMAHandlerManager::requestCallback( Thor::Interrupt::DMA::PeriphConfig pConfig )
      {
        switch ( pConfig.peripheral_instance )
        {
#if defined( SPI1 )
          case Thor::Interrupt::SRC_SPI1:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 1 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 1 );
            }
            break;
#endif

#if defined( SPI2 )
          case Thor::Interrupt::SRC_SPI2:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 2 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 2 );
            }
            break;
#endif

#if defined( SPI3 )
          case Thor::Interrupt::SRC_SPI3:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 3 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 3 );
            }
            break;
#endif

#if defined( SPI4 )
          case Thor::Interrupt::SRC_SPI4:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 4 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 4 );
            }
            break;
#endif

#if defined( SPI5 )
          case Thor::Interrupt::SRC_SPI5:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 5 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 5 );
            }
            break;
#endif

#if defined( SPI6 )
          case Thor::Interrupt::SRC_SPI6:
            if ( pConfig.direction == Thor::DMA::MEM_TO_PERIPH )
            {
              executeCallback_TXDMA( 6 );
            }
            else if ( pConfig.direction == Thor::DMA::PERIPH_TO_MEM )
            {
              executeCallback_RXDMA( 6 );
            }
            break;
#endif

          default:
            break;
        }
      }
    };    // namespace SPI
  };      // namespace Interrupt
}    // namespace Thor


#if defined( DMA1 )
void DMA1_Stream0_IRQHandler( void )
{
  dmaHandler.updateDMASource( 1u, 0u );
  dmaHandler.copyRegisters( DMA1_LISR, DMA1_HISR, DMA1_S0CR, DMA1_S0PAR );
  dmaHandler.IRQHandler();
}

void DMA1_Stream1_IRQHandler( void )
{
  dmaHandler.updateDMASource( 1u, 1u );
  dmaHandler.copyRegisters( DMA1_LISR, DMA1_HISR, DMA1_S1CR, DMA1_S1PAR );
  dmaHandler.IRQHandler();
}

void DMA1_Stream2_IRQHandler( void )
{
  dmaHandler.updateDMASource( 1u, 2u );
  dmaHandler.copyRegisters( DMA1_LISR, DMA1_HISR, DMA1_S2CR, DMA1_S2PAR );
  dmaHandler.IRQHandler();
}

void DMA1_Stream3_IRQHandler( void )
{
  dmaHandler.updateDMASource( 1u, 3u );
  dmaHandler.copyRegisters( DMA1_LISR, DMA1_HISR, DMA1_S3CR, DMA1_S3PAR );
  dmaHandler.IRQHandler();
}

void DMA1_Stream4_IRQHandler( void )
{
  dmaHandler.updateDMASource( 1u, 4u );
  dmaHandler.copyRegisters( DMA1_LISR, DMA1_HISR, DMA1_S4CR, DMA1_S4PAR );
  dmaHandler.IRQHandler();
}

void DMA1_Stream5_IRQHandler( void )
{
  dmaHandler.updateDMASource( 1u, 5u );
  dmaHandler.copyRegisters( DMA1_LISR, DMA1_HISR, DMA1_S5CR, DMA1_S5PAR );
  dmaHandler.IRQHandler();
}

void DMA1_Stream6_IRQHandler( void )
{
  dmaHandler.updateDMASource( 1u, 6u );
  dmaHandler.copyRegisters( DMA1_LISR, DMA1_HISR, DMA1_S6CR, DMA1_S6PAR );
  dmaHandler.IRQHandler();
}

void DMA1_Stream7_IRQHandler( void )
{
  dmaHandler.updateDMASource( 1u, 7u );
  dmaHandler.copyRegisters( DMA1_LISR, DMA1_HISR, DMA1_S7CR, DMA1_S7PAR );
  dmaHandler.IRQHandler();
}
#endif /* DMA1 */

#if defined( DMA2 )
void DMA2_Stream0_IRQHandler( void )
{
  dmaHandler.updateDMASource( 2u, 0u );
  dmaHandler.copyRegisters( DMA2_LISR, DMA2_HISR, DMA2_S0CR, DMA2_S0PAR );
  dmaHandler.IRQHandler();
}

void DMA2_Stream1_IRQHandler( void )
{
  dmaHandler.updateDMASource( 2u, 1u );
  dmaHandler.copyRegisters( DMA2_LISR, DMA2_HISR, DMA2_S1CR, DMA2_S1PAR );
  dmaHandler.IRQHandler();
}

void DMA2_Stream2_IRQHandler( void )
{
  dmaHandler.updateDMASource( 2u, 2u );
  dmaHandler.copyRegisters( DMA2_LISR, DMA2_HISR, DMA2_S2CR, DMA2_S2PAR );
  dmaHandler.IRQHandler();
}

void DMA2_Stream3_IRQHandler( void )
{
  dmaHandler.updateDMASource( 2u, 3u );
  dmaHandler.copyRegisters( DMA2_LISR, DMA2_HISR, DMA2_S3CR, DMA2_S3PAR );
  dmaHandler.IRQHandler();
}

void DMA2_Stream4_IRQHandler( void )
{
  dmaHandler.updateDMASource( 2u, 4u );
  dmaHandler.copyRegisters( DMA2_LISR, DMA2_HISR, DMA2_S4CR, DMA2_S4PAR );
  dmaHandler.IRQHandler();
}

void DMA2_Stream5_IRQHandler( void )
{
  dmaHandler.updateDMASource( 2u, 5u );
  dmaHandler.copyRegisters( DMA2_LISR, DMA2_HISR, DMA2_S5CR, DMA2_S5PAR );
  dmaHandler.IRQHandler();
}

void DMA2_Stream6_IRQHandler( void )
{
  dmaHandler.updateDMASource( 2u, 6u );
  dmaHandler.copyRegisters( DMA2_LISR, DMA2_HISR, DMA2_S6CR, DMA2_S6PAR );
  dmaHandler.IRQHandler();
}

void DMA2_Stream7_IRQHandler( void )
{
  dmaHandler.updateDMASource( 2u, 7u );
  dmaHandler.copyRegisters( DMA2_LISR, DMA2_HISR, DMA2_S7CR, DMA2_S7PAR );
  dmaHandler.IRQHandler();
}
#endif /* DMA2 */
