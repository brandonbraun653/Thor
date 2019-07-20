/********************************************************************************
 *   File Name:
 *    hw_dma_mapping.hpp
 *
 *   Description:
 *    STM32 Mappings for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/resources/dma_resources.hpp>


/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/dma/hw_dma_driver.hpp>
#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>
#include <Thor/drivers/f4/dma/hw_dma_prj.hpp>
#include <Thor/drivers/f4/dma/hw_dma_types.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

namespace Thor::Driver::DMA
{
}



using namespace Thor::DMA;
using namespace Thor::Driver::DMA;

void DMA1_Stream0_IRQHandler( void )
{
  constexpr uint8_t stream = 0u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA1_Stream1_IRQHandler( void )
{
  constexpr uint8_t stream = 1u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA1_Stream2_IRQHandler( void )
{
  constexpr uint8_t stream = 2u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA1_Stream3_IRQHandler( void )
{
  constexpr uint8_t stream = 3u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA1_Stream4_IRQHandler( void )
{
  constexpr uint8_t stream = 4u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA1_Stream5_IRQHandler( void )
{
  constexpr uint8_t stream = 5u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA1_Stream6_IRQHandler( void )
{
  constexpr uint8_t stream = 6u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA1_Stream7_IRQHandler( void )
{
  constexpr uint8_t stream = 7u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}


void DMA2_Stream0_IRQHandler( void )
{
  constexpr uint8_t stream = 0u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA2_Stream1_IRQHandler( void )
{
  constexpr uint8_t stream = 1u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA2_Stream2_IRQHandler( void )
{
  constexpr uint8_t stream = 2u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA2_Stream3_IRQHandler( void )
{
  constexpr uint8_t stream = 3u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA2_Stream4_IRQHandler( void )
{
  constexpr uint8_t stream = 4u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA2_Stream5_IRQHandler( void )
{
  constexpr uint8_t stream = 5u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA2_Stream6_IRQHandler( void )
{
  constexpr uint8_t stream = 6u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

void DMA2_Stream7_IRQHandler( void )
{
  constexpr uint8_t stream = 7u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_PERIPH, stream ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( ( request < requestHandlers.size() ) && requestHandlers[ request ] )
  {
    requestHandlers[ request ]();
  }
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */