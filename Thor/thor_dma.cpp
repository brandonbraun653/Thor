/********************************************************************************
* File Name:
*   thor_dma.cpp
*
* Description:
*   Implements the Thor DMA driver
*
* 2019 | Brandon Braun | brandonbraun653@gmail.com
********************************************************************************/

/* Thor Includes */
#include <Thor/thor.hpp>
#include <Thor/dma.hpp>

namespace Thor
{
  namespace DMA
  {
    std::array<boost::function<void(void)>, Source::S_NUM_DMA_REQUESTORS> requestHandlers;
  }    // namespace DMA
}    // namespace Thor

using namespace Thor::DMA;

#if defined( DMA1 )
void DMA1_Stream0_IRQHandler( void )
{
  constexpr uint8_t periph = 1u;
  constexpr uint8_t stream = 0u;
  uint8_t channel = ((DMA1_S0CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma1RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA1_Stream1_IRQHandler( void )
{
  constexpr uint8_t periph = 1u;
  constexpr uint8_t stream = 1u;
  uint8_t channel = ((DMA1_S1CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma1RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA1_Stream2_IRQHandler( void )
{
  constexpr uint8_t periph = 1u;
  constexpr uint8_t stream = 2u;
  uint8_t channel = ((DMA1_S2CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma1RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA1_Stream3_IRQHandler( void )
{
  constexpr uint8_t periph = 1u;
  constexpr uint8_t stream = 3u;
  uint8_t channel = ((DMA1_S3CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma1RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA1_Stream4_IRQHandler( void )
{
  constexpr uint8_t periph = 1u;
  constexpr uint8_t stream = 4u;
  uint8_t channel = ((DMA1_S4CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma1RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA1_Stream5_IRQHandler( void )
{
  constexpr uint8_t periph = 1u;
  constexpr uint8_t stream = 5u;
  uint8_t channel = ((DMA1_S5CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma1RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA1_Stream6_IRQHandler( void )
{
  constexpr uint8_t periph = 1u;
  constexpr uint8_t stream = 6u;
  uint8_t channel = ((DMA1_S6CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma1RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA1_Stream7_IRQHandler( void )
{
  constexpr uint8_t periph = 1u;
  constexpr uint8_t stream = 7u;
  uint8_t channel = ((DMA1_S7CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma1RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}
#endif /* DMA1 */

#if defined( DMA2 )
void DMA2_Stream0_IRQHandler( void )
{
  constexpr uint8_t periph = 2u;
  constexpr uint8_t stream = 0u;
  uint8_t channel = ((DMA2_S0CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma2RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA2_Stream1_IRQHandler( void )
{
  constexpr uint8_t periph = 2u;
  constexpr uint8_t stream = 1u;
  uint8_t channel = ((DMA2_S1CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma2RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA2_Stream2_IRQHandler( void )
{
  constexpr uint8_t periph = 2u;
  constexpr uint8_t stream = 2u;
  uint8_t channel = ((DMA2_S2CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma2RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA2_Stream3_IRQHandler( void )
{
  constexpr uint8_t periph = 2u;
  constexpr uint8_t stream = 3u;
  uint8_t channel = ((DMA2_S3CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma2RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA2_Stream4_IRQHandler( void )
{
  constexpr uint8_t periph = 2u;
  constexpr uint8_t stream = 4u;
  uint8_t channel = ((DMA2_S4CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma2RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA2_Stream5_IRQHandler( void )
{
  constexpr uint8_t periph = 2u;
  constexpr uint8_t stream = 5u;
  uint8_t channel = ((DMA2_S5CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma2RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA2_Stream6_IRQHandler( void )
{
  constexpr uint8_t periph = 2u;
  constexpr uint8_t stream = 6u;
  uint8_t channel = ((DMA2_S6CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma2RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}

void DMA2_Stream7_IRQHandler( void )
{
  constexpr uint8_t periph = 2u;
  constexpr uint8_t stream = 7u;
  uint8_t channel = ((DMA2_S7CR & DMAx_SxCR_CHSEL) >> DMAx_SxCR_CHSEL_POS) & DMAx_SxCR_CHSEL_MSK;

  const uint8_t req = Source::dma2RequestMapping[channel][stream];
  if ( ( req < requestHandlers.size() ) && requestHandlers[req])
  {
    requestHandlers[req]();
  }
}
#endif /* DMA2 */
