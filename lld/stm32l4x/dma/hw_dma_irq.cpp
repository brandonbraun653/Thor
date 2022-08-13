/********************************************************************************
 *  File Name:
 *    hw_dma_irq.cpp
 *
 *  Description:
 *    DMA IRQ Functions
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/common/macros.hpp>

using namespace Thor::LLD::DMA;

/*-----------------------------------------------------------------------------
Macros
-----------------------------------------------------------------------------*/
/**
 * @brief Shared ISR handler code for DMA streams
 *
 * @param P   DMA peripheral number (1-2)
 * @param C   DMA channel number (1-7)
 */
#define CORE_ISR_HANDLER( P, C )                                                                     \
  static const Stream_rPtr stream  = getStream( Controller::DMA_##P, Streamer::STREAM_##C );         \
  const uint8_t            channel = CSELR_ALL::get( DMA##P##_PERIPH ) >> CSELR_C##C##S_Pos;         \
  const uint8_t            status  = ( ISR_ALL::get( DMA##P##_PERIPH ) >> ISR_GIF##C##_Pos ) & 0xFF; \
                                                                                                     \
  stream->IRQHandler( channel, status );

/*-----------------------------------------------------------------------------
Public ISR Functions
-----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif
  void DMA1_Stream0_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 1, 1 );
  }

  void DMA1_Stream1_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 1, 2 );
  }

  void DMA1_Stream2_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 1, 3 );
  }

  void DMA1_Stream3_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 1, 4 );
  }

  void DMA1_Stream4_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 1, 5 );
  }

  void DMA1_Stream5_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 1, 6 );
  }

  void DMA1_Stream6_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 1, 7 );
  }


  void DMA2_Stream0_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 2, 1 );
  }

  void DMA2_Stream1_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 2, 2 );
  }

  void DMA2_Stream2_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 2, 3 );
  }

  void DMA2_Stream3_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 2, 4 );
  }

  void DMA2_Stream4_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 2, 5 );
  }

  void DMA2_Stream5_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 2, 6 );
  }

  void DMA2_Stream6_IRQHandler( void )
  {
    CORE_ISR_HANDLER( 2, 7 );
  }

#ifdef __cplusplus
} /* extern "C" */
#endif
