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
#if defined( SEGGER_SYS_VIEW ) && defined( EMBEDDED )
#include "SEGGER_SYSVIEW.h"
#endif
#include <Thor/lld/interface/inc/dma>

using namespace Thor::LLD::DMA;

#ifdef __cplusplus
extern "C"
{
#endif

  void DMA1_Stream0_IRQHandler( void )
  {
    SEGGER_SYSVIEW_RecordEnterISR();
    const uint8_t channel = CSELR_ALL::get( DMA1_PERIPH ) >> CSELR_C1S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA1_PERIPH ) >> ISR_GIF1_Pos ) & 0xFF;
    getStream( Controller::DMA_1, Streamer::STREAM_1 )->IRQHandler( channel, status );
    SEGGER_SYSVIEW_RecordExitISR();
  }

  void DMA1_Stream1_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA1_PERIPH ) >> CSELR_C2S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA1_PERIPH ) >> ISR_GIF2_Pos ) & 0xFF;
    getStream( Controller::DMA_1, Streamer::STREAM_2 )->IRQHandler( channel, status );
  }

  void DMA1_Stream2_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA1_PERIPH ) >> CSELR_C3S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA1_PERIPH ) >> ISR_GIF3_Pos ) & 0xFF;
    getStream( Controller::DMA_1, Streamer::STREAM_3 )->IRQHandler( channel, status );
  }

  void DMA1_Stream3_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA1_PERIPH ) >> CSELR_C4S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA1_PERIPH ) >> ISR_GIF4_Pos ) & 0xFF;
    getStream( Controller::DMA_1, Streamer::STREAM_4 )->IRQHandler( channel, status );
  }

  void DMA1_Stream4_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA1_PERIPH ) >> CSELR_C5S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA1_PERIPH ) >> ISR_GIF5_Pos ) & 0xFF;
    getStream( Controller::DMA_1, Streamer::STREAM_5 )->IRQHandler( channel, status );
  }

  void DMA1_Stream5_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA1_PERIPH ) >> CSELR_C6S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA1_PERIPH ) >> ISR_GIF6_Pos ) & 0xFF;
    getStream( Controller::DMA_1, Streamer::STREAM_6 )->IRQHandler( channel, status );
  }

  void DMA1_Stream6_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA1_PERIPH ) >> CSELR_C7S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA1_PERIPH ) >> ISR_GIF7_Pos ) & 0xFF;
    getStream( Controller::DMA_1, Streamer::STREAM_7 )->IRQHandler( channel, status );
  }



  void DMA2_Stream0_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA2_PERIPH ) >> CSELR_C1S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA2_PERIPH ) >> ISR_GIF1_Pos ) & 0xFF;
    getStream( Controller::DMA_2, Streamer::STREAM_1 )->IRQHandler( channel, status );
  }

  void DMA2_Stream1_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA2_PERIPH ) >> CSELR_C2S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA2_PERIPH ) >> ISR_GIF2_Pos ) & 0xFF;
    getStream( Controller::DMA_2, Streamer::STREAM_2 )->IRQHandler( channel, status );
  }

  void DMA2_Stream2_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA2_PERIPH ) >> CSELR_C3S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA2_PERIPH ) >> ISR_GIF3_Pos ) & 0xFF;
    getStream( Controller::DMA_2, Streamer::STREAM_3 )->IRQHandler( channel, status );
  }

  void DMA2_Stream3_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA2_PERIPH ) >> CSELR_C4S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA2_PERIPH ) >> ISR_GIF4_Pos ) & 0xFF;
    getStream( Controller::DMA_2, Streamer::STREAM_4 )->IRQHandler( channel, status );
  }

  void DMA2_Stream4_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA2_PERIPH ) >> CSELR_C5S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA2_PERIPH ) >> ISR_GIF5_Pos ) & 0xFF;
    getStream( Controller::DMA_2, Streamer::STREAM_5 )->IRQHandler( channel, status );
  }

  void DMA2_Stream5_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA2_PERIPH ) >> CSELR_C6S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA2_PERIPH ) >> ISR_GIF6_Pos ) & 0xFF;
    getStream( Controller::DMA_2, Streamer::STREAM_6 )->IRQHandler( channel, status );
  }

  void DMA2_Stream6_IRQHandler( void )
  {
    const uint8_t channel = CSELR_ALL::get( DMA2_PERIPH ) >> CSELR_C7S_Pos;
    const uint8_t status = ( ISR_ALL::get( DMA2_PERIPH ) >> ISR_GIF7_Pos ) & 0xFF;
    getStream( Controller::DMA_2, Streamer::STREAM_7 )->IRQHandler( channel, status );
  }

#ifdef __cplusplus
} /* extern "C" */
#endif
