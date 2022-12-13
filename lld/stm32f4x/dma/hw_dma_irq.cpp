/******************************************************************************
 *  File Name:
 *    hw_dma_irq.cpp
 *
 *  Description:
 *    DMA IRQ Functions
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/inc/dma>

using namespace Thor::LLD::DMA;

#ifdef __cplusplus
extern "C"
{
#endif

  void DMA1_Stream0_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA1_STREAM0 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( LISR_ALL::get( DMA1_PERIPH ) >> LISR_FEIF0_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_1, Streamer::STREAM_0 )->IRQHandler( channel, status );
  }

  void DMA1_Stream1_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA1_STREAM1 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( LISR_ALL::get( DMA1_PERIPH ) >> LISR_FEIF1_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_1, Streamer::STREAM_1 )->IRQHandler( channel, status );
  }

  void DMA1_Stream2_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA1_STREAM2 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( LISR_ALL::get( DMA1_PERIPH ) >> LISR_FEIF2_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_1, Streamer::STREAM_2 )->IRQHandler( channel, status );
  }

  void DMA1_Stream3_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA1_STREAM3 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( LISR_ALL::get( DMA1_PERIPH ) >> LISR_FEIF3_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_1, Streamer::STREAM_3 )->IRQHandler( channel, status );
  }

  void DMA1_Stream4_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA1_STREAM4 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( HISR_ALL::get( DMA1_PERIPH ) >> HISR_FEIF4_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_1, Streamer::STREAM_4 )->IRQHandler( channel, status );
  }

  void DMA1_Stream5_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA1_STREAM5 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( HISR_ALL::get( DMA1_PERIPH ) >> HISR_FEIF5_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_1, Streamer::STREAM_5 )->IRQHandler( channel, status );
  }

  void DMA1_Stream6_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA1_STREAM6 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( HISR_ALL::get( DMA1_PERIPH ) >> HISR_FEIF6_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_1, Streamer::STREAM_6 )->IRQHandler( channel, status );
  }

  void DMA1_Stream7_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA1_STREAM7 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( HISR_ALL::get( DMA1_PERIPH ) >> HISR_FEIF7_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_1, Streamer::STREAM_7 )->IRQHandler( channel, status );
  }

  void DMA2_Stream0_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA2_STREAM0 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( LISR_ALL::get( DMA2_PERIPH ) >> LISR_FEIF0_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_2, Streamer::STREAM_0 )->IRQHandler( channel, status );
  }

  void DMA2_Stream1_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA2_STREAM1 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( LISR_ALL::get( DMA2_PERIPH ) >> LISR_FEIF1_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_2, Streamer::STREAM_1 )->IRQHandler( channel, status );
  }

  void DMA2_Stream2_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA2_STREAM2 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( LISR_ALL::get( DMA2_PERIPH ) >> LISR_FEIF2_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_2, Streamer::STREAM_2 )->IRQHandler( channel, status );
  }

  void DMA2_Stream3_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA2_STREAM3 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( LISR_ALL::get( DMA2_PERIPH ) >> LISR_FEIF3_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_2, Streamer::STREAM_3 )->IRQHandler( channel, status );
  }

  void DMA2_Stream4_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA2_STREAM4 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( HISR_ALL::get( DMA2_PERIPH ) >> HISR_FEIF4_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_2, Streamer::STREAM_4 )->IRQHandler( channel, status );
  }

  void DMA2_Stream5_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA2_STREAM5 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( HISR_ALL::get( DMA2_PERIPH ) >> HISR_FEIF5_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_2, Streamer::STREAM_5 )->IRQHandler( channel, status );
  }

  void DMA2_Stream6_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA2_STREAM6 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( HISR_ALL::get( DMA2_PERIPH ) >> HISR_FEIF6_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_2, Streamer::STREAM_6 )->IRQHandler( channel, status );
  }

  void DMA2_Stream7_IRQHandler( void )
  {
    const uint8_t channel = CHSEL::get( DMA2_STREAM7 ) >> SxCR_CHSEL_Pos;
    const uint8_t status = ( HISR_ALL::get( DMA2_PERIPH ) >> HISR_FEIF7_Pos ) & SR_Field_Msk;
    getStream( Controller::DMA_2, Streamer::STREAM_7 )->IRQHandler( channel, status );
  }

#ifdef __cplusplus
} /* extern "C" */
#endif
