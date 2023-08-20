/******************************************************************************
 *  File Name:
 *    hw_dma_stream.cpp
 *
 *  Description:
 *    DMA Stream Implementation
 *
 *  2021-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/interrupt>
#include <Chimera/thread>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>

namespace Thor::LLD::DMA
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  void streamEnable( StreamMap *stream )
  {
    EN::set( stream, SxCR_EN );
  }


  void streamDisable( StreamMap *stream )
  {
    EN::clear( stream, SxCR_EN );
  }


  void streamClearInterruptEnableFlags( StreamMap *stream )
  {
    stream->CR &= ~( SxCR_TCIE | SxCR_HTIE | SxCR_TEIE | SxCR_DMEIE );
    FEIE::clear( stream, SxFCR_FEIE );
  }

  /*---------------------------------------------------------------------------
  Stream Class
  ---------------------------------------------------------------------------*/
  Chimera::Status_t Stream::configure( StreamConfig *const config, TCB *const cb )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !config || !cb )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Try to apply the configuration to the DMA stream
    -------------------------------------------------------------------------*/
    disableInterrupts();
    {
      /*-----------------------------------------------------------------------
      Check to see if a transfer is currently going
      -----------------------------------------------------------------------*/
      if ( mStreamTCB.state != StreamState::TRANSFER_IDLE )
      {
        enableInterrupts();
        return Chimera::Status::BUSY;
      }

      /*-----------------------------------------------------------------------
      Pull in the user's TCB and reset it's state
      -----------------------------------------------------------------------*/
      memcpy( &mStreamTCB, cb, sizeof( TCB ) );

      mStreamTCB.state               = StreamState::TRANSFER_IDLE;
      mStreamTCB.elementsTransferred = 0;
      mStreamTCB.resourceIndex       = mStreamResourceIndex;

      /*-----------------------------------------------------------------------
      Disable the stream and clear out the LISR/HISR registers to reset back to
      a configurable state.
      -----------------------------------------------------------------------*/
      EN::clear( mStream, SxCR_EN );
      while ( EN::get( mStream ) )
      {
        continue;
      }
      reset_isr_flags();

      /*-----------------------------------------------------------------------
      Configure direction dependent settings
      -----------------------------------------------------------------------*/
      if ( ( config->direction == Chimera::DMA::Direction::PERIPH_TO_MEMORY ) ||
           ( config->direction == Chimera::DMA::Direction::MEMORY_TO_MEMORY ) )
      {
        PA::set( mStream, cb->srcAddress );
        configure_periph_settings( config->srcAddrIncr, config->srcBurstSize, config->srcAddrAlign );

        M0A::set( mStream, cb->dstAddress );
        configure_memory_settings( config->dstAddrIncr, config->dstBurstSize, config->dstAddrAlign );
      }
      else if ( config->direction == Chimera::DMA::Direction::MEMORY_TO_PERIPH )
      {
        M0A::set( mStream, cb->srcAddress );
        configure_memory_settings( config->srcAddrIncr, config->srcBurstSize, config->srcAddrAlign );

        PA::set( mStream, cb->dstAddress );
        configure_periph_settings( config->dstAddrIncr, config->dstBurstSize, config->dstAddrAlign );
      }
      else
      {
        return Chimera::Status::FAIL;
      }

      /*-----------------------------------------------------------------------
      Set how many bytes are to be transferred (RM0390 Section 9.3.16)
      -----------------------------------------------------------------------*/
      RT_HARD_ASSERT( cb->transferSize <= 65535 );
      NDT::set( mStream, cb->transferSize );

      /*-----------------------------------------------------------------------
      Select the DMA channel request
      -----------------------------------------------------------------------*/
      CHSEL::set( mStream, 0 );
      if ( config->channel < Channel::NUM_OPTIONS )
      {
        CHSEL::set( mStream, EnumValue( config->channel ) << SxCR_CHSEL_Pos );
      }

      /*-----------------------------------------------------------------------
      Set the DMA mode. This includes flow control and
      the memory access style.
      -----------------------------------------------------------------------*/
      PFCTRL::clear( mStream, SxCR_PFCTRL );
      CIRC::clear( mStream, SxCR_CIRC );
      DBM::clear( mStream, SxCR_DBM );

      if ( config->dmaMode == Chimera::DMA::Mode::PERIPHERAL )
      {
        PFCTRL::set( mStream, SxCR_PFCTRL );
      }
      else if ( config->dmaMode == Chimera::DMA::Mode::CIRCULAR )
      {
        /* Circular mode is mutually exclusive from peripheral mode */
        CIRC::set( mStream, SxCR_CIRC );
      }
      else if ( config->dmaMode == Chimera::DMA::Mode::DOUBLE_BUFFER )
      {
        /* Double buffer mode currently not supported */
        RT_HARD_ASSERT( false );
      }
      // else normal DMA peripheral is the flow controller

      /*-----------------------------------------------------------------------
      Set the transfer priority
      -----------------------------------------------------------------------*/
      PL::set( mStream, EnumValue( config->priority ) << SxCR_PL_Pos );

      static_assert( EnumValue( Chimera::DMA::Priority::LOW ) == 0 );
      static_assert( EnumValue( Chimera::DMA::Priority::MEDIUM ) == 1 );
      static_assert( EnumValue( Chimera::DMA::Priority::HIGH ) == 2 );
      static_assert( EnumValue( Chimera::DMA::Priority::VERY_HIGH ) == 3 );

      /*-----------------------------------------------------------------------
      Set the FIFO usage
      -----------------------------------------------------------------------*/
      /* Direct Mode */
      DMDIS::set( mStream, SxFCR_DMDIS );
      if ( config->fifoMode == Chimera::DMA::FifoMode::DIRECT_ENABLE )
      {
        DMDIS::clear( mStream, SxFCR_DMDIS );
      }

      /* Threshold */
      FTH::set( mStream, EnumValue( config->fifoThreshold ) << SxFCR_FTH_Pos );

      /*-----------------------------------------------------------------------
      Data transfer direction
      -----------------------------------------------------------------------*/
      switch ( config->direction )
      {
        case Chimera::DMA::Direction::PERIPH_TO_MEMORY:
          DIR::set( mStream, 0x00 << SxCR_DIR_Pos );
          break;

        case Chimera::DMA::Direction::MEMORY_TO_PERIPH:
          DIR::set( mStream, 0x01 << SxCR_DIR_Pos );
          break;

        case Chimera::DMA::Direction::MEMORY_TO_MEMORY:
          DIR::set( mStream, 0x02 << SxCR_DIR_Pos );
          break;

        default:
          RT_HARD_ASSERT( false );
          break;
      }

      /*-----------------------------------------------------------------------
      Interrupt Settings: By default, enable everything
      -----------------------------------------------------------------------*/
      mStream->CR |= ( SxCR_TCIE | SxCR_TEIE | SxCR_DMEIE );
      FEIE::set( mStream, SxFCR_FEIE ); /* Fifo error */

      /*-----------------------------------------------------------------------
      Configure the global interrupt priority
      -----------------------------------------------------------------------*/
      INT::setPriority( mStreamIRQn, INT::DMA_STREAM_PREEMPT_PRIORITY, 0u );
      INT::enableIRQ( mStreamIRQn );

      mStreamTCB.state = StreamState::TRANSFER_CONFIGURED;
    }
    enableInterrupts();

    return Chimera::Status::OK;
  }


  void Stream::IRQHandler( const uint8_t channel, const uint8_t status )
  {
    /*-------------------------------------------------------------------------
    Local Namespaces
    -------------------------------------------------------------------------*/
    using namespace Chimera::Peripheral;
    using namespace Chimera::Thread;
    using namespace Chimera::DMA;

    /*-------------------------------------------------------------------------
    Local Constants
    -------------------------------------------------------------------------*/
    static constexpr uint8_t TCIF  = 1u << 5;
    static constexpr uint8_t HTCIF = 1u << 4;
    static constexpr uint8_t TEIF  = 1u << 3;
    static constexpr uint8_t DMEIF = 1u << 2;
    static constexpr uint8_t FEIF  = 1u << 0;

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    bool disableIsr     = false;
    bool wakeUserThread = false;

    /*-------------------------------------------------------------------------
    Transfer Errors: Allow these to be ignored due to some quirks with the HW.
    USARTs will throw FIFO errors even though it doesn't use the FIFO.
    -------------------------------------------------------------------------*/
    mStreamTCB.fifoError       = ( status & FEIF ) && ( ( mStreamTCB.errorsToIgnore & Errors::FIFO ) != Errors::FIFO );
    mStreamTCB.transferError   = ( status & TEIF ) && ( ( mStreamTCB.errorsToIgnore & Errors::TRANSFER ) != Errors::TRANSFER );
    mStreamTCB.directModeError = ( status & DMEIF ) && ( ( mStreamTCB.errorsToIgnore & Errors::DIRECT ) != Errors::DIRECT );

    if ( mStreamTCB.fifoError || mStreamTCB.transferError || mStreamTCB.directModeError )
    {
      disableIsr               = true;
      wakeUserThread           = mStreamTCB.wakeUserOnComplete;
      mStreamTCB.state         = StreamState::TRANSFER_IDLE;
      mStreamTCB.transferError = true;
    }

    /*-------------------------------------------------------------------------
    Transfer Complete?
    -------------------------------------------------------------------------*/
    if ( ( status & TCIF ) && TCIE::get( mStream ) )
    {
      mStreamTCB.selectedChannel     = channel;
      mStreamTCB.state               = StreamState::TRANSFER_IDLE;
      mStreamTCB.transferError       = false;

      /*-----------------------------------------------------------------------
      Calculate the number of elements transferred. This changes behavior based
      upon who acts as the flow controller.
      See RM0390 Section 9.2 and 9.3.16
      -----------------------------------------------------------------------*/
      if( PFCTRL::get( mStream ) )  /* Peripheral is the flow controller */
      {
        mStreamTCB.elementsTransferred = 0xFFFFu - NDT::get( mStream );
      }
      else  /* DMA is the flow controller */
      {
        mStreamTCB.elementsTransferred = mStreamTCB.transferSize - NDT::get( mStream );

        /*---------------------------------------------------------------------
        When using circular transfer mode, the number of elements transferred
        is automatically reloaded. We can trust we fully transferred the data
        though since we've got the transfer complete interrupt.
        ---------------------------------------------------------------------*/
        if ( CIRC::get( mStream ) )
        {
          mStreamTCB.elementsTransferred = mStreamTCB.transferSize;
        }
      }

      /*-----------------------------------------------------------------------
      Only disable DMA if requested
      -----------------------------------------------------------------------*/
      if ( !mStreamTCB.persistent )
      {
        disableIsr     = true;
        wakeUserThread = mStreamTCB.wakeUserOnComplete;
        streamDisable( mStream );
      }

      /*-----------------------------------------------------------------------
      ACK the transfer complete flag
      -----------------------------------------------------------------------*/
      reset_isr_flags();

      /*-----------------------------------------------------------------------
      Invoke the ISR callback should one exist
      -----------------------------------------------------------------------*/
      if ( mStreamTCB.isrCallback )
      {
        Chimera::DMA::TransferStats stats;
        stats.error     = false;
        stats.requestId = mStreamTCB.requestId;
        stats.size      = mStreamTCB.elementsTransferred;

        mStreamTCB.isrCallback( stats );
      }
    }

    /*-------------------------------------------------------------------------
    Transfer Half-Complete
    -------------------------------------------------------------------------*/
    if ( ( status & HTCIF ) && HTIE::get( mStream ) )
    {
      mStream->CR &= ~SxCR_HTIE;
    }

    /*-------------------------------------------------------------------------
    Disable the ISR signals on exit if needed
    -------------------------------------------------------------------------*/
    if ( disableIsr )
    {
      streamClearInterruptEnableFlags( mStream );
      reset_isr_flags();
    }

    /*-------------------------------------------------------------------------
    Wake up the user thread to handle the generated events
    -------------------------------------------------------------------------*/
    if ( wakeUserThread )
    {
      Resource::ISRQueue.push( mStreamTCB );
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_DMA ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  void Stream::reset_isr_flags()
  {
    switch ( mStreamPhysicalId )
    {
      case Streamer::STREAM_0:
        mPeriph->LIFCR = LIFCR_CTCIF0 | LIFCR_CHTIF0 | LIFCR_CTEIF0 | LIFCR_CDMEIF0 | LIFCR_CFEIF0;
        break;

      case Streamer::STREAM_1:
        mPeriph->LIFCR = LIFCR_CTCIF1 | LIFCR_CHTIF1 | LIFCR_CTEIF1 | LIFCR_CDMEIF1 | LIFCR_CFEIF1;
        break;

      case Streamer::STREAM_2:
        mPeriph->LIFCR = LIFCR_CTCIF2 | LIFCR_CHTIF2 | LIFCR_CTEIF2 | LIFCR_CDMEIF2 | LIFCR_CFEIF2;
        break;

      case Streamer::STREAM_3:
        mPeriph->LIFCR = LIFCR_CTCIF3 | LIFCR_CHTIF3 | LIFCR_CTEIF3 | LIFCR_CDMEIF3 | LIFCR_CFEIF3;
        break;

      case Streamer::STREAM_4:
        mPeriph->HIFCR = HIFCR_CTCIF4 | HIFCR_CHTIF4 | HIFCR_CTEIF4 | HIFCR_CDMEIF4 | HIFCR_CFEIF4;
        break;

      case Streamer::STREAM_5:
        mPeriph->HIFCR = HIFCR_CTCIF5 | HIFCR_CHTIF5 | HIFCR_CTEIF5 | HIFCR_CDMEIF5 | HIFCR_CFEIF5;
        break;

      case Streamer::STREAM_6:
        mPeriph->HIFCR = HIFCR_CTCIF6 | HIFCR_CHTIF6 | HIFCR_CTEIF6 | HIFCR_CDMEIF6 | HIFCR_CFEIF6;
        break;

      case Streamer::STREAM_7:
        mPeriph->HIFCR = HIFCR_CTCIF7 | HIFCR_CHTIF7 | HIFCR_CTEIF7 | HIFCR_CDMEIF7 | HIFCR_CFEIF7;
        break;

      default:
        break;
    }
  }


  void Stream::configure_memory_settings( const bool incr, const Chimera::DMA::BurstSize bSize,
                                          const Chimera::DMA::Alignment align )
  {
    /*-------------------------------------------------------------------------
    Memory Auto-Increment
    -------------------------------------------------------------------------*/
    MINC::clear( mStream, SxCR_MINC );
    if ( incr )
    {
      MINC::set( mStream, SxCR_MINC );
    }

    /*-------------------------------------------------------------------------
    Memory Burst Size
    -------------------------------------------------------------------------*/
    switch ( bSize )
    {
      case Chimera::DMA::BurstSize::BURST_SIZE_1:
        MBURST::set( mStream, 0x00 );
        break;

      case Chimera::DMA::BurstSize::BURST_SIZE_4:
        MBURST::set( mStream, 0x1 << SxCR_MBURST_Pos );
        break;

      case Chimera::DMA::BurstSize::BURST_SIZE_8:
        MBURST::set( mStream, 0x2 << SxCR_MBURST_Pos );
        break;

      case Chimera::DMA::BurstSize::BURST_SIZE_16:
        MBURST::set( mStream, 0x3 << SxCR_MBURST_Pos );
        break;

      default:
        // Not necessarily needs configuration
        break;
    }

    /*-------------------------------------------------------------------------
    Memory Alignment
    -------------------------------------------------------------------------*/
    switch ( align )
    {
      case Chimera::DMA::Alignment::BYTE:
        MSIZE::set( mStream, 0x00 << SxCR_MSIZE_Pos );
        break;

      case Chimera::DMA::Alignment::HALF_WORD:
        MSIZE::set( mStream, 0x01 << SxCR_MSIZE_Pos );
        break;

      case Chimera::DMA::Alignment::WORD:
        MSIZE::set( mStream, 0x02 << SxCR_MSIZE_Pos );
        break;

      default:
        // Not necessarily needs configuration
        break;
    }
  }


  void Stream::configure_periph_settings( const bool incr, const Chimera::DMA::BurstSize bSize,
                                          const Chimera::DMA::Alignment align )
  {
    /*-------------------------------------------------------------------------
    Peripheral Auto-Increment Offset Size
    -------------------------------------------------------------------------*/
    PINCOS::clear( mStream, SxCR_PINCOS );
    PINC::clear( mStream, SxCR_PINC );
    if ( incr )
    {
      PINC::set( mStream, SxCR_PINC );
    }

    /*-------------------------------------------------------------------------
    Peripheral Burst Size
    -------------------------------------------------------------------------*/
    switch ( bSize )
    {
      case Chimera::DMA::BurstSize::BURST_SIZE_1:
        PBURST::set( mStream, 0x00 );
        break;

      case Chimera::DMA::BurstSize::BURST_SIZE_4:
        PBURST::set( mStream, 0x1 << SxCR_PBURST_Pos );
        break;

      case Chimera::DMA::BurstSize::BURST_SIZE_8:
        PBURST::set( mStream, 0x2 << SxCR_PBURST_Pos );
        break;

      case Chimera::DMA::BurstSize::BURST_SIZE_16:
        PBURST::set( mStream, 0x3 << SxCR_PBURST_Pos );
        break;

      default:
        // Not necessarily needs configuration
        break;
    }

    /*-------------------------------------------------------------------------
    Peripheral Alignment
    -------------------------------------------------------------------------*/
    switch ( align )
    {
      case Chimera::DMA::Alignment::BYTE:
        PSIZE::set( mStream, 0x00 << SxCR_PSIZE_Pos );
        break;

      case Chimera::DMA::Alignment::HALF_WORD:
        PSIZE::set( mStream, 0x01 << SxCR_PSIZE_Pos );
        break;

      case Chimera::DMA::Alignment::WORD:
        PSIZE::set( mStream, 0x02 << SxCR_PSIZE_Pos );
        break;

      default:
        // Not necessarily needs configuration
        break;
    }
  }
}    // namespace Thor::LLD::DMA
