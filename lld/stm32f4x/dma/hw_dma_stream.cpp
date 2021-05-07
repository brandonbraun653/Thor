/********************************************************************************
 *  File Name:
 *    hw_dma_stream.cpp
 *
 *  Description:
 *    DMA Stream Implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/assert>
#include <Chimera/common>
#include <Chimera/interrupt>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>
#include <Thor/lld/interface/inc/rcc>


namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Stream Class
  -------------------------------------------------------------------------------*/
  Stream::Stream()
  {
  }


  Stream::~Stream()
  {
  }


  Chimera::Status_t Stream::attach( StreamMap *const peripheral, RegisterMap *const parent )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if( !peripheral || !parent )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Register the peripheral
    -------------------------------------------------*/
    mStream = peripheral;
    mPeriph = parent;

    return Chimera::Status::OK;
  }


  Chimera::Status_t Stream::configure( StreamConfig *const config, TCB *const cb )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if( !config || !cb )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Try to apply the configuration to the DMA stream
    -------------------------------------------------*/
    disableInterrupts();
    {
      /*-------------------------------------------------
      Check to see if a transfer is currently going
      -------------------------------------------------*/
      if( mTCB.state != StreamState::TRANSFER_IDLE )
      {
        enableInterrupts();
        return Chimera::Status::BUSY;
      }

      /*-------------------------------------------------
      Pull in the user's TCB and reset it's state
      -------------------------------------------------*/
      memcpy( &mTCB, cb, sizeof( TCB ) );

      mTCB.state            = StreamState::TRANSFER_IDLE;
      mTCB.bytesTransferred = 0;
      mTCB.selectedChannel  = 0;

      /*-------------------------------------------------
      Step 1: Disable the stream (should already be) and
      clear out the LISR/HISR registers to reset back to
      a configurable state.
      -------------------------------------------------*/
      EN::clear( mStream, SxCR_EN );
      while( EN::get( mStream ) )
      {
        continue;
      }
      reset_isr();

      /*-------------------------------------------------
      Step 2/3: Set the address registers
      -------------------------------------------------*/
      if ( ( config->direction == Chimera::DMA::Direction::PERIPH_TO_MEMORY ) ||
           ( config->direction == Chimera::DMA::Direction::MEMORY_TO_MEMORY ) )
      {
        PA::set( mStream, cb->srcAddress );
        M0A::set( mStream, cb->dstAddress );
      }
      else if ( config->direction == Chimera::DMA::Direction::MEMORY_TO_PERIPH )
      {
        M0A::set( mStream, cb->srcAddress );
        PA::set( mStream, cb->dstAddress );
      }
      else
      {
        return Chimera::Status::FAIL;
      }

      /*------------------------------------------------
      Step 4: Set how many bytes are to be transferred
      ------------------------------------------------*/
      NDT::set( mStream, cb->transferSize );

      /*------------------------------------------------
      Step 5: Select the DMA channel request
      ------------------------------------------------*/
      CHSEL::set( mStream, EnumValue( config->channel ) << SxCR_CHSEL_Pos);

      /*------------------------------------------------
      Step 6: Set the DMA mode. This includes flow
      control and the memory access style.
      ------------------------------------------------*/
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
      else if( config->dmaMode == Chimera::DMA::Mode::DOUBLE_BUFFER )
      {
        /* Double buffer mode currently not supported */
        RT_HARD_ASSERT( false );
      }
      // else normal DMA peripheral is the flow controller

      /*------------------------------------------------
      Step 7: Set the transfer priority
      ------------------------------------------------*/
      switch( config->priority )
      {
        case Chimera::DMA::Priority::LOW:
          PL::set( mStream, 0x00 << SxCR_PL_Pos );
          break;

        case Chimera::DMA::Priority::MEDIUM:
          PL::set( mStream, 0x01 << SxCR_PL_Pos );
          break;

        case Chimera::DMA::Priority::HIGH:
          PL::set( mStream, 0x02 << SxCR_PL_Pos );
          break;

        case Chimera::DMA::Priority::VERY_HIGH:
          PL::set( mStream, 0x03 << SxCR_PL_Pos );
          break;

        default:
          RT_HARD_ASSERT( false );
          break;
      }

      /*------------------------------------------------
      Step 8: Set the FIFO usage
      ------------------------------------------------*/
      /* Direct Mode */
      DMDIS::set( mStream, SxFCR_DMDIS );
      if( config->fifoMode == FifoMode::DIRECT_ENABLE )
      {
        DMDIS::clear( mStream, SxFCR_DMDIS );
      }

      /* Threshold */
      FTH::set( mStream, EnumValue( config->fifoThreshold ) << SxFCR_FTH_Pos );

      /*------------------------------------------------
      Step 9: Data transfer direction
      ------------------------------------------------*/
      switch( config->direction )
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

      /*-------------------------------------------------
      Step 10: Memory Settings
      -------------------------------------------------*/
      /* Memory Auto-Increment */
      MINC::clear( mStream, SxCR_MINC );
      if( config->memoryAddrIncr )
      {
        MINC::set( mStream, SxCR_MINC );
      }

      /* Memory Burst Size */
      switch( config->memoryBurstSize )
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
          RT_HARD_ASSERT( false );
          break;
      };

      /* Memory Alignment */
      switch( config->memoryAddrAlign )
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
          RT_HARD_ASSERT( false );
          break;
      };

      /*-------------------------------------------------
      Step 11: Peripheral Settings
      -------------------------------------------------*/
      /* Peripheral Auto-Increment */
      PINCOS::clear( mStream, SxCR_PINCOS );
      PINC::clear( mStream, SxCR_PINC );
      if( config->periphAddrIncr )
      {
        PINC::set( mStream, SxCR_PINC );
      }

      /* Peripheral Burst Size */
      switch( config->periphBurstSize )
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
          RT_HARD_ASSERT( false );
          break;
      };

      /* Peripheral Alignment */
      switch( config->periphAddrAlign )
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
          RT_HARD_ASSERT( false );
          break;
      };

      /*-------------------------------------------------
      Step 12: Interrupt Settings
      By default, enable everything.
      -------------------------------------------------*/
      TCIE::set( mStream, SxCR_TCIE );   /* Transfer complete */
      HTIE::set( mStream, SxCR_HTIE );   /* Half-Tranfer complete */
      TEIE::set( mStream, SxCR_TEIE );   /* Transfer error */
      DMEIE::set( mStream, SxCR_DMEIE ); /* Direct mode error */
      FEIE::set( mStream, SxFCR_FEIE );  /* Fifo error */

      mTCB.state = StreamState::TRANSFER_CONFIGURED;
    }
    enableInterrupts();

    return Chimera::Status::OK;
  }


  Chimera::Status_t Stream::start()
  {
    disableInterrupts();
    {
      /*-------------------------------------------------
      Make sure the hardware is in the correct state
      -------------------------------------------------*/
      if( mTCB.state != StreamState::TRANSFER_CONFIGURED )
      {
        enableInterrupts();
        return Chimera::Status::FAIL;
      }

      mTCB.state = StreamState::TRANSFER_IN_PROGRESS;
    }
    enableInterrupts();
    EN::set( mStream, SxCR_EN );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Stream::abort()
  {
    /*-------------------------------------------------
    Should abruptly disable the hardware and fire an
    ISR if there is an ongoing transfer
    -------------------------------------------------*/
    EN::clear( mStream, SxCR_EN );
    return Chimera::Status::OK;
  }


  void Stream::IRQHandler( const uint8_t channel, const uint8_t status )
  {
    using namespace Chimera::Peripheral;
    using namespace Chimera::Thread;

    static constexpr uint8_t TCIF  = 1u << 5;
    static constexpr uint8_t HTCIF = 1u << 4;
    static constexpr uint8_t TEIF  = 1u << 3;
    static constexpr uint8_t DMEIF = 1u << 2;
    static constexpr uint8_t FEIF  = 1u << 0;

    bool disableIsr = false;

    /*------------------------------------------------
    Read the status registers and parse the flags for this stream
    ------------------------------------------------*/
    const Reg32_t sxcr = CR_ALL::get( mStream );
    const Reg32_t fxcr = FCR_ALL::get( mStream );

    /*------------------------------------------------
    Reset the control block
    ------------------------------------------------*/
    mTCB.fifoError       = false;
    mTCB.transferError   = false;
    mTCB.directModeError = false;

    /*------------------------------------------------
    Transfer Complete
    ------------------------------------------------*/
    if ( ( status & TCIF ) && TCIE::get( mStream ) )
    {
      disableIsr = true;

      /*------------------------------------------------
      Update control block with the event information
      ------------------------------------------------*/
      mTCB.selectedChannel  = channel;
      mTCB.bytesTransferred = mTCB.transferSize - NDT::get( mStream );
      mTCB.state            = mTCB.state | StreamState::TRANSFER_COMPLETE;
    }

    /*------------------------------------------------
    Transfer Half-Complete
    ------------------------------------------------*/
    if ( ( status & HTCIF ) && HTIE::get( mStream ) ) {}    // Currently not supported

    /*------------------------------------------------
    Transfer Errors
    ------------------------------------------------*/
    if ( ( status & TEIF ) && TEIE::get( mStream ) )
    {
      disableIsr         = true;
      mTCB.transferError = true;
    }

    if ( ( status & DMEIF ) && DMEIE::get( mStream ) )
    {
      disableIsr           = true;
      mTCB.directModeError = true;
    }

    if ( ( status & FEIF ) && FEIE::get( mStream ) )
    {
      disableIsr     = true;
      mTCB.fifoError = true;
    }

    /*-------------------------------------------------
    Disable ISR signals on exit if needed
    -------------------------------------------------*/
    if( disableIsr )
    {
      TCIE::clear( mStream, SxCR_TCIE );
      HTIE::clear( mStream, SxCR_HTIE );
      TEIE::clear( mStream, SxCR_TEIE );
      DMEIE::clear( mStream, SxCR_DMEIE );
      FEIE::clear( mStream, SxFCR_FEIE );

      /*-------------------------------------------------
      Wake up the HLD thread to handle the events
      -------------------------------------------------*/
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_DMA ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  void Stream::disableInterrupts()
  {
    Thor::LLD::INT::disableIRQ( mIRQn );
  }


  void Stream::enableInterrupts()
  {
    Thor::LLD::INT::enableIRQ( mIRQn );
  }


  void Stream::reset_isr()
  {
    switch( getStream( reinterpret_cast<std::uintptr_t>( mStream ) ) )
    {
      case Streamer::STREAM_0:
        CTCIF0::set( mPeriph, LIFCR_CTCIF0 );
        CHTIF0::set( mPeriph, LIFCR_CHTIF0 );
        CTEIF0::set( mPeriph, LIFCR_CTEIF0 );
        CDMEIF0::set( mPeriph, LIFCR_CDMEIF0 );
        CFEIF0::set( mPeriph, LIFCR_CFEIF0 );
        break;

      case Streamer::STREAM_1:
        CTCIF1::set( mPeriph, LIFCR_CTCIF1 );
        CHTIF1::set( mPeriph, LIFCR_CHTIF1 );
        CTEIF1::set( mPeriph, LIFCR_CTEIF1 );
        CDMEIF1::set( mPeriph, LIFCR_CDMEIF1 );
        CFEIF1::set( mPeriph, LIFCR_CFEIF1 );
        break;

      case Streamer::STREAM_2:
        CTCIF2::set( mPeriph, LIFCR_CTCIF2 );
        CHTIF2::set( mPeriph, LIFCR_CHTIF2 );
        CTEIF2::set( mPeriph, LIFCR_CTEIF2 );
        CDMEIF2::set( mPeriph, LIFCR_CDMEIF2 );
        CFEIF2::set( mPeriph, LIFCR_CFEIF2 );
        break;

      case Streamer::STREAM_3:
        CTCIF3::set( mPeriph, LIFCR_CTCIF3 );
        CHTIF3::set( mPeriph, LIFCR_CHTIF3 );
        CTEIF3::set( mPeriph, LIFCR_CTEIF3 );
        CDMEIF3::set( mPeriph, LIFCR_CDMEIF3 );
        CFEIF3::set( mPeriph, LIFCR_CFEIF3 );
        break;

      case Streamer::STREAM_4:
        CTCIF4::set( mPeriph, HIFCR_CTCIF4 );
        CHTIF4::set( mPeriph, HIFCR_CHTIF4 );
        CTEIF4::set( mPeriph, HIFCR_CTEIF4 );
        CDMEIF4::set( mPeriph, HIFCR_CDMEIF4 );
        CFEIF4::set( mPeriph, HIFCR_CFEIF4 );
        break;

      case Streamer::STREAM_5:
        CTCIF5::set( mPeriph, HIFCR_CTCIF5 );
        CHTIF5::set( mPeriph, HIFCR_CHTIF5 );
        CTEIF5::set( mPeriph, HIFCR_CTEIF5 );
        CDMEIF5::set( mPeriph, HIFCR_CDMEIF5 );
        CFEIF5::set( mPeriph, HIFCR_CFEIF5 );
        break;

      case Streamer::STREAM_6:
        CTCIF6::set( mPeriph, HIFCR_CTCIF6 );
        CHTIF6::set( mPeriph, HIFCR_CHTIF6 );
        CTEIF6::set( mPeriph, HIFCR_CTEIF6 );
        CDMEIF6::set( mPeriph, HIFCR_CDMEIF6 );
        CFEIF6::set( mPeriph, HIFCR_CFEIF6 );
        break;

      case Streamer::STREAM_7:
        CTCIF7::set( mPeriph, HIFCR_CTCIF7 );
        CHTIF7::set( mPeriph, HIFCR_CHTIF7 );
        CTEIF7::set( mPeriph, HIFCR_CTEIF7 );
        CDMEIF7::set( mPeriph, HIFCR_CDMEIF7 );
        CFEIF7::set( mPeriph, HIFCR_CFEIF7 );
        break;

      default:
        break;
    }
  }

}  // namespace Thor::LLD::DMA
