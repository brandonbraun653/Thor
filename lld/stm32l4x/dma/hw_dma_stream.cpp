/********************************************************************************
 *  File Name:
 *    hw_dma_stream.cpp
 *
 *  Description:
 *    DMA Stream Implementation
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
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
    if ( !peripheral || !parent )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Grab the resource index for the stream
    -------------------------------------------------*/
    mResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );
    if ( mResourceIndex == INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------
    Register the peripheral
    -------------------------------------------------*/
    mStream = peripheral;
    mPeriph = parent;
    mIRQn   = Resource::IRQSignals[ mResourceIndex ];

    /*-------------------------------------------------
    Configure the global interrupt priority
    -------------------------------------------------*/
    INT::setPriority( mIRQn, INT::DMA_STREAM_PREEMPT_PRIORITY, 0u );
    INT::enableIRQ( mIRQn );

    /*-------------------------------------------------
    Initialize the transfer control block
    -------------------------------------------------*/
    mTCB.state = StreamState::TRANSFER_IDLE;

    return Chimera::Status::OK;
  }


  Chimera::Status_t Stream::configure( StreamConfig *const config, TCB *const cb )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if ( !config || !cb )
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
      if ( mTCB.state != StreamState::TRANSFER_IDLE )
      {
        enableInterrupts();
        return Chimera::Status::BUSY;
      }

      /*-------------------------------------------------
      Pull in the user's TCB and reset it's state
      -------------------------------------------------*/
      memcpy( &mTCB, cb, sizeof( TCB ) );

      mTCB.state               = StreamState::TRANSFER_IDLE;
      mTCB.elementsTransferred = 0;
      mTCB.resourceIndex       = mResourceIndex;

      /*-------------------------------------------------
      Disable the stream and clear out the LISR/HISR
      registers to reset back to a configurable state.
      -------------------------------------------------*/
      EN::clear( mStream, CCR_EN );
      while ( EN::get( mStream ) )
      {
        continue;
      }
      reset_isr_flags();

      /*-------------------------------------------------
      Configure direction dependent settings
      -------------------------------------------------*/
      if ( ( config->direction == Chimera::DMA::Direction::PERIPH_TO_MEMORY ) ||
           ( config->direction == Chimera::DMA::Direction::MEMORY_TO_MEMORY ) )
      {
        PA::set( mStream, cb->srcAddress );
        configure_periph_settings( config->srcAddrIncr, config->srcBurstSize, config->srcAddrAlign );

        MA::set( mStream, cb->dstAddress );
        configure_memory_settings( config->dstAddrIncr, config->dstBurstSize, config->dstAddrAlign );
      }
      else if ( config->direction == Chimera::DMA::Direction::MEMORY_TO_PERIPH )
      {
        MA::set( mStream, cb->srcAddress );
        configure_memory_settings( config->srcAddrIncr, config->srcBurstSize, config->srcAddrAlign );

        PA::set( mStream, cb->dstAddress );
        configure_periph_settings( config->dstAddrIncr, config->dstBurstSize, config->dstAddrAlign );
      }
      else
      {
        return Chimera::Status::FAIL;
      }

      /*------------------------------------------------
      Set how many bytes are to be transferred
      ------------------------------------------------*/
      NDT::set( mStream, cb->transferSize );

      /*------------------------------------------------
      Select the DMA channel request
      ------------------------------------------------*/
      switch ( mResourceIndex )
      {
        case DMA1_STREAM1_RESOURCE_INDEX:
        case DMA2_STREAM1_RESOURCE_INDEX:
          CS1S::set( mPeriph, EnumValue( config->channel ) << CSELR_C1S_Pos );
          break;

        case DMA1_STREAM2_RESOURCE_INDEX:
        case DMA2_STREAM2_RESOURCE_INDEX:
          CS2S::set( mPeriph, EnumValue( config->channel ) << CSELR_C2S_Pos );
          break;

        case DMA1_STREAM3_RESOURCE_INDEX:
        case DMA2_STREAM3_RESOURCE_INDEX:
          CS3S::set( mPeriph, EnumValue( config->channel ) << CSELR_C3S_Pos );
          break;

        case DMA1_STREAM4_RESOURCE_INDEX:
        case DMA2_STREAM4_RESOURCE_INDEX:
          CS4S::set( mPeriph, EnumValue( config->channel ) << CSELR_C4S_Pos );
          break;

        case DMA1_STREAM5_RESOURCE_INDEX:
        case DMA2_STREAM5_RESOURCE_INDEX:
          CS5S::set( mPeriph, EnumValue( config->channel ) << CSELR_C5S_Pos );
          break;

        case DMA1_STREAM6_RESOURCE_INDEX:
        case DMA2_STREAM6_RESOURCE_INDEX:
          CS6S::set( mPeriph, EnumValue( config->channel ) << CSELR_C6S_Pos );
          break;

        case DMA1_STREAM7_RESOURCE_INDEX:
        case DMA2_STREAM7_RESOURCE_INDEX:
          CS7S::set( mPeriph, EnumValue( config->channel ) << CSELR_C7S_Pos );
          break;

        default:
          // Do nothing
          break;
      };

      /*------------------------------------------------
      Set the DMA mode. This includes flow control and
      the memory access style.
      ------------------------------------------------*/
      CIRC::clear( mStream, CCR_CIRC );
      if ( config->dmaMode == Chimera::DMA::Mode::CIRCULAR )
      {
        CIRC::set( mStream, CCR_CIRC );
      }

      /*------------------------------------------------
      Set the transfer priority
      ------------------------------------------------*/
      switch ( config->priority )
      {
        case Chimera::DMA::Priority::LOW:
          PL::set( mStream, 0x00 << CCR_PL_Pos );
          break;

        case Chimera::DMA::Priority::MEDIUM:
          PL::set( mStream, 0x01 << CCR_PL_Pos );
          break;

        case Chimera::DMA::Priority::HIGH:
          PL::set( mStream, 0x02 << CCR_PL_Pos );
          break;

        case Chimera::DMA::Priority::VERY_HIGH:
          PL::set( mStream, 0x03 << CCR_PL_Pos );
          break;

        default:
          RT_HARD_ASSERT( false );
          break;
      }

      /*------------------------------------------------
      Data transfer direction
      ------------------------------------------------*/
      switch ( config->direction )
      {
        case Chimera::DMA::Direction::PERIPH_TO_MEMORY:
          DIR::set( mStream, 0x00 << CCR_DIR_Pos );
          MEM2MEM::set( mStream, 0 );
          break;

        case Chimera::DMA::Direction::MEMORY_TO_PERIPH:
        case Chimera::DMA::Direction::MEMORY_TO_MEMORY:
          DIR::set( mStream, 0x01 << CCR_DIR_Pos );
          MEM2MEM::set( mStream, CCR_MEM2MEM );
          break;

        default:
          RT_HARD_ASSERT( false );
          break;
      }

      /*-------------------------------------------------
      Interrupt Settings: By default, enable everything
      -------------------------------------------------*/
      mStream->CCR |= ( CCR_TCIE | CCR_HTIE | CCR_TEIE );

      /*-------------------------------------------------
      Configure the global interrupt priority
      -------------------------------------------------*/
      INT::setPriority( mIRQn, INT::DMA_STREAM_PREEMPT_PRIORITY, 0u );
      INT::enableIRQ( mIRQn );

      mTCB.state = StreamState::TRANSFER_CONFIGURED;
      mCfg       = *config;
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
      if ( mTCB.state != StreamState::TRANSFER_CONFIGURED )
      {
        enableInterrupts();
        return Chimera::Status::FAIL;
      }

      mTCB.state = StreamState::TRANSFER_IN_PROGRESS;
    }
    enableInterrupts();

    /*-------------------------------------------------
    Enable the stream
    -------------------------------------------------*/
    EN::set( mStream, CCR_EN );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Stream::abort()
  {
    /*-------------------------------------------------
    Should abruptly disable the hardware and fire an
    ISR if there is an ongoing transfer
    -------------------------------------------------*/
    EN::clear( mStream, CCR_EN );
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
    static constexpr uint8_t TEIF  = 1u << 3;
    static constexpr uint8_t HTCIF = 1u << 2;
    static constexpr uint8_t TCIF  = 1u << 1;
    static constexpr uint8_t GIF   = 1u << 0;

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    bool disableIsr     = false;
    bool wakeUserThread = false;

    /*-------------------------------------------------------------------------
    Transfer Errors: Allow these to be ignored due to some quirks with the
    hardware. USARTs will throw FIFO errors even though it doesn't use it.
    -------------------------------------------------------------------------*/
    mTCB.fifoError       = false;
    mTCB.directModeError = false;
    mTCB.transferError   = ( status & TEIF ) && ( ( mTCB.errorsToIgnore & Errors::TRANSFER ) != Errors::TRANSFER );

    if ( mTCB.transferError )
    {
      disableIsr     = true;
      wakeUserThread = mTCB.wakeUserOnComplete;
      mTCB.state     = StreamState::ERROR;
    }

    /*-------------------------------------------------------------------------
    Transfer Complete?
    -------------------------------------------------------------------------*/
    if ( ( status & TCIF ) && TCIE::get( mStream ) )
    {
      mTCB.selectedChannel     = channel;
      mTCB.elementsTransferred = mTCB.transferSize - NDT::get( mStream );
      mTCB.state               = StreamState::TRANSFER_COMPLETE;

      /*-----------------------------------------------------------------------
      Only disable DMA if requested
      -----------------------------------------------------------------------*/
      if ( !mTCB.persistent )
      {
        disableIsr     = true;
        wakeUserThread = mTCB.wakeUserOnComplete;
        EN::clear( mStream, CCR_EN );
      }

      /*-----------------------------------------------------------------------
      ACK the transfer complete flag
      -----------------------------------------------------------------------*/
      reset_isr_flags();

      /*-----------------------------------------------------------------------
      Invoke the ISR callback should one exist
      -----------------------------------------------------------------------*/
      if ( mTCB.isrCallback )
      {
        Chimera::DMA::TransferStats stats;
        stats.clear();
        stats.error     = false;
        stats.requestId = mTCB.requestId;
        stats.size      = mTCB.elementsTransferred;

        mTCB.isrCallback( stats );
      }
    }

    /*-------------------------------------------------------------------------
    ACK the half-transfer complete flag
    -------------------------------------------------------------------------*/
    if ( ( status & HTCIF ) && HTIE::get( mStream ) )
    {
      reset_isr_flags();
    }

    /*-------------------------------------------------------------------------
    Disable the ISR signals on exit if needed
    -------------------------------------------------------------------------*/
    if ( disableIsr )
    {
      mStream->CCR &= ~( CCR_TCIE | CCR_HTIE | CCR_TEIE );
      reset_isr_flags();
    }

    /*-------------------------------------------------------------------------
    Wake up the user thread to handle the generated events
    -------------------------------------------------------------------------*/
    if ( wakeUserThread )
    {
      Resource::ISRQueue.push( mTCB );
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_DMA ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  void Stream::ackTransfer()
  {
    disableInterrupts();
    mTCB.state = StreamState::TRANSFER_IDLE;
    enableInterrupts();
  }


  void Stream::disableInterrupts()
  {
    Thor::LLD::INT::disableIRQ( mIRQn );
  }


  void Stream::enableInterrupts()
  {
    Thor::LLD::INT::enableIRQ( mIRQn );
  }


  void Stream::reset_isr_flags()
  {
    switch ( getStream( reinterpret_cast<std::uintptr_t>( mStream ) ) )
    {
      case Streamer::STREAM_1:
        mPeriph->IFCR = IFCR_CTCIF1 | IFCR_CHTIF1 | IFCR_CTEIF1 | IFCR_CGIF1;
        break;

      case Streamer::STREAM_2:
        mPeriph->IFCR = IFCR_CTCIF2 | IFCR_CHTIF2 | IFCR_CTEIF2 | IFCR_CGIF2;
        break;

      case Streamer::STREAM_3:
        mPeriph->IFCR = IFCR_CTCIF3 | IFCR_CHTIF3 | IFCR_CTEIF3 | IFCR_CGIF3;
        break;

      case Streamer::STREAM_4:
        mPeriph->IFCR = IFCR_CTCIF4 | IFCR_CHTIF4 | IFCR_CTEIF4 | IFCR_CGIF4;
        break;

      case Streamer::STREAM_5:
        mPeriph->IFCR = IFCR_CTCIF5 | IFCR_CHTIF5 | IFCR_CTEIF5 | IFCR_CGIF5;
        break;

      case Streamer::STREAM_6:
        mPeriph->IFCR = IFCR_CTCIF6 | IFCR_CHTIF6 | IFCR_CTEIF6 | IFCR_CGIF6;
        break;

      case Streamer::STREAM_7:
        mPeriph->IFCR = IFCR_CTCIF7 | IFCR_CHTIF7 | IFCR_CTEIF7 | IFCR_CGIF7;
        break;

      default:
        break;
    }
  }


  void Stream::configure_memory_settings( const bool incr, const Chimera::DMA::BurstSize bSize,
                                          const Chimera::DMA::Alignment align )
  {
    /* Memory Auto-Increment */
    MINC::clear( mStream, CCR_MINC );
    if ( incr )
    {
      MINC::set( mStream, CCR_MINC );
    }

    /* Memory Alignment */
    switch ( align )
    {
      case Chimera::DMA::Alignment::BYTE:
        MSIZE::set( mStream, 0x00 << CCR_MSIZE_Pos );
        break;

      case Chimera::DMA::Alignment::HALF_WORD:
        MSIZE::set( mStream, 0x01 << CCR_MSIZE_Pos );
        break;

      case Chimera::DMA::Alignment::WORD:
        MSIZE::set( mStream, 0x02 << CCR_MSIZE_Pos );
        break;

      default:
        // Not necessarily needs configuration
        break;
    };
  }


  void Stream::configure_periph_settings( const bool incr, const Chimera::DMA::BurstSize bSize,
                                          const Chimera::DMA::Alignment align )
  {
    /* Peripheral Auto-Increment */
    PINC::clear( mStream, CCR_PINC );
    if ( incr )
    {
      PINC::set( mStream, CCR_PINC );
    }


    /* Peripheral Alignment */
    switch ( align )
    {
      case Chimera::DMA::Alignment::BYTE:
        PSIZE::set( mStream, 0x00 << CCR_PSIZE_Pos );
        break;

      case Chimera::DMA::Alignment::HALF_WORD:
        PSIZE::set( mStream, 0x01 << CCR_PSIZE_Pos );
        break;

      case Chimera::DMA::Alignment::WORD:
        PSIZE::set( mStream, 0x02 << CCR_PSIZE_Pos );
        break;

      default:
        // Not necessarily needs configuration
        break;
    };
  }

}    // namespace Thor::LLD::DMA
