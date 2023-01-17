/******************************************************************************
 *  File Name:
 *    hw_dma_stream.cpp
 *
 *  Description:
 *    DMA Stream Implementation
 *
 *  2022 | Brandon Braun | brandonbraun653@gmail.com
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
  Stream Class
  ---------------------------------------------------------------------------*/
  Stream::Stream()
  {
  }


  Stream::~Stream()
  {
  }


  Chimera::Status_t Stream::attach( StreamMap *const peripheral, RegisterMap *const parent )
  {
    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !peripheral || !parent )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------------------------------
    Grab the resource index for the stream
    -------------------------------------------------------------------------*/
    mStreamResourceIndex = getResourceIndex( reinterpret_cast<std::uintptr_t>( peripheral ) );
    if ( mStreamResourceIndex == INVALID_RESOURCE_INDEX )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }

    /*-------------------------------------------------------------------------
    Register the peripheral
    -------------------------------------------------------------------------*/
    mStream           = peripheral;
    mPeriph           = parent;
    mStreamPhysicalId = getStream( reinterpret_cast<std::uintptr_t>( mStream ) );
    mStreamIRQn       = Resource::IRQSignals[ mStreamResourceIndex ];

    /*-------------------------------------------------------------------------
    Configure the global interrupt priority
    -------------------------------------------------------------------------*/
    INT::setPriority( mStreamIRQn, INT::DMA_STREAM_PREEMPT_PRIORITY, 0u );
    INT::enableIRQ( mStreamIRQn );

    /*-------------------------------------------------------------------------
    Initialize the transfer control block
    -------------------------------------------------------------------------*/
    mStreamTCB.state = StreamState::TRANSFER_IDLE;

    return Chimera::Status::OK;
  }


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
      Disable the stream and clear out the LISR/HISR
      registers to reset back to a configurable state.
      -----------------------------------------------------------------------*/
      EN::clear( mStream, CCR_EN );
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

      /*-----------------------------------------------------------------------
      Set how many bytes are to be transferred
      -----------------------------------------------------------------------*/
      NDT::set( mStream, cb->transferSize );

      /*-----------------------------------------------------------------------
      Select the DMA channel request
      -----------------------------------------------------------------------*/
      uint32_t pos = mStreamResourceIndex * 4;
      uint32_t msk = 0xF << pos;
      uint32_t tmp = mPeriph->CSELR;

      tmp &= ~msk;
      tmp |= EnumValue( config->channel ) << pos;
      mPeriph->CSELR = tmp;

      /*-----------------------------------------------------------------------
      Set the DMA mode. This includes flow control and the memory access style.
      -----------------------------------------------------------------------*/
      CIRC::clear( mStream, CCR_CIRC );
      if ( config->dmaMode == Chimera::DMA::Mode::CIRCULAR )
      {
        CIRC::set( mStream, CCR_CIRC );
      }

      /*-----------------------------------------------------------------------
      Set the transfer priority
      -----------------------------------------------------------------------*/
      PL::set( mStream, EnumValue( config->priority ) << CCR_PL_Pos );

      static_assert( EnumValue( Chimera::DMA::Priority::LOW ) == 0 );
      static_assert( EnumValue( Chimera::DMA::Priority::MEDIUM ) == 1 );
      static_assert( EnumValue( Chimera::DMA::Priority::HIGH ) == 2 );
      static_assert( EnumValue( Chimera::DMA::Priority::VERY_HIGH ) == 3 );

      /*-----------------------------------------------------------------------
      Data transfer direction
      -----------------------------------------------------------------------*/
      MEM2MEM::clear( mStream, CCR_MEM2MEM );
      DIR::clear( mStream, CCR_DIR );

      switch ( config->direction )
      {
        case Chimera::DMA::Direction::MEMORY_TO_PERIPH:
        case Chimera::DMA::Direction::MEMORY_TO_MEMORY:
          DIR::set( mStream, CCR_DIR );

          if ( config->direction == Chimera::DMA::Direction::MEMORY_TO_MEMORY )
          {
            MEM2MEM::set( mStream, CCR_MEM2MEM );
          }
          break;

        case Chimera::DMA::Direction::PERIPH_TO_MEMORY:
        default:
          // Do nothing
          break;
      };

      /*-----------------------------------------------------------------------
      Interrupt Settings: If the user registers a callback, make sure the ISR
      handlers get executed to process it. Always allow the error events to be
      processed by driver.
      -----------------------------------------------------------------------*/
      mStream->CCR |= CCR_TEIE;

      if ( mStreamTCB.isrCallback )
      {
        mStream->CCR |= CCR_TCIE;
      }
      else
      {
        mStream->CCR &= ~CCR_TCIE;
      }

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


  Chimera::Status_t Stream::start()
  {
    disableInterrupts();
    {
      /*-----------------------------------------------------------------------
      Make sure the hardware is in the correct state
      -----------------------------------------------------------------------*/
      if ( mStreamTCB.state != StreamState::TRANSFER_CONFIGURED )
      {
        enableInterrupts();
        return Chimera::Status::FAIL;
      }

      mStreamTCB.state = StreamState::TRANSFER_IN_PROGRESS;
    }
    enableInterrupts();

    /*-------------------------------------------------------------------------
    Enable the stream
    -------------------------------------------------------------------------*/
    EN::set( mStream, CCR_EN );

    return Chimera::Status::OK;
  }


  Chimera::Status_t Stream::abort()
  {
    disableInterrupts();
    {
      EN::clear( mStream, CCR_EN );
    }
    enableInterrupts();
    return Chimera::Status::OK;
  }


  __attribute__( ( long_call, section( ".thor_ram_code" ) ) ) void Stream::IRQHandler( const uint8_t channel,
                                                                                       const uint8_t status )
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

    /*-------------------------------------------------------------------------
    Local Variables
    -------------------------------------------------------------------------*/
    bool disableIsr     = false;
    bool wakeUserThread = false;

    /*-------------------------------------------------------------------------
    Transfer Errors: Allow these to be ignored due to some quirks with the
    hardware. USARTs will throw FIFO errors even though it doesn't use FIFOs.
    -------------------------------------------------------------------------*/
    if ( ( status & TEIF ) && ( ( mStreamTCB.errorsToIgnore & Errors::TRANSFER ) != Errors::TRANSFER ) )
    {
      disableIsr               = true;
      wakeUserThread           = mStreamTCB.wakeUserOnComplete;
      mStreamTCB.state         = StreamState::ERROR;
      mStreamTCB.transferError = true;
    }

    /*-------------------------------------------------------------------------
    Transfer Complete?
    -------------------------------------------------------------------------*/
    if ( ( status & TCIF ) && TCIE::get( mStream ) )
    {
      mStreamTCB.selectedChannel     = channel;
      mStreamTCB.elementsTransferred = mStreamTCB.transferSize - NDT::get( mStream );
      mStreamTCB.state               = StreamState::TRANSFER_COMPLETE;

      /*-----------------------------------------------------------------------
      Only disable DMA if requested
      -----------------------------------------------------------------------*/
      if ( !mStreamTCB.persistent )
      {
        disableIsr     = true;
        wakeUserThread = mStreamTCB.wakeUserOnComplete;
        EN::clear( mStream, CCR_EN );
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
      Resource::ISRQueue.push( mStreamTCB );
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_DMA ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  void Stream::ackTransfer()
  {
    disableInterrupts();
    mStreamTCB.state = StreamState::TRANSFER_IDLE;
    enableInterrupts();
  }


  void Stream::disableInterrupts()
  {
    Thor::LLD::INT::disableIRQ( mStreamIRQn );
  }


  void Stream::enableInterrupts()
  {
    Thor::LLD::INT::enableIRQ( mStreamIRQn );
  }


  void Stream::reset_isr_flags()
  {
    /*-------------------------------------------------------------------------
    Lookup table reduces execution time. Function may execute inside an ISR.
    -------------------------------------------------------------------------*/
    static const uint32_t ifcr_flag_table[ EnumValue( Streamer::NUM_OPTIONS ) ] = {
      0,    // Stream 0 is unused on L4
      IFCR_CTCIF1 | IFCR_CHTIF1 | IFCR_CTEIF1 | IFCR_CGIF1,
      IFCR_CTCIF2 | IFCR_CHTIF2 | IFCR_CTEIF2 | IFCR_CGIF2,
      IFCR_CTCIF3 | IFCR_CHTIF3 | IFCR_CTEIF3 | IFCR_CGIF3,
      IFCR_CTCIF4 | IFCR_CHTIF4 | IFCR_CTEIF4 | IFCR_CGIF4,
      IFCR_CTCIF5 | IFCR_CHTIF5 | IFCR_CTEIF5 | IFCR_CGIF5,
      IFCR_CTCIF6 | IFCR_CHTIF6 | IFCR_CTEIF6 | IFCR_CGIF6,
      IFCR_CTCIF7 | IFCR_CHTIF7 | IFCR_CTEIF7 | IFCR_CGIF7
    };

    mPeriph->IFCR = ifcr_flag_table[ EnumValue( mStreamPhysicalId ) ];
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
