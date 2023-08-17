/******************************************************************************
 *  File Name:
 *    dma_common_stream.cpp
 *
 *  Description:
 *    Shared driver for DMA streams across multiple STM32 chips
 *
 *  2023 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/interrupt>

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
    streamEnable( mStream );
    return Chimera::Status::OK;
  }


  void Stream::abort()
  {
    using namespace Chimera::Peripheral;
    using namespace Chimera::Thread;

    /*-------------------------------------------------------------------------
    Disable interrupts and the stream
    -------------------------------------------------------------------------*/
    streamClearInterruptEnableFlags( mStream );
    streamDisable( mStream );
    reset_isr_flags();
    mStreamTCB.state         = StreamState::TRANSFER_IDLE;
    mStreamTCB.transferError = false;

    /*-----------------------------------------------------------------------
    Calculate the number of elements transferred. This changes behavior based
    upon who acts as the flow controller.
    See RM0390 Section 9.2 and 9.3.16
    -----------------------------------------------------------------------*/
    if( PFCTRL::get( mStream ) )
    {
      /* Peripheral is the flow controller */
      mStreamTCB.elementsTransferred = 0xFFFFu - NDT::get( mStream );
    }
    else
    {
      /* DMA is the flow controller */
      mStreamTCB.elementsTransferred = mStreamTCB.transferSize - NDT::get( mStream );
    }

    /*-------------------------------------------------------------------------
    If an ISR callback was registered, invoke it now
    -------------------------------------------------------------------------*/
    if ( mStreamTCB.isrCallback )
    {
      Chimera::DMA::TransferStats stats;
      stats.error     = mStreamTCB.transferError;
      stats.requestId = mStreamTCB.requestId;
      stats.size      = mStreamTCB.elementsTransferred;

      mStreamTCB.isrCallback( stats );
    }

    /*-------------------------------------------------------------------------
    Let the user thread handle acknowledging the result and associated callback
    -------------------------------------------------------------------------*/
    if( mStreamTCB.wakeUserOnComplete )
    {
      Resource::ISRQueue.push( mStreamTCB );
      sendTaskMsg( INT::getUserTaskId( Type::PERIPH_DMA ), ITCMsg::TSK_MSG_ISR_HANDLER, TIMEOUT_DONT_WAIT );
    }
  }


  void Stream::disableInterrupts()
  {
    Thor::LLD::INT::disableIRQ( mStreamIRQn );
  }


  void Stream::enableInterrupts()
  {
    Thor::LLD::INT::enableIRQ( mStreamIRQn );
  }

}  // namespace Thor::LLD::DMA
