/********************************************************************************
 *  File Name:
 *    hw_dma_stream.cpp
 *
 *  Description:
 *    DMA Stream Implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/lld/interface/inc/dma>


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
    using namespace Chimera::Thread;

    auto result = Chimera::Status::LOCKED;


    return result;
  }


  Chimera::Status_t Stream::configure( StreamConfig *const config, TCB *const cb )
  {
    using namespace Chimera::Thread;

    auto result = Chimera::Status::LOCKED;

    // if ( try_lock_for( 100 ) )
    // {
    //   enterCriticalSection();
    //   result = Chimera::Status::OK;

    //   /*------------------------------------------------
    //   Initialize the control block
    //   ------------------------------------------------*/
    //   memcpy( &controlBlock, cb, sizeof( TCB ) );
    //   controlBlock.transferState = Runtime::Flag::TRANSFER_NOT_READY;

    //   /*------------------------------------------------
    //   Step 1: Check for enabled status and clear ISR flags
    //   ------------------------------------------------*/
    //   SxCR::EN::set( stream, 0 );
    //   LIFCR::setStreamMap( parent, streamRegisterIndex );
    //   HIFCR::setStreamMap( parent, streamRegisterIndex );

    //   /*------------------------------------------------
    //   Step 2: Set the transfer address registers
    //   See table 30 in RM0390
    //   ------------------------------------------------*/
    //   if ( ( config->Direction == Configuration::Direction::P2M ) ||
    //        ( config->Direction == Configuration::Direction::M2M ) )
    //   {
    //     SxPAR::set( stream, cb->srcAddress );
    //     SxM0AR::set( stream, cb->dstAddress );
    //   }
    //   else if ( config->Direction == Configuration::Direction::M2P )
    //   {
    //     SxM0AR::set( stream, cb->srcAddress );
    //     SxPAR::set( stream, cb->dstAddress );
    //   }
    //   else
    //   {
    //     return Chimera::Status::FAIL;
    //   }

    //   /*------------------------------------------------
    //   Step 3: Set how many bytes are to be transfered
    //   ------------------------------------------------*/
    //   SxNDTR::set( stream, cb->transferSize );

    //   /*------------------------------------------------
    //   Step 4: Select the DMA channel request
    //   ------------------------------------------------*/
    //   SxCR::CHSEL::set( stream, config->Channel );

    //   /*------------------------------------------------
    //   Step 5: Set the flow controller options
    //   ------------------------------------------------*/
    //   SxCR::PFCTRL::set( stream, 0 );
    //   SxCR::CIRC::set( stream, 0 );

    //   if ( config->Mode == Configuration::Mode::Periph )
    //   {
    //     SxCR::PFCTRL::set( stream, config->Mode );
    //   }
    //   else if ( config->Mode == Configuration::Mode::Circular )
    //   {
    //     SxCR::CIRC::set( stream, config->Mode );
    //   }
    //   // else normal DMA peripheral is the flow controller

    //   /*------------------------------------------------
    //   Step 6: Set the transfer priority
    //   ------------------------------------------------*/
    //   SxCR::PL::set( stream, config->Priority );

    //   /*------------------------------------------------
    //   Step 7: Set the FIFO usage
    //   ------------------------------------------------*/
    //   SxFCR::DMDIS::set( stream, config->FIFOMode );
    //   SxFCR::FTH::set( stream, config->FIFOThreshold );

    //   /*------------------------------------------------
    //   Step 8: Set additional misc settings
    //   ------------------------------------------------*/
    //   SxCR::DIR::set( stream, config->Direction );

    //   SxCR::MINC::set( stream, config->MemInc );
    //   SxCR::MBURST::set( stream, config->MemBurst );
    //   SxCR::MSIZE::set( stream, config->MemDataAlignment );

    //   SxCR::PINC::set( stream, config->PeriphInc );
    //   SxCR::PBURST::set( stream, config->PeriphBurst );
    //   SxCR::PSIZE::set( stream, config->PeriphDataAlignment );

    //   controlBlock.transferState = Runtime::Flag::TRANSFER_READY;
    //   exitCriticalSection();
    // }

    return result;
  }


  Chimera::Status_t Stream::start()
  {
    using namespace Chimera::Thread;

    auto result = Chimera::Status::LOCKED;

    // if ( try_lock_for( 100 ) )
    // {
    //   result = Chimera::Status::OK;

    //   /*------------------------------------------------
    //   Set up the interrupt bits
    //   ------------------------------------------------*/
    //   enterCriticalSection();
    //   enableTransferIRQ();

    //   /*------------------------------------------------
    //   Initialize the transfer control block
    //   ------------------------------------------------*/
    //   controlBlock.transferState = Runtime::Flag::TRANSFER_IN_PROGRESS;

    //   /*------------------------------------------------
    //   Enable stream, which starts the transfer if already configured
    //   ------------------------------------------------*/
    //   exitCriticalSection();
    //   SxCR::EN::set( stream, SxCR_EN );
    // }

    return result;
  }


  Chimera::Status_t Stream::abort()
  {
    // Should abruptly disable the hardware and fire an ISR if there is an ongoing transfer.
    //SxCR::EN::set( stream, 0 );
    return Chimera::Status::OK;
  }


  void Stream::IRQHandler( const uint8_t channel )
  {
    /* Indicates the split between the LISR and HISR registers */
    static constexpr uint32_t LOW_HIGH_REGISTER_STREAM_BOUNDARY = 3u;

    /*------------------------------------------------
    Read the status registers and parse the flags for this stream
    ------------------------------------------------*/
    // uint32_t statusRegister = 0;

    // if ( streamRegisterIndex <= LOW_HIGH_REGISTER_STREAM_BOUNDARY )
    // {
    //   statusRegister = LISR::get( parent );
    // }
    // else
    // {
    //   statusRegister = HISR::get( parent );
    // }

    // bool TCIF  = statusRegister & DMAStream_TCIF[ streamResourceIndex ];
    // bool HTIF  = statusRegister & DMAStream_HTIF[ streamResourceIndex ];
    // bool TEIF  = statusRegister & DMAStream_TEIF[ streamResourceIndex ];
    // bool DMEIF = statusRegister & DMAStream_DMEIF[ streamResourceIndex ];
    // bool FEIF  = statusRegister & DMAStream_FEIF[ streamResourceIndex ];

    // /*------------------------------------------------
    // Read the control registers for config information
    // ------------------------------------------------*/
    // const bool TCEIE = SxCR::TCIE::get( stream );
    // const bool HTIE  = SxCR::HTIE::get( stream );
    // const bool TEIE  = SxCR::TEIE::get( stream );
    // const bool DMEIE = SxCR::DMEIE::get( stream );
    // const bool FEIE  = SxFCR::FEIE::get( stream );

    // /*------------------------------------------------
    // Initialize the control block
    // ------------------------------------------------*/
    // controlBlock.fifoError       = false;
    // controlBlock.transferError   = false;
    // controlBlock.directModeError = false;

    // /*------------------------------------------------
    // Transfer Complete
    // ------------------------------------------------*/
    // bool semaphoreGiven = false;

    // if ( TCIF && TCEIE )
    // {
    //   /*------------------------------------------------
    //   Update the control block with the event information. This
    //   is how the thread that gets woken up will know what occurred.
    //   ------------------------------------------------*/
    //   controlBlock.selectedChannel = channel;
    //   controlBlock.bytesTransfered = controlBlock.transferSize - SxNDTR::get( stream );
    //   controlBlock.transferState |= Runtime::Flag::TRANSFER_COMPLETE;

    //   /*------------------------------------------------
    //   Exit the ISR with no more interrupts and the class
    //   resources unlocked.
    //   ------------------------------------------------*/
    //   disableTransferIRQ();
    //   unlockFromISR();

    //   /*------------------------------------------------
    //   Wake up the stream event listener thread
    //   ------------------------------------------------*/
    //   if ( !semaphoreGiven && wakeupSignal )
    //   {
    //     wakeupSignal->releaseFromISR();
    //     semaphoreGiven = true;
    //   }
    // }

    // /*------------------------------------------------
    // Transfer Half-Complete
    // ------------------------------------------------*/
    // if ( HTIF && HTIE ) {}    // Currently not supported

    // /*------------------------------------------------
    // Transfer Errors
    // ------------------------------------------------*/
    // if ( ( TEIF && TEIE ) || ( DMEIF && DMEIE ) || ( FEIF && FEIE ) )
    // {
    //   /*------------------------------------------------
    //   Make sure the ISR can't fire indefinitely in case we didn't
    //   get here after a transfer completion.
    //   ------------------------------------------------*/
    //   disableTransferIRQ();

    //   /*------------------------------------------------
    //   Let whichever thread that handles the result of the
    //   transfer know if there were any errors.
    //   ------------------------------------------------*/
    //   controlBlock.fifoError       = FEIF;
    //   controlBlock.transferError   = TEIF;
    //   controlBlock.directModeError = DMEIF;

    //   /*------------------------------------------------
    //   Wake up the stream event listener thread
    //   ------------------------------------------------*/
    //   if ( !semaphoreGiven && wakeupSignal )
    //   {
    //     wakeupSignal->releaseFromISR();
    //   }
    // }
  }


  void Stream::disableInterrupts()
  {
    //Thor::LLD::INT::disableIRQ( streamIRQn );
  }


  void Stream::enableInterrupts()
  {
    //Thor::LLD::INT::enableIRQ( streamIRQn );
  }


}  // namespace Thor::LLD::DMA
