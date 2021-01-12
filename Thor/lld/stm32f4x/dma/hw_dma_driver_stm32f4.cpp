/********************************************************************************
 *  File Name:
 *    hw_dma_driver_stm32f4.cpp
 *
 *  Description:
 *    STM32F4 DMA Driver Implementation
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/event>
#include <Thor/hld/interrupt/hld_interrupt_definitions.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_driver.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_mapping.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_prj.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32f4x/nvic/hw_nvic_driver.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_DMA )

namespace Thor::LLD::DMA
{
  static std::array<StreamController *, NUM_DMA_STREAMS> streamObjects;


  bool streamIsOnController( RegisterMap *const controller, StreamX *const stream )
  {
    auto streamAddress = reinterpret_cast<std::uintptr_t>( stream );

    switch ( reinterpret_cast<std::uintptr_t>( controller ) )
    {
      case static_cast<std::uintptr_t>( DMA1_BASE_ADDR ):
        return ( ( streamAddress == DMA1_STREAM0_BASE_ADDR ) || ( streamAddress == DMA1_STREAM1_BASE_ADDR ) ||
                 ( streamAddress == DMA1_STREAM2_BASE_ADDR ) || ( streamAddress == DMA1_STREAM3_BASE_ADDR ) ||
                 ( streamAddress == DMA1_STREAM4_BASE_ADDR ) || ( streamAddress == DMA1_STREAM5_BASE_ADDR ) ||
                 ( streamAddress == DMA1_STREAM6_BASE_ADDR ) || ( streamAddress == DMA1_STREAM7_BASE_ADDR ) );
        break;

      case static_cast<std::uintptr_t>( DMA2_BASE_ADDR ):
        return ( ( streamAddress == DMA2_STREAM0_BASE_ADDR ) || ( streamAddress == DMA2_STREAM1_BASE_ADDR ) ||
                 ( streamAddress == DMA2_STREAM2_BASE_ADDR ) || ( streamAddress == DMA2_STREAM3_BASE_ADDR ) ||
                 ( streamAddress == DMA2_STREAM4_BASE_ADDR ) || ( streamAddress == DMA2_STREAM5_BASE_ADDR ) ||
                 ( streamAddress == DMA2_STREAM6_BASE_ADDR ) || ( streamAddress == DMA2_STREAM7_BASE_ADDR ) );
        break;

      default:
        return false;
        break;
    }
  }

  Chimera::Status_t initialize()
  {
    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/

    initializeMapping();

    return Chimera::Status::OK;
  }

  StreamController * getStreamController( const uint8_t resourceIndex )
  {
    if( ( resourceIndex < DMA_RESOURCE_INDEX_START) && ( resourceIndex > DMA_RESOURCE_INDEX_END ) )
    {
      return nullptr;
    }

    return streamObjects[ resourceIndex ];
  }

  StreamController::StreamController() :
      stream( nullptr ), parent( nullptr ), streamRegisterIndex( 0 ), streamResourceIndex( 0 ), wakeupSignal( nullptr ),
      listenerIDCount( 0 )
  {
  }

  StreamController::~StreamController()
  {
  }

  Chimera::Status_t StreamController::attach( StreamX *const peripheral, RegisterMap *const parent )
  {
    using namespace Chimera::Threading;

    auto result = Chimera::Status::LOCKED;

    if ( this->try_lock_for( 100 ) )
    {
      /*------------------------------------------------
      Initialize class level variables
      ------------------------------------------------*/
      stream              = peripheral;
      this->parent        = parent;
      streamRegisterIndex = StreamToRegisterIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
      streamResourceIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
      streamIRQn          = DMAStream_IRQn[ streamResourceIndex ];

      result = Chimera::Status::OK;
    }

    return result;
  }

  Chimera::Status_t StreamController::attachISRWakeup( Chimera::Threading::BinarySemaphore *const wakeup )
  {
    using namespace Chimera::Threading;

    auto result = Chimera::Status::LOCKED;

    if ( try_lock_for( 100 ) )
    {
      enterCriticalSection();

      wakeupSignal = wakeup;
      result       = Chimera::Status::OK;

      exitCriticalSection();
    }

    return result;
  }

  Chimera::Status_t StreamController::configure( StreamConfig *const config, TCB *const cb )
  {
    using namespace Chimera::Threading;

    auto result = Chimera::Status::LOCKED;

    if ( try_lock_for( 100 ) )
    {
      enterCriticalSection();
      result = Chimera::Status::OK;

      /*------------------------------------------------
      Initialize the control block
      ------------------------------------------------*/
      memcpy( &controlBlock, cb, sizeof( TCB ) );
      controlBlock.transferState = Runtime::Flag::TRANSFER_NOT_READY;

      /*------------------------------------------------
      Step 1: Check for enabled status and clear ISR flags
      ------------------------------------------------*/
      SxCR::EN::set( stream, 0 );
      LIFCR::setStreamX( parent, streamRegisterIndex );
      HIFCR::setStreamX( parent, streamRegisterIndex );

      /*------------------------------------------------
      Step 2: Set the transfer address registers
      See table 30 in RM0390
      ------------------------------------------------*/
      if ( ( config->Direction == Configuration::TransferDirection::P2M ) ||
           ( config->Direction == Configuration::TransferDirection::M2M ) )
      {
        SxPAR::set( stream, cb->srcAddress );
        SxM0AR::set( stream, cb->dstAddress );
      }
      else if ( config->Direction == Configuration::TransferDirection::M2P )
      {
        SxM0AR::set( stream, cb->srcAddress );
        SxPAR::set( stream, cb->dstAddress );
      }
      else
      {
        return Chimera::Status::FAIL;
      }

      /*------------------------------------------------
      Step 3: Set how many bytes are to be transfered
      ------------------------------------------------*/
      SxNDTR::set( stream, cb->transferSize );

      /*------------------------------------------------
      Step 4: Select the DMA channel request
      ------------------------------------------------*/
      SxCR::CHSEL::set( stream, config->Channel );

      /*------------------------------------------------
      Step 5: Set the flow controller options
      ------------------------------------------------*/
      SxCR::PFCTRL::set( stream, 0 );
      SxCR::CIRC::set( stream, 0 );

      if ( config->Mode == Configuration::Mode::Periph )
      {
        SxCR::PFCTRL::set( stream, config->Mode );
      }
      else if ( config->Mode == Configuration::Mode::Circular )
      {
        SxCR::CIRC::set( stream, config->Mode );
      }
      // else normal DMA peripheral is the flow controller

      /*------------------------------------------------
      Step 6: Set the transfer priority
      ------------------------------------------------*/
      SxCR::PL::set( stream, config->Priority );

      /*------------------------------------------------
      Step 7: Set the FIFO usage
      ------------------------------------------------*/
      SxFCR::DMDIS::set( stream, config->FIFOMode );
      SxFCR::FTH::set( stream, config->FIFOThreshold );

      /*------------------------------------------------
      Step 8: Set additional misc settings
      ------------------------------------------------*/
      SxCR::DIR::set( stream, config->Direction );

      SxCR::MINC::set( stream, config->MemInc );
      SxCR::MBURST::set( stream, config->MemBurst );
      SxCR::MSIZE::set( stream, config->MemDataAlignment );

      SxCR::PINC::set( stream, config->PeriphInc );
      SxCR::PBURST::set( stream, config->PeriphBurst );
      SxCR::PSIZE::set( stream, config->PeriphDataAlignment );

      controlBlock.transferState = Runtime::Flag::TRANSFER_READY;
      exitCriticalSection();
    }

    return result;
  }

  Chimera::Status_t StreamController::start()
  {
    using namespace Chimera::Threading;

    auto result = Chimera::Status::LOCKED;

    if ( try_lock_for( 100 ) )
    {
      result = Chimera::Status::OK;

      /*------------------------------------------------
      Set up the interrupt bits
      ------------------------------------------------*/
      enterCriticalSection();
      enableTransferIRQ();

      /*------------------------------------------------
      Initialize the transfer control block
      ------------------------------------------------*/
      controlBlock.transferState = Runtime::Flag::TRANSFER_IN_PROGRESS;

      /*------------------------------------------------
      Enable stream, which starts the transfer if already configured
      ------------------------------------------------*/
      exitCriticalSection();
      SxCR::EN::set( stream, SxCR_EN );
    }

    return result;
  }

  Chimera::Status_t StreamController::abort()
  {
    // Should abruptly disable the hardware and fire an ISR if there is an ongoing transfer.
    SxCR::EN::set( stream, 0 );
    return Chimera::Status::OK;
  }

  Chimera::Status_t StreamController::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    registrationID = ++listenerIDCount;
    listener.id    = registrationID;

    eventListeners.push_back( listener );

    return Chimera::Status::OK;
  }

  Chimera::Status_t StreamController::removeListener( const size_t registrationID, const size_t timeout )
  {
    for ( auto iterator = eventListeners.begin(); iterator != eventListeners.end(); iterator++ )
    {
      if ( iterator->id == registrationID )
      {
        eventListeners.erase( iterator );
        return Chimera::Status::OK;
        break;
      }
    }

    return Chimera::Status::NOT_FOUND;
  }

  Chimera::Status_t StreamController::postISRProcessing()
  {
    using namespace Chimera::Threading;

    auto result = Chimera::Status::OK;

    TCB cb;
    cb.clear();

    /*------------------------------------------------
    Make sure we gain exclusive access to the control block
    ------------------------------------------------*/
    if ( try_lock_for( 100 ) )
    {
      enterCriticalSection();
      cb = controlBlock;
      exitCriticalSection();
    }
    else
    {
      return Chimera::Status::LOCKED;
    }

    /*------------------------------------------------
    Transfer Complete
    ------------------------------------------------*/
    if ( cb.transferState & Runtime::Flag::TRANSFER_COMPLETE )
    {
      processListeners( Chimera::Event::TRIGGER_TRANSFER_COMPLETE );
    }

    /*------------------------------------------------
    Transfer Errors
    ------------------------------------------------*/
    if ( cb.transferState & ( Runtime::Flag::TRANSFER_ERROR | Runtime::Flag::DIRECT_MODE_ERROR | Runtime::Flag::FIFO_ERROR ) )
    {
      processListeners( Chimera::Event::TRIGGER_SYSTEM_ERROR );
    }

    return result;
  }

  void StreamController::IRQHandler( const uint8_t channel )
  {
    /* Indicates the split between the LISR and HISR registers */
    static constexpr uint32_t LOW_HIGH_REGISTER_STREAM_BOUNDARY = 3u;

    /*------------------------------------------------
    Read the status registers and parse the flags for this stream
    ------------------------------------------------*/
    uint32_t statusRegister = 0;

    if ( streamRegisterIndex <= LOW_HIGH_REGISTER_STREAM_BOUNDARY )
    {
      statusRegister = LISR::get( parent );
    }
    else
    {
      statusRegister = HISR::get( parent );
    }

    bool TCIF  = statusRegister & DMAStream_TCIF[ streamResourceIndex ];
    bool HTIF  = statusRegister & DMAStream_HTIF[ streamResourceIndex ];
    bool TEIF  = statusRegister & DMAStream_TEIF[ streamResourceIndex ];
    bool DMEIF = statusRegister & DMAStream_DMEIF[ streamResourceIndex ];
    bool FEIF  = statusRegister & DMAStream_FEIF[ streamResourceIndex ];

    /*------------------------------------------------
    Read the control registers for config information
    ------------------------------------------------*/
    const bool TCEIE = SxCR::TCIE::get( stream );
    const bool HTIE  = SxCR::HTIE::get( stream );
    const bool TEIE  = SxCR::TEIE::get( stream );
    const bool DMEIE = SxCR::DMEIE::get( stream );
    const bool FEIE  = SxFCR::FEIE::get( stream );

    /*------------------------------------------------
    Initialize the control block
    ------------------------------------------------*/
    controlBlock.fifoError       = false;
    controlBlock.transferError   = false;
    controlBlock.directModeError = false;

    /*------------------------------------------------
    Transfer Complete
    ------------------------------------------------*/
    bool semaphoreGiven = false;

    if ( TCIF && TCEIE )
    {
      /*------------------------------------------------
      Update the control block with the event information. This
      is how the thread that gets woken up will know what occurred.
      ------------------------------------------------*/
      controlBlock.selectedChannel = channel;
      controlBlock.bytesTransfered = controlBlock.transferSize - SxNDTR::get( stream );
      controlBlock.transferState |= Runtime::Flag::TRANSFER_COMPLETE;

      /*------------------------------------------------
      Exit the ISR with no more interrupts and the class
      resources unlocked.
      ------------------------------------------------*/
      disableTransferIRQ();
      unlockFromISR();

      /*------------------------------------------------
      Wake up the stream event listener thread
      ------------------------------------------------*/
      if ( !semaphoreGiven && wakeupSignal )
      {
        wakeupSignal->releaseFromISR();
        semaphoreGiven = true;
      }
    }

    /*------------------------------------------------
    Transfer Half-Complete
    ------------------------------------------------*/
    if ( HTIF && HTIE ) {}    // Currently not supported

    /*------------------------------------------------
    Transfer Errors
    ------------------------------------------------*/
    if ( ( TEIF && TEIE ) || ( DMEIF && DMEIE ) || ( FEIF && FEIE ) )
    {
      /*------------------------------------------------
      Make sure the ISR can't fire indefinitely in case we didn't
      get here after a transfer completion.
      ------------------------------------------------*/
      disableTransferIRQ();

      /*------------------------------------------------
      Let whichever thread that handles the result of the
      transfer know if there were any errors.
      ------------------------------------------------*/
      controlBlock.fifoError       = FEIF;
      controlBlock.transferError   = TEIF;
      controlBlock.directModeError = DMEIF;

      /*------------------------------------------------
      Wake up the stream event listener thread
      ------------------------------------------------*/
      if ( !semaphoreGiven && wakeupSignal )
      {
        wakeupSignal->releaseFromISR();
      }
    }
  }

  void StreamController::enableTransferIRQ()
  {
    /*------------------------------------------------
    Make sure the interrupt priority has been set to correctly
    ------------------------------------------------*/
    Thor::LLD::IT::setPriority( streamIRQn, Thor::Interrupt::DMA_STREAM_PREEMPT_PRIORITY, 0 );

    /*------------------------------------------------
    - Transfer complete
    - Transfer error
    - Direct mode error
    - FIFO error
    ------------------------------------------------*/
    SxCR::TCIE::set( stream, SxCR_TCIE );
    SxCR::TEIE::set( stream, SxCR_TEIE );
    SxCR::DMEIE::set( stream, SxCR_DMEIE );
    SxFCR::FEIE::set( stream, SxFCR_FEIE );
  }

  void StreamController::disableTransferIRQ()
  {
    /*------------------------------------------------
    - Transfer complete
    - Transfer error
    - Direct mode error
    - FIFO error
    ------------------------------------------------*/
    SxCR::TCIE::set( stream, 0 );
    SxCR::TEIE::set( stream, 0 );
    SxCR::DMEIE::set( stream, 0 );
    SxFCR::FEIE::set( stream, 0 );
  }

  void StreamController::enterCriticalSection()
  {
    Thor::LLD::IT::disableIRQ( streamIRQn );
  }

  void StreamController::exitCriticalSection()
  {
    Thor::LLD::IT::enableIRQ( streamIRQn );
  }

  void StreamController::processListeners( const Chimera::Event::Trigger event )
  {
    Chimera::Event::notifyListenerList( event, eventListeners, 0 );
  }

  /*------------------------------------------------
  Driver Implementation
  ------------------------------------------------*/
  ChannelController::ChannelController() : periph( nullptr )
  {
  }

  ChannelController::~ChannelController()
  {
  }

  Chimera::Status_t ChannelController::attach( RegisterMap *const peripheral )
  {
    periph = peripheral;
    clockEnable();
    return Chimera::Status::OK;
  }

  Chimera::Status_t ChannelController::clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::getPeripheralClock();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::Status::OK;
  }

  Chimera::Status_t ChannelController::clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::getPeripheralClock();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::Status::OK;
  }

  Chimera::Status_t ChannelController::reset()
  {
    auto rcc   = Thor::LLD::RCC::getPeripheralClock();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->reset( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::Status::OK;
  }

  Chimera::Status_t ChannelController::init()
  {
    if ( !periph )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Reset the peripheral back to its default conditions
    ------------------------------------------------*/
    clockEnable();
    reset();
    clockEnable();

    /*------------------------------------------------
    Initialize all the stream objects for the DMA
    peripherals if one hasn't been created yet.
    ------------------------------------------------*/
    static_assert( DMA_RESOURCE_INDEX_START == 0, "DMA1 resource index invalid" );

    if ( periph == DMA1_PERIPH )
    {
      for ( uint8_t x = DMA_RESOURCE_INDEX_START; x < DMA1_RESOURCE_INDEX_END; x++ )
      {
        if ( !streamObjects[ x ] )
        {
          /* x is already zero indexed, no need to convert it to get the proper stream */
          StreamX *streamInstance = getStreamRegisters( periph, x );
          streamObjects[ x ]      = new StreamController();
          streamObjects[ x ]->attach( streamInstance, periph );
        }
      }
    }
    else if ( periph == DMA2_PERIPH )
    {
      for ( uint8_t x = DMA2_RESOURCE_INDEX_START; x < DMA2_RESOURCE_INDEX_END; x++ )
      {
        if ( !streamObjects[ x ] )
        {
          /* Need to convert back to zero index to get the stream properly */
          auto streamInstance = getStreamRegisters( periph, ( x - DMA2_RESOURCE_INDEX_START ) );
          streamObjects[ x ]  = new StreamController();
          streamObjects[ x ]->attach( streamInstance, periph );
        }
      }
    }

    return Chimera::Status::OK;
  }

  Chimera::Status_t ChannelController::configure( StreamX *const stream, StreamConfig *const config, TCB *const controlBlock )
  {
    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( streamObjects[ streamIndex ] )
    {
      return streamObjects[ streamIndex ]->configure( config, controlBlock );
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }

  Chimera::Status_t ChannelController::start( StreamX *const stream )
  {
    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( streamObjects[ streamIndex ] )
    {
      return streamObjects[ streamIndex ]->start();
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }

  Chimera::Status_t ChannelController::abort( StreamX *const stream )
  {
    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( streamObjects[ streamIndex ] )
    {
      return streamObjects[ streamIndex ]->abort();
    }
    else
    {
      return Chimera::Status::FAIL;
    }
  }

  Chimera::Status_t ChannelController::registerListener( StreamX *const stream, Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    auto result          = Chimera::Status::FAIL;
    size_t resourceIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( ( resourceIndex < streamObjects.size() ) && streamObjects[ resourceIndex ] )
    {
      result = streamObjects[ resourceIndex ]->registerListener( listener, timeout, registrationID );
    }

    return result;
  }

  Chimera::Status_t ChannelController::removeListener( StreamX *const stream, const size_t registrationID, const size_t timeout )
  {
    auto result          = Chimera::Status::FAIL;
    size_t resourceIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( ( resourceIndex < streamObjects.size() ) && streamObjects[ resourceIndex ] )
    {
      result = streamObjects[ resourceIndex ]->removeListener( registrationID, timeout );
    }

    return result;
  }

}    // namespace Thor::LLD::DMA


using namespace Thor::DMA;
using namespace Thor::LLD::DMA;

void DMA1_Stream0_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA1_STREAM0_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA1_STREAM0 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA1_Stream1_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA1_STREAM1_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA1_STREAM1 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA1_Stream2_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA1_STREAM2_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA1_STREAM2 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA1_Stream3_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA1_STREAM3_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA1_STREAM3 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA1_Stream4_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA1_STREAM4_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA1_STREAM4 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA1_Stream5_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA1_STREAM5_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA1_STREAM5 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}
void DMA1_Stream6_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA1_STREAM6_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA1_STREAM6 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA1_Stream7_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA1_STREAM7_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA1_STREAM7 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA2_Stream0_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA2_STREAM0_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA2_STREAM0 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA2_Stream1_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA2_STREAM1_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA2_STREAM1 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA2_Stream2_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA2_STREAM2_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA2_STREAM2 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA2_Stream3_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA2_STREAM3_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA2_STREAM3 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA2_Stream4_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA2_STREAM4_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA2_STREAM4 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA2_Stream5_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA2_STREAM5_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA2_STREAM5 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA2_Stream6_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA2_STREAM6_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA2_STREAM6 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

void DMA2_Stream7_IRQHandler( void )
{
  constexpr auto resourceIndex = DMA2_STREAM7_RESOURCE_INDEX;
  const uint8_t channel        = SxCR::CHSEL::get( DMA2_STREAM7 ) >> SxCR_CHSEL_Pos;

  if ( streamObjects[ resourceIndex ] )
  {
    streamObjects[ resourceIndex ]->IRQHandler( channel );
  }
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */