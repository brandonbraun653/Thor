/********************************************************************************
 *  File Name:
 *    hw_dma_driver_stm32f4.cpp
 *
 *  Description:
 *    STM32F4 DMA Driver Implementation
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/lld/stm32f4x/dma/hw_dma_driver.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_mapping.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_prj.hpp>
#include <Thor/lld/stm32f4x/dma/hw_dma_types.hpp>
#include <Thor/lld/stm32f4x/rcc/hw_rcc_driver.hpp>
#include <Thor/lld/stm32f4x/nvic/hw_nvic_driver.hpp>

namespace DMADriver = Thor::LLD::DMA;

#if defined( TARGET_STM32F4 ) && ( THOR_LLD_DMA )

static std::array<Thor::LLD::DMA::Stream *, Thor::LLD::DMA::NUM_DMA_STREAMS> streamObjects;

static void DMA1_Stream0_ISRPostProcessorThread( void *argument );
static void DMA1_Stream1_ISRPostProcessorThread( void *argument );
static void DMA1_Stream2_ISRPostProcessorThread( void *argument );
static void DMA1_Stream3_ISRPostProcessorThread( void *argument );
static void DMA1_Stream4_ISRPostProcessorThread( void *argument );
static void DMA1_Stream5_ISRPostProcessorThread( void *argument );
static void DMA1_Stream6_ISRPostProcessorThread( void *argument );
static void DMA1_Stream7_ISRPostProcessorThread( void *argument );
static void DMA2_Stream0_ISRPostProcessorThread( void *argument );
static void DMA2_Stream1_ISRPostProcessorThread( void *argument );
static void DMA2_Stream2_ISRPostProcessorThread( void *argument );
static void DMA2_Stream3_ISRPostProcessorThread( void *argument );
static void DMA2_Stream4_ISRPostProcessorThread( void *argument );
static void DMA2_Stream5_ISRPostProcessorThread( void *argument );
static void DMA2_Stream6_ISRPostProcessorThread( void *argument );
static void DMA2_Stream7_ISRPostProcessorThread( void *argument );

/*------------------------------------------------
Static Data
------------------------------------------------*/
/* clang-format off */
/* Post Processor Thread Handles */
static std::array<TaskHandle_t, DMADriver::NUM_DMA_STREAMS> postProcessorHandle = { 
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,
  nullptr
};

/* Post Processor Thread Wakeup Signals */
static std::array<SemaphoreHandle_t, DMADriver::NUM_DMA_STREAMS> postProcessorSignal = { 
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,  
  nullptr,
  nullptr,
  nullptr
};

/* Post Processor Thread Function Pointers */
static std::array<Chimera::Function::void_func_void_ptr, DMADriver::NUM_DMA_STREAMS> postProcessorThread = {
  DMA1_Stream0_ISRPostProcessorThread,
  DMA1_Stream1_ISRPostProcessorThread,
  DMA1_Stream2_ISRPostProcessorThread,
  DMA1_Stream3_ISRPostProcessorThread,
  DMA1_Stream4_ISRPostProcessorThread,
  DMA1_Stream5_ISRPostProcessorThread,
  DMA1_Stream6_ISRPostProcessorThread,
  DMA1_Stream7_ISRPostProcessorThread,
  DMA2_Stream0_ISRPostProcessorThread,
  DMA2_Stream1_ISRPostProcessorThread,
  DMA2_Stream2_ISRPostProcessorThread,
  DMA2_Stream3_ISRPostProcessorThread,
  DMA2_Stream4_ISRPostProcessorThread,
  DMA2_Stream5_ISRPostProcessorThread,
  DMA2_Stream6_ISRPostProcessorThread,
  DMA2_Stream7_ISRPostProcessorThread
};
/* clang-format on */

namespace Thor::LLD::DMA
{
  bool streamIsOnPeripheral( RegisterMap *const controller, StreamX *const stream )
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

  void initialize()
  {
    initializeRegisters();
    initializeMapping();
  }


  Stream::Stream() :
      stream( nullptr ), parent( nullptr ), streamRegisterIndex( 0 ), streamResourceIndex( 0 ), wakeupSignal( nullptr ),
      listenerIDCount( 0 )
  {
  }

  Stream::~Stream()
  {
  }

  Chimera::Status_t Stream::attach( StreamX *const peripheral, RegisterMap *const parent )
  {
    using namespace Chimera::Threading;

    auto result = Chimera::CommonStatusCodes::LOCKED;

    if ( try_lock_for( 100 ) )
    {
      /*------------------------------------------------
      Initialize class level variables
      ------------------------------------------------*/
      stream              = peripheral;
      this->parent        = parent;
      streamRegisterIndex = StreamToRegisterIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
      streamResourceIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
      streamIRQn          = DMAStream_IRQn[ streamResourceIndex ];

      /*------------------------------------------------
      Initialize ISR handler thread resources
      ------------------------------------------------*/
      postProcessorSignal[ streamResourceIndex ] = xSemaphoreCreateBinary();
      postProcessorHandle[ streamResourceIndex ] = new TaskHandle_t;

      wakeupSignal = postProcessorSignal[ streamResourceIndex ];

      Chimera::Threading::Thread thread;
      thread.initialize( postProcessorThread[ streamResourceIndex ], nullptr, Chimera::Threading::Priority::LEVEL_5, 500, "" );
      thread.start();
      postProcessorHandle[ streamResourceIndex ] = thread.native_handle();

      result = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  Chimera::Status_t Stream::attachISRWakeup( SemaphoreHandle_t wakeup )
  {
    using namespace Chimera::Threading;

    auto result = Chimera::CommonStatusCodes::LOCKED;

    if ( try_lock_for( 100 ) )
    {
      enterCriticalSection();

      wakeupSignal = wakeup;
      result       = Chimera::CommonStatusCodes::OK;

      exitCriticalSection();
    }

    return result;
  }

  Chimera::Status_t Stream::configure( StreamConfig *const config, TCB *const cb )
  {
    using namespace Chimera::Threading;

    auto result = Chimera::CommonStatusCodes::LOCKED;

    if ( try_lock_for( 100 ) )
    {
      enterCriticalSection();
      result = Chimera::CommonStatusCodes::OK;

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
        return Chimera::CommonStatusCodes::FAIL;
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

  Chimera::Status_t Stream::start()
  {
    using namespace Chimera::Threading;

    auto result = Chimera::CommonStatusCodes::LOCKED;

    if ( try_lock_for( 100 ) )
    {
      result = Chimera::CommonStatusCodes::OK;

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

  Chimera::Status_t Stream::abort()
  {
    // Should abruptly disable the hardware and fire an ISR if there is an ongoing transfer.
    SxCR::EN::set( stream, 0 );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Stream::registerListener( Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    registrationID = ++listenerIDCount;
    listener.id    = registrationID;

    eventListeners.push_back( listener );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Stream::removeListener( const size_t registrationID, const size_t timeout )
  {
    for ( auto iterator = eventListeners.begin(); iterator != eventListeners.end(); iterator++ )
    {
      if ( iterator->id == registrationID )
      {
        eventListeners.erase( iterator );
        return Chimera::CommonStatusCodes::OK;
        break;
      }
    }

    return Chimera::CommonStatusCodes::NOT_FOUND;
  }

  Chimera::Status_t Stream::postISRProcessing()
  {
    using namespace Chimera::Threading;

    auto result = Chimera::CommonStatusCodes::OK;

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
      return Chimera::CommonStatusCodes::LOCKED;
    }

    /*------------------------------------------------
    Transfer Complete
    ------------------------------------------------*/
    if ( cb.transferState & Runtime::Flag::TRANSFER_COMPLETE ) 
    {
      processListeners( Chimera::Event::Trigger::TRANSFER_COMPLETE );
    }

    /*------------------------------------------------
    Transfer Errors
    ------------------------------------------------*/
    if ( cb.transferState & ( Runtime::Flag::TRANSFER_ERROR | Runtime::Flag::DIRECT_MODE_ERROR | Runtime::Flag::FIFO_ERROR ) )
    {
      processListeners( Chimera::Event::Trigger::SYSTEM_ERROR );
    }

    return result;
  }

  void Stream::IRQHandler( const uint8_t channel )
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
        xSemaphoreGiveFromISR( wakeupSignal, nullptr );
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
        xSemaphoreGiveFromISR( wakeupSignal, nullptr );
      }
    }
  }

  void Stream::enableTransferIRQ()
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

  void Stream::disableTransferIRQ()
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

  void Stream::enterCriticalSection()
  {
    Thor::LLD::IT::disableIRQ( streamIRQn );
  }

  void Stream::exitCriticalSection()
  {
    Thor::LLD::IT::enableIRQ( streamIRQn );
  }

  void Stream::processListeners( const Chimera::Event::Trigger event )
  {
    for ( auto &listener : eventListeners )
    {
      if ( listener.trigger != event )
      {
        continue;
      }

      Thor::Event::notifyAtomic( event, listener, static_cast<uint32_t>( event ) );
      Thor::Event::notifyThread( event, listener );
      Thor::Event::executeISRCallback( event, listener, nullptr, 0 );
    }
  }

  /*------------------------------------------------
  Driver Implementation
  ------------------------------------------------*/
  Driver::Driver() : periph( nullptr )
  {
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    periph = peripheral;
    clockEnable();
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::PeripheralController::get();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::PeripheralController::get();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::reset()
  {
    auto rcc   = Thor::LLD::RCC::PeripheralController::get();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->reset( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::init()
  {
    if ( !periph )
    {
      return Chimera::CommonStatusCodes::NOT_INITIALIZED;
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
    static_assert( DMA1_RESOURCE_INDEX_START == 0, "DMA1 resource index invalid" );

    if ( periph == DMA1_PERIPH )
    {
      for ( uint8_t x = DMA1_RESOURCE_INDEX_START; x < DMA1_RESOURCE_INDEX_END; x++ )
      {
        if ( !streamObjects[ x ] )
        {
          /* x is already zero indexed, no need to convert it to get the proper stream */
          StreamX * streamInstance = getStreamRegisters( periph, x );
          streamObjects[ x ]  = new Stream();
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
          streamObjects[ x ]  = new Stream();
          streamObjects[ x ]->attach( streamInstance, periph );
        }
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::configure( StreamX *const stream, StreamConfig *const config, TCB *const controlBlock )
  {
    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( streamObjects[ streamIndex ] )
    {
      return streamObjects[ streamIndex ]->configure( config, controlBlock );
    }
    else
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
  }

  Chimera::Status_t Driver::start( StreamX *const stream )
  {
    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( streamObjects[ streamIndex ] )
    {
      return streamObjects[ streamIndex ]->start();
    }
    else
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
  }

  Chimera::Status_t Driver::abort( StreamX *const stream )
  {
    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( streamObjects[ streamIndex ] )
    {
      return streamObjects[ streamIndex ]->abort();
    }
    else
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
  }

  Chimera::Status_t Driver::registerListener( StreamX *const stream, Chimera::Event::Actionable &listener, const size_t timeout,
                                              size_t &registrationID )
  {
    auto result          = Chimera::CommonStatusCodes::FAIL;
    size_t resourceIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

    if ( ( resourceIndex < streamObjects.size() ) && streamObjects[ resourceIndex ] )
    {
      result = streamObjects[ resourceIndex ]->registerListener( listener, timeout, registrationID );
    }

    return result;
  }

  Chimera::Status_t Driver::removeListener( StreamX *const stream, const size_t registrationID, const size_t timeout )
  {
    auto result = Chimera::CommonStatusCodes::FAIL;
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

static void DMA1_Stream0_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM0_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA1_Stream1_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM1_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA1_Stream2_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM2_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA1_Stream3_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM3_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA1_Stream4_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM4_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA1_Stream5_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM5_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA1_Stream6_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM6_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA1_Stream7_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA1_STREAM7_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA2_Stream0_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM0_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA2_Stream1_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM1_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA2_Stream2_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM2_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA2_Stream3_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM3_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA2_Stream4_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM4_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA2_Stream5_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM5_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA2_Stream6_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM6_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
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

static void DMA2_Stream7_ISRPostProcessorThread( void *argument )
{
  using namespace Thor::LLD::DMA;
  constexpr auto resourceIndex = DMA2_STREAM7_RESOURCE_INDEX;

  

  while ( 1 )
  {
    /*------------------------------------------------
    Wait for the ISR to wake up this thread before doing any processing
    ------------------------------------------------*/
    if ( xSemaphoreTake( postProcessorSignal[ resourceIndex ], portMAX_DELAY ) == pdPASS )
    {
      if ( auto stream = streamObjects[ resourceIndex ]; stream )
      {
        stream->postISRProcessing();
      }
    }
  }
}

#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */