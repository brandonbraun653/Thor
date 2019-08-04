/********************************************************************************
 *   File Name:
 *    hw_dma_mapping.hpp
 *
 *   Description:
 *    STM32 Mappings for the DMA Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Thor Includes */
#include <Thor/resources/dma_resources.hpp>
#include <Chimera/types/peripheral_types.hpp>


/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/definitions/interrupt_definitions.hpp>
#include <Thor/drivers/f4/dma/hw_dma_driver.hpp>
#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>
#include <Thor/drivers/f4/dma/hw_dma_prj.hpp>
#include <Thor/drivers/f4/dma/hw_dma_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>
#include <Thor/drivers/f4/nvic/hw_nvic_driver.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

static std::array<Thor::Driver::DMA::Internal::Stream *, Thor::Driver::DMA::NUM_DMA_STREAMS_PER_PERIPH> dma1Streams;
static std::array<Thor::Driver::DMA::Internal::Stream *, Thor::Driver::DMA::NUM_DMA_STREAMS_PER_PERIPH> dma2Streams;

namespace Thor::Driver::DMA
{
  namespace Internal
  {
    Stream::Stream() :
        stream( nullptr ), parent( nullptr ), streamRegisterIndex( 0 ), streamResourceIndex( 0 ), wakeupSignal( nullptr )
    {
    }

    Stream::~Stream()
    {
    }

    Chimera::Status_t Stream::attach( StreamX *const peripheral, RegisterMap *const parent )
    {
      using namespace Chimera::Threading;

      auto result = Chimera::CommonStatusCodes::LOCKED;

      if ( LockGuard( *this ).lock() )
      {
        stream              = peripheral;
        this->parent        = parent;
        streamRegisterIndex = StreamToRegisterIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
        streamResourceIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
        streamIRQn          = DMAStream_IRQn[ streamResourceIndex ];

        result = Chimera::CommonStatusCodes::OK;
      }

      return result;
    }

    Chimera::Status_t Stream::attachISRWakeup( SemaphoreHandle_t wakeup )
    {
      using namespace Chimera::Threading;

      auto result = Chimera::CommonStatusCodes::LOCKED;

      if ( LockGuard( *this ).lock() )
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

      if ( LockGuard( *this ).lock() )
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
          SxM1AR::set( stream, cb->dstAddress );
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
        // Nothing to do at the moment

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

      if ( lock( TIMEOUT_DONT_WAIT ) )
      {
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
    }

    Chimera::Status_t Stream::abort()
    {
      // Should abruptly disable the hardware and fire an ISR if there is an ongoing transfer.
      SxCR::EN::set( stream, 0 );
      return Chimera::CommonStatusCodes::OK;
    }

    void Stream::IRQHandler( const uint8_t channel, const Thor::DMA::Source_t request )
    {
      /* Indicates the split between the LISR and HISR registers */
      static constexpr uint32_t LOW_HIGH_REGISTER_STREAM_BOUNDARY = 3u;

      /*------------------------------------------------
      Read the status registers and parse the flags for this stream
      ------------------------------------------------*/
      uint32_t statusRegister = 0;

      if ( streamRegisterIndex > LOW_HIGH_REGISTER_STREAM_BOUNDARY )
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
        controlBlock.selectedChannel  = channel;
        controlBlock.requestGenerator = request;
        controlBlock.bytesTransfered  = controlBlock.transferSize - SxNDTR::get( stream );

        /*------------------------------------------------
        Exit the ISR with
        ------------------------------------------------*/
        disableTransferIRQ();
        xSemaphoreGiveFromISR( wakeupSignal, nullptr );
        semaphoreGiven = true;
        unlock();
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
        if ( !semaphoreGiven )
        {
          xSemaphoreGiveFromISR( wakeupSignal, nullptr );
        }
      }
    }

    void Stream::enableTransferIRQ()
    {
      using namespace Thor::Driver::Interrupt;

      setPriority( streamIRQn, Thor::Interrupt::DMA_STREAM_PREEMPT_PRIORITY, 0 );

      SxCR::TCIE::set( stream, SxCR_TCIE );
      SxCR::TEIE::set( stream, SxCR_TEIE );
      SxCR::DMEIE::set( stream, SxCR_DMEIE );
    }

    void Stream::disableTransferIRQ()
    {
      SxCR::TCIE::set( stream, 0 );
      SxCR::TEIE::set( stream, 0 );
      SxCR::DMEIE::set( stream, 0 );
    }

    void Stream::enterCriticalSection()
    {
      Thor::Driver::Interrupt::disableIRQ( streamIRQn );
    }

    void Stream::exitCriticalSection()
    {
      Thor::Driver::Interrupt::enableIRQ( streamIRQn );
    }
  }    // namespace Internal


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
    auto rcc   = Thor::Driver::RCC::PeripheralController::get();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::clockDisable()
  {
    auto rcc   = Thor::Driver::RCC::PeripheralController::get();
    auto index = InstanceToResourceIndex.find( reinterpret_cast<std::uintptr_t>( periph ) )->second;

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::reset()
  {
    auto rcc   = Thor::Driver::RCC::PeripheralController::get();
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
    Initialize all the stream objects for this DMA peripheral
    ------------------------------------------------*/
    auto tmp = &dma1Streams;
    if ( periph == DMA2_PERIPH )
    {
      tmp = &dma2Streams;
    }

    for ( uint8_t x = 0; x < (*tmp).size(); x++ )
    {
      if ( !(*tmp)[ x ] )
      {
        auto streamInstance = getStream( periph, x );
        (*tmp)[ x ] = new Internal::Stream();
        (*tmp)[ x ]->attach( streamInstance, periph );
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::configure( StreamX *const stream, StreamConfig *const config, TCB *const controlBlock )
  {
    // TODO: Optimize this

    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
    auto streamArray = dma1Streams;
    if ( periph == DMA2_PERIPH )
    {
      streamArray = dma2Streams;
    }

    if ( streamArray[ streamIndex ] ) 
    {
      return streamArray[ streamIndex ]->configure( config, controlBlock );
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t Driver::start( StreamX *const stream )
  {
    // TODO: Optimize this

    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
    auto streamArray = dma1Streams;
    if ( periph == DMA2_PERIPH )
    {
      streamArray = dma2Streams;
    }

    if ( streamArray[ streamIndex ] )
    {
      return streamArray[ streamIndex ]->start();
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

  Chimera::Status_t Driver::abort( StreamX *const stream )
  {
    // TODO: Optimize this

    auto streamIndex = StreamToResourceIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;
    auto streamArray = dma1Streams;
    if ( periph == DMA2_PERIPH )
    {
      streamArray = dma2Streams;
    }

    if ( streamArray[ streamIndex ] )
    {
      return streamArray[ streamIndex ]->abort();
    }

    return Chimera::CommonStatusCodes::FAIL;
  }

}    // namespace Thor::Driver::DMA


using namespace Thor::DMA;
using namespace Thor::Driver::DMA;

void DMA1_Stream0_IRQHandler( void )
{
  constexpr uint8_t stream          = 0u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA1_STREAM0 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream1_IRQHandler( void )
{
  constexpr uint8_t stream          = 1u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA1_STREAM1 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream2_IRQHandler( void )
{
  constexpr uint8_t stream          = 2u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA1_STREAM2 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream3_IRQHandler( void )
{
  constexpr uint8_t stream          = 3u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA1_STREAM3 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream4_IRQHandler( void )
{
  constexpr uint8_t stream          = 4u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA1_STREAM4 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream5_IRQHandler( void )
{
  constexpr uint8_t stream          = 5u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA1_STREAM5 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream6_IRQHandler( void )
{
  constexpr uint8_t stream          = 6u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA1_STREAM6 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream7_IRQHandler( void )
{
  constexpr uint8_t stream          = 7u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA1_STREAM7 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream0_IRQHandler( void )
{
  constexpr uint8_t stream          = 0u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA2_STREAM0 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream1_IRQHandler( void )
{
  constexpr uint8_t stream          = 1u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA2_STREAM1 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream2_IRQHandler( void )
{
  constexpr uint8_t stream          = 2u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA2_STREAM2 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream3_IRQHandler( void )
{
  constexpr uint8_t stream          = 3u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA2_STREAM3 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream4_IRQHandler( void )
{
  constexpr uint8_t stream          = 4u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA2_STREAM4 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream5_IRQHandler( void )
{
  constexpr uint8_t stream          = 5u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA2_STREAM5 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream6_IRQHandler( void )
{
  constexpr uint8_t stream          = 6u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA2_STREAM6 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream7_IRQHandler( void )
{
  constexpr uint8_t stream          = 7u;
  const uint8_t channel             = SxCR::CHSEL::get( DMA2_STREAM7 ) >> SxCR_CHSEL_Pos;
  const Thor::DMA::Source_t request = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}


#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */