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
#include <Thor/drivers/f4/dma/hw_dma_driver.hpp>
#include <Thor/drivers/f4/dma/hw_dma_mapping.hpp>
#include <Thor/drivers/f4/dma/hw_dma_prj.hpp>
#include <Thor/drivers/f4/dma/hw_dma_types.hpp>
#include <Thor/drivers/f4/rcc/hw_rcc_driver.hpp>


#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_DMA == 1 )

static std::array<Thor::Driver::DMA::Driver *, Thor::Driver::DMA::NUM_DMA_PERIPHS> dmaPeriphs;
static std::array<Thor::Driver::DMA::Internal::Stream *, Thor::Driver::DMA::NUM_DMA_STREAMS> dma1Streams;
static std::array<Thor::Driver::DMA::Internal::Stream *, Thor::Driver::DMA::NUM_DMA_STREAMS> dma2Streams;

namespace Thor::Driver::DMA
{
  namespace Internal
  {
    Stream::Stream() : stream( nullptr ), parent( nullptr ), streamIndex( 0 ), wakeupSignal( nullptr )
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
        stream       = peripheral;
        this->parent = parent;
        streamIndex  = StreamToRegisterIndex.find( reinterpret_cast<std::uintptr_t>( stream ) )->second;

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
        wakeupSignal = wakeup;
        result       = Chimera::CommonStatusCodes::OK;
      }

      return result;
    }

    Chimera::Status_t Stream::configure( StreamConfig *const config, TCB *const controlBlock )
    {
      using namespace Chimera::Threading;

      auto result = Chimera::CommonStatusCodes::LOCKED;

      if ( LockGuard( *this ).lock() )
      {
        this->controlBlock = controlBlock;

        /*------------------------------------------------
        Step 1: Check for enabled status and clear ISR flags
        ------------------------------------------------*/
        SxCR::EN::set( stream, 0 );
        LIFCR::setStreamX( parent, streamIndex );
        HIFCR::setStreamX( parent, streamIndex );

        /*------------------------------------------------
        Step 2: Set the transfer address registers
        See table 30 in RM0390
        ------------------------------------------------*/
        if ( ( config->Direction == Configuration::TransferDirection::P2M ) ||
             ( config->Direction == Configuration::TransferDirection::M2M ) )
        {
          SxPAR::set( stream, controlBlock->srcAddress );
          SxM0AR::set( stream, controlBlock->dstAddress );
        }
        else if ( config->Direction == Configuration::TransferDirection::M2P )
        {
          SxM0AR::set( stream, controlBlock->srcAddress );
          SxM1AR::set( stream, controlBlock->dstAddress );
        }
        else
        {
          return Chimera::CommonStatusCodes::FAIL;
        }

        /*------------------------------------------------
        Step 3: Set how many bytes are to be transfered
        ------------------------------------------------*/
        SxNDTR::set( stream, controlBlock->numBytes );

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
      }
    }

    Chimera::Status_t Stream::start()
    {
      using namespace Chimera::Threading;

      auto result = Chimera::CommonStatusCodes::LOCKED;

      if ( lock( TIMEOUT_DONT_WAIT ) )
      {
        // Set up interrupts (bits/priority/nvic)

        // Make sure the TCB is ok

        // Set the enable bit
      }
    }

    Chimera::Status_t Stream::pause()
    {
      // no need for lock
    }

    Chimera::Status_t Stream::abort()
    {
      // no need for lock
    }

    void Stream::IRQHandler( const uint8_t channel, const uint8_t request )
    {
      // Update the control block


      xSemaphoreGiveFromISR( wakeupSignal, nullptr );
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
    Make sure a stream instance exists
    ------------------------------------------------*/
    auto streamArray = dma1Streams;

    if ( periph == DMA2_PERIPH )
    {
      streamArray = dma2Streams;
    }

    for ( uint8_t x = 0; x < streamArray.size(); x++ )
    {
      if ( !streamArray[ x ] )
      {
        streamArray[ x ] = new Internal::Stream();
        streamArray[ x ]->attach( getStream( periph, x ), periph );
      }
    }

    return Chimera::CommonStatusCodes::OK;
  }
}    // namespace Thor::Driver::DMA


using namespace Thor::DMA;
using namespace Thor::Driver::DMA;

void DMA1_Stream0_IRQHandler( void )
{
  constexpr uint8_t stream = 0u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_STREAM0 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream1_IRQHandler( void )
{
  constexpr uint8_t stream = 1u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_STREAM1 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream2_IRQHandler( void )
{
  constexpr uint8_t stream = 2u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_STREAM2 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream3_IRQHandler( void )
{
  constexpr uint8_t stream = 3u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_STREAM3 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream4_IRQHandler( void )
{
  constexpr uint8_t stream = 4u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_STREAM4 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream5_IRQHandler( void )
{
  constexpr uint8_t stream = 5u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_STREAM5 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream6_IRQHandler( void )
{
  constexpr uint8_t stream = 6u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_STREAM6 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA1_Stream7_IRQHandler( void )
{
  constexpr uint8_t stream = 7u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA1_STREAM7 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma1RequestMapping[ channel ][ stream ];

  if ( dma1Streams[ stream ] )
  {
    dma1Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream0_IRQHandler( void )
{
  constexpr uint8_t stream = 0u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_STREAM0 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream1_IRQHandler( void )
{
  constexpr uint8_t stream = 1u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_STREAM1 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream2_IRQHandler( void )
{
  constexpr uint8_t stream = 2u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_STREAM2 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream3_IRQHandler( void )
{
  constexpr uint8_t stream = 3u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_STREAM3 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream4_IRQHandler( void )
{
  constexpr uint8_t stream = 4u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_STREAM4 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream5_IRQHandler( void )
{
  constexpr uint8_t stream = 5u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_STREAM5 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream6_IRQHandler( void )
{
  constexpr uint8_t stream = 6u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_STREAM6 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}

void DMA2_Stream7_IRQHandler( void )
{
  constexpr uint8_t stream = 7u;
  const uint8_t channel    = SxCR::CHSEL::get( DMA2_STREAM7 ) >> SxCR_CHSEL_Pos;
  const uint8_t request    = dma2RequestMapping[ channel ][ stream ];

  if ( dma2Streams[ stream ] )
  {
    dma2Streams[ stream ]->IRQHandler( channel, request );
  }
}


#endif /* TARGET_STM32F4 && THOR_DRIVER_DMA */