/******************************************************************************
 *  File Name:
 *    dma_common_driver.cpp
 *
 *  Description:
 *    Shared driver for DMA across multiple STM32 chips
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Aurora Includes */
#include <Aurora/utility>

/* Chimera Includes */
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/rcc>

#if defined( THOR_DMA )
namespace Thor::LLD::DMA
{
  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static Driver s_dma_drivers[ NUM_DMA_PERIPHS ];
  static Stream s_stream_drivers[ NUM_DMA_STREAMS_TOTAL ];

  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  namespace Resource
  {
    // static void lockDMA()
    // {
    //   for( size_t idx = 0; idx < ARRAY_COUNT( Resource::IRQSignals ); idx++ )
    //   {
    //     INT::disableIRQ( Resource::IRQSignals[ idx ] );
    //   }
    // }

    // static void unlockDMA()
    // {
    //   for( size_t idx = 0; idx < ARRAY_COUNT( Resource::IRQSignals ); idx++ )
    //   {
    //     INT::enableIRQ( Resource::IRQSignals[ idx ] );
    //   }
    // }

    // static etl::function_fv<lockDMA> fv_lockDMA;
    // static etl::function_fv<unlockDMA> fv_unlockDMA;

    ISREventQueue ISRQueue; //{ fv_lockDMA, fv_unlockDMA };
  }    // namespace Resource


  /*---------------------------------------------------------------------------
  Interface Implementations
  ---------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------------------------------*/
    attachDriverInstances( s_dma_drivers, ARRAY_COUNT( s_dma_drivers ) );

    /*-------------------------------------------------------------------------
    Perform the HW init sequence
    -------------------------------------------------------------------------*/
    for( size_t idx = 0; idx < ARRAY_COUNT( s_dma_drivers ); idx++ )
    {
      s_dma_drivers[ idx ].init();
    }

    return true;
  }


  Driver_rPtr getDriver( const Controller channel )
  {
    if ( auto idx = getResourceIndex( channel, Streamer::NONE ); idx != INVALID_RESOURCE_INDEX )
    {
      return &s_dma_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  Stream_rPtr getStream( const Controller control, const Streamer stream )
  {
    RIndex_t idx = getResourceIndex( control, stream );
    if( idx < ARRAY_COUNT( s_stream_drivers ) )
    {
      return &s_stream_drivers[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  Stream_rPtr getStream( const RIndex_t index )
  {
    if( index < ARRAY_COUNT( s_stream_drivers ) )
    {
      return &s_stream_drivers[ index ];
    }
    else
    {
      return nullptr;
    }
  }


  StreamMap *streamView( RegisterMap *const periph, const Streamer streamNum )
  {
    #if defined( STM32F446xx )
    /*-------------------------------------------------------------------------
    See 9.5.5 in RM0390
    -------------------------------------------------------------------------*/
    static constexpr size_t fixedOffset  = 0x10;
    static constexpr size_t streamOffset = 0x18;

    auto address = reinterpret_cast<std::uintptr_t>( periph ) + fixedOffset + ( streamOffset * EnumValue( streamNum ) );
    return reinterpret_cast<StreamMap *const>( address );

    #elif defined( STM32L432xx )
    /*-------------------------------------------------------------------------
    See 11.6.3 in RM0394
    -------------------------------------------------------------------------*/
    static constexpr size_t fixedOffset  = 0x08;
    static constexpr size_t streamOffset = 0x14;

    if( ( EnumValue( streamNum ) < 1 ) || ( EnumValue( streamNum ) > 7 ) )
    {
      return nullptr;
    }

    auto address = reinterpret_cast<std::uintptr_t>( periph ) + fixedOffset + ( streamOffset * ( EnumValue( streamNum ) - 1 ) );
    return reinterpret_cast<StreamMap *const>( address );
    #else
    return nullptr;
    #endif
  }


  /*---------------------------------------------------------------------------
  Driver Implementation
  ---------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init()
  {
    /*-------------------------------------------------------------------------
    Entrancy protection
    -------------------------------------------------------------------------*/
    if ( !mPeriph )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*-------------------------------------------------------------------------
    Reset the peripheral back to its default conditions
    -------------------------------------------------------------------------*/
    clockEnable();
    reset();
    clockEnable();

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    /*-------------------------------------------------------------------------
    Configure the stream drivers
    -------------------------------------------------------------------------*/
    auto result = Chimera::Status::OK;
    switch( getController( reinterpret_cast<std::uintptr_t>( peripheral ) ) )
    {
      case Controller::DMA_1:
#if defined( STM32_DMA1_STREAM0_AVAILABLE )
        result |= s_stream_drivers[ DMA1_STREAM0_RESOURCE_INDEX ].attach( DMA1_STREAM0, peripheral );
#endif
#if defined( STM32_DMA1_STREAM1_AVAILABLE )
        result |= s_stream_drivers[ DMA1_STREAM1_RESOURCE_INDEX ].attach( DMA1_STREAM1, peripheral );
#endif
#if defined( STM32_DMA1_STREAM2_AVAILABLE )
        result |= s_stream_drivers[ DMA1_STREAM2_RESOURCE_INDEX ].attach( DMA1_STREAM2, peripheral );
#endif
#if defined( STM32_DMA1_STREAM3_AVAILABLE )
        result |= s_stream_drivers[ DMA1_STREAM3_RESOURCE_INDEX ].attach( DMA1_STREAM3, peripheral );
#endif
#if defined( STM32_DMA1_STREAM4_AVAILABLE )
        result |= s_stream_drivers[ DMA1_STREAM4_RESOURCE_INDEX ].attach( DMA1_STREAM4, peripheral );
#endif
#if defined( STM32_DMA1_STREAM5_AVAILABLE )
        result |= s_stream_drivers[ DMA1_STREAM5_RESOURCE_INDEX ].attach( DMA1_STREAM5, peripheral );
#endif
#if defined( STM32_DMA1_STREAM6_AVAILABLE )
        result |= s_stream_drivers[ DMA1_STREAM6_RESOURCE_INDEX ].attach( DMA1_STREAM6, peripheral );
#endif
#if defined( STM32_DMA1_STREAM7_AVAILABLE )
        result |= s_stream_drivers[ DMA1_STREAM7_RESOURCE_INDEX ].attach( DMA1_STREAM7, peripheral );
#endif
        break;

      case Controller::DMA_2:
#if defined( STM32_DMA1_STREAM0_AVAILABLE )
        result |= s_stream_drivers[ DMA2_STREAM0_RESOURCE_INDEX ].attach( DMA2_STREAM0, peripheral );
#endif
#if defined( STM32_DMA1_STREAM1_AVAILABLE )
        result |= s_stream_drivers[ DMA2_STREAM1_RESOURCE_INDEX ].attach( DMA2_STREAM1, peripheral );
#endif
#if defined( STM32_DMA1_STREAM2_AVAILABLE )
        result |= s_stream_drivers[ DMA2_STREAM2_RESOURCE_INDEX ].attach( DMA2_STREAM2, peripheral );
#endif
#if defined( STM32_DMA1_STREAM3_AVAILABLE )
        result |= s_stream_drivers[ DMA2_STREAM3_RESOURCE_INDEX ].attach( DMA2_STREAM3, peripheral );
#endif
#if defined( STM32_DMA1_STREAM4_AVAILABLE )
        result |= s_stream_drivers[ DMA2_STREAM4_RESOURCE_INDEX ].attach( DMA2_STREAM4, peripheral );
#endif
#if defined( STM32_DMA1_STREAM5_AVAILABLE )
        result |= s_stream_drivers[ DMA2_STREAM5_RESOURCE_INDEX ].attach( DMA2_STREAM5, peripheral );
#endif
#if defined( STM32_DMA1_STREAM6_AVAILABLE )
        result |= s_stream_drivers[ DMA2_STREAM6_RESOURCE_INDEX ].attach( DMA2_STREAM6, peripheral );
#endif
#if defined( STM32_DMA1_STREAM7_AVAILABLE )
        result |= s_stream_drivers[ DMA2_STREAM7_RESOURCE_INDEX ].attach( DMA2_STREAM7, peripheral );
#endif
        break;

      default:
        return Chimera::Status::NOT_SUPPORTED;
        break;
    };

    /*-------------------------------------------------------------------------
    Enable the peripheral
    -------------------------------------------------------------------------*/
    mPeriph = peripheral;
    clockEnable();

    return result;
  }


  void Driver::clockEnable()
  {
    auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( mPeriph ) );

    rcc->enableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
  }


  void Driver::clockDisable()
  {
    auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( mPeriph ) );

    rcc->disableClock( Chimera::Peripheral::Type::PERIPH_DMA, index );
  }


  void Driver::reset()
  {
    auto rcc   = Thor::LLD::RCC::getPeriphClockCtrl();
    auto index = getResourceIndex( reinterpret_cast<std::uintptr_t>( mPeriph ) );

    rcc->reset( Chimera::Peripheral::Type::PERIPH_DMA, index );
  }

}  // namespace Thor::LLD::DMA
#endif  /* THOR_LLD_DMA */
