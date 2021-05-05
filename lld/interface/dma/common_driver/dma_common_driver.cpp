/********************************************************************************
 *  File Name:
 *    dma_common_driver.cpp
 *
 *  Description:
 *    Shared driver for DMA across multiple STM32 chips
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Aurora Includes */
#include <Aurora/utility>

/* Chimera Includes */
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/lld/interface/inc/dma>
#include <Thor/lld/interface/inc/rcc>

namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Static Data
  -------------------------------------------------------------------------------*/
  static Driver s_dma_drivers[ NUM_DMA_PERIPHS ];
  static Stream s_stream_drivers[ NUM_DMA_STREAMS_TOTAL ];


  /*-------------------------------------------------------------------------------
  Interface Implementations
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    /*-------------------------------------------------
    Attach all the expected peripherals to the drivers
    -------------------------------------------------*/
    if ( attachDriverInstances( s_dma_drivers, ARRAY_COUNT( s_dma_drivers ) ) )
    {
      return Chimera::Status::OK;
    }
    else
    {
      return Chimera::Status::FAIL;
    }
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


  StreamMap *streamView( RegisterMap *const periph, const Streamer streamNum )
  {
    /*------------------------------------------------
    This equation taken directly from register spec in datasheet.
    See 9.5.5 in RM0390
    ------------------------------------------------*/
    static constexpr size_t fixedOffset  = 0x10;
    static constexpr size_t streamOffset = 0x18;

    auto address = reinterpret_cast<std::uintptr_t>( periph ) + fixedOffset + ( streamOffset * EnumValue( streamNum ) );
    return reinterpret_cast<StreamMap *const>( address );
  }


  /*-------------------------------------------------------------------------------
  Driver Implementation
  -------------------------------------------------------------------------------*/
  Driver::Driver() : mPeriph( nullptr )
  {
  }


  Driver::~Driver()
  {
  }


  Chimera::Status_t Driver::init()
  {
    /*-------------------------------------------------
    Entrancy protection
    -------------------------------------------------*/
    if ( !mPeriph )
    {
      return Chimera::Status::NOT_INITIALIZED;
    }

    /*------------------------------------------------
    Reset the peripheral back to its default conditions
    ------------------------------------------------*/
    clockEnable();
    reset();
    clockEnable();

    return Chimera::Status::OK;
  }


  Chimera::Status_t Driver::attach( RegisterMap *const peripheral )
  {
    mPeriph = peripheral;
    clockEnable();
    return Chimera::Status::OK;
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
