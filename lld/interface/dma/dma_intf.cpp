/********************************************************************************
 *  File Name:
 *    dma_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/inc/dma>

#if defined( THOR_LLD_DMA )
namespace Thor::LLD::DMA
{
  /*-------------------------------------------------------------------------------
  Static Functions
  -------------------------------------------------------------------------------*/
  static uint8_t find_signal_attributes( const Source signal )
  {
    for( size_t idx = 0; idx < ARRAY_COUNT( Config::RequestMap ); idx++ )
    {
      if( signal == Config::RequestMap[ idx ].request )
      {
        return Config::RequestMap[ idx ].attributes;
      }
    }

    return 0;
  }

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported( const Controller channel, const Streamer stream )
  {
#if defined( STM32_DMA1_PERIPH_AVAILABLE )
    if( channel == Controller::DMA_1 )
    {
      switch( stream )
      {
        case Streamer::NONE:
          return true;
          break;

#if defined( STM32_DMA1_STREAM0_AVAILABLE )
        case Streamer::STREAM_0:
          return true;
          break;
#endif

#if defined( STM32_DMA1_STREAM1_AVAILABLE )
        case Streamer::STREAM_1:
          return true;
          break;
#endif

#if defined( STM32_DMA1_STREAM2_AVAILABLE )
        case Streamer::STREAM_2:
          return true;
          break;
#endif

#if defined( STM32_DMA1_STREAM3_AVAILABLE )
        case Streamer::STREAM_3:
          return true;
          break;
#endif

#if defined( STM32_DMA1_STREAM4_AVAILABLE )
        case Streamer::STREAM_4:
          return true;
          break;
#endif

#if defined( STM32_DMA1_STREAM5_AVAILABLE )
        case Streamer::STREAM_5:
          return true;
          break;
#endif

#if defined( STM32_DMA1_STREAM6_AVAILABLE )
        case Streamer::STREAM_6:
          return true;
          break;
#endif

#if defined( STM32_DMA1_STREAM7_AVAILABLE )
        case Streamer::STREAM_7:
          return true;
          break;
#endif

        default:
          return false;
          break;
      };
    }
#endif
#if defined( STM32_DMA2_PERIPH_AVAILABLE )
    if( channel == Controller::DMA_2 )
    {
      switch( stream )
      {
        case Streamer::NONE:
          return true;
          break;

#if defined( STM32_DMA2_STREAM0_AVAILABLE )
        case Streamer::STREAM_0:
          return true;
          break;
#endif

#if defined( STM32_DMA2_STREAM1_AVAILABLE )
        case Streamer::STREAM_1:
          return true;
          break;
#endif

#if defined( STM32_DMA2_STREAM2_AVAILABLE )
        case Streamer::STREAM_2:
          return true;
          break;
#endif

#if defined( STM32_DMA2_STREAM3_AVAILABLE )
        case Streamer::STREAM_3:
          return true;
          break;
#endif

#if defined( STM32_DMA2_STREAM4_AVAILABLE )
        case Streamer::STREAM_4:
          return true;
          break;
#endif

#if defined( STM32_DMA2_STREAM5_AVAILABLE )
        case Streamer::STREAM_5:
          return true;
          break;
#endif

#if defined( STM32_DMA2_STREAM6_AVAILABLE )
        case Streamer::STREAM_6:
          return true;
          break;
#endif

#if defined( STM32_DMA2_STREAM7_AVAILABLE )
        case Streamer::STREAM_7:
          return true;
          break;
#endif

        default:
          return false;
          break;
      };
    }
#endif

    return false;
  }


  RIndex_t getResourceIndex( const Source dmaSignal )
  {
    /*-------------------------------------------------
    Find the entry in the DMA signal map
    -------------------------------------------------*/
    auto attr = find_signal_attributes( dmaSignal );
    if( !attr )
    {
      return INVALID_RESOURCE_INDEX;
    }

    /*-------------------------------------------------
    Convert the packed bit fields and look up the index
    -------------------------------------------------*/
    Controller ctrl = ( ( attr & ON_DMA1 ) == ON_DMA1 ) ? Controller::DMA_1 : Controller::DMA_2;
    Streamer stream = static_cast<Streamer>( ( attr & ON_STREAM_MSK ) >> ON_STREAM_POS );

    return getResourceIndex( ctrl, stream );
  }


  Channel getChannel( const Source dmaSignal )
  {
    /*-------------------------------------------------
    Find the entry in the DMA signal map
    -------------------------------------------------*/
    auto attr = find_signal_attributes( dmaSignal );
    if( !attr )
    {
      return Channel::INVALID;
    }

    /*-------------------------------------------------
    Convert the packed bit field in to the channel id
    -------------------------------------------------*/
    return static_cast<Channel>( ( attr & ON_CHANNEL_MSK ) >> ON_CHANNEL_POS );
  }


  RIndex_t getResourceIndex( const Controller channel, Streamer stream )
  {
#if defined( STM32_DMA1_PERIPH_AVAILABLE )
    if( channel == Controller::DMA_1 )
    {
      switch( stream )
      {
        case Streamer::NONE:
          return DMA1_RESOURCE_INDEX;
          break;

#if defined( STM32_DMA1_STREAM0_AVAILABLE )
        case Streamer::STREAM_0:
          return DMA1_STREAM0_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA1_STREAM1_AVAILABLE )
        case Streamer::STREAM_1:
          return DMA1_STREAM1_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA1_STREAM2_AVAILABLE )
        case Streamer::STREAM_2:
          return DMA1_STREAM2_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA1_STREAM3_AVAILABLE )
        case Streamer::STREAM_3:
          return DMA1_STREAM3_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA1_STREAM4_AVAILABLE )
        case Streamer::STREAM_4:
          return DMA1_STREAM4_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA1_STREAM5_AVAILABLE )
        case Streamer::STREAM_5:
          return DMA1_STREAM5_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA1_STREAM6_AVAILABLE )
        case Streamer::STREAM_6:
          return DMA1_STREAM6_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA1_STREAM7_AVAILABLE )
        case Streamer::STREAM_7:
          return DMA1_STREAM7_RESOURCE_INDEX;
          break;
#endif

        default:
          return INVALID_RESOURCE_INDEX;
          break;
      };
    }
#endif
#if defined( STM32_DMA2_PERIPH_AVAILABLE )
    if( channel == Controller::DMA_2 )
    {
      switch( stream )
      {
        case Streamer::NONE:
          return DMA2_RESOURCE_INDEX;
          break;

#if defined( STM32_DMA2_STREAM0_AVAILABLE )
        case Streamer::STREAM_0:
          return DMA2_STREAM0_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA2_STREAM1_AVAILABLE )
        case Streamer::STREAM_1:
          return DMA2_STREAM1_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA2_STREAM2_AVAILABLE )
        case Streamer::STREAM_2:
          return DMA2_STREAM2_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA2_STREAM3_AVAILABLE )
        case Streamer::STREAM_3:
          return DMA2_STREAM3_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA2_STREAM4_AVAILABLE )
        case Streamer::STREAM_4:
          return DMA2_STREAM4_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA2_STREAM5_AVAILABLE )
        case Streamer::STREAM_5:
          return DMA2_STREAM5_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA2_STREAM6_AVAILABLE )
        case Streamer::STREAM_6:
          return DMA2_STREAM6_RESOURCE_INDEX;
          break;
#endif

#if defined( STM32_DMA2_STREAM7_AVAILABLE )
        case Streamer::STREAM_7:
          return DMA2_STREAM7_RESOURCE_INDEX;
          break;
#endif

        default:
          return INVALID_RESOURCE_INDEX;
          break;
      };
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_DMA1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( DMA1_PERIPH ) )
    {
      return DMA1_RESOURCE_INDEX;
    }
#if defined( STM32_DMA1_STREAM0_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ) )
    {
      return DMA1_STREAM0_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA1_STREAM1_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ) )
    {
      return DMA1_STREAM1_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA1_STREAM2_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ) )
    {
      return DMA1_STREAM2_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA1_STREAM3_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ) )
    {
      return DMA1_STREAM3_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA1_STREAM4_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ) )
    {
      return DMA1_STREAM4_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA1_STREAM5_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ) )
    {
      return DMA1_STREAM5_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA1_STREAM6_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM6 ) )
    {
      return DMA1_STREAM6_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA1_STREAM7_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ) )
    {
      return DMA1_STREAM7_RESOURCE_INDEX;
    }
#endif
#endif /* STM32_DMA1_PERIPH_AVAILABLE */

#if defined( STM32_DMA2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( DMA2_PERIPH ) )
    {
      return DMA2_RESOURCE_INDEX;
    }
#if defined( STM32_DMA2_STREAM0_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ) )
    {
      return DMA2_STREAM0_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA2_STREAM1_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ) )
    {
      return DMA2_STREAM2_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA2_STREAM2_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ) )
    {
      return DMA2_STREAM2_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA2_STREAM3_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ) )
    {
      return DMA2_STREAM3_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA2_STREAM4_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ) )
    {
      return DMA2_STREAM4_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA2_STREAM5_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ) )
    {
      return DMA2_STREAM5_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA2_STREAM6_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM6 ) )
    {
      return DMA2_STREAM6_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_DMA2_STREAM7_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ) )
    {
      return DMA2_STREAM7_RESOURCE_INDEX;
    }
#endif
#endif  /* STM32_DMA2_PERIPH_AVAILABLE */

    return INVALID_RESOURCE_INDEX;
  }


  Controller getController( const std::uintptr_t address )
  {
#if defined( STM32_DMA1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( DMA1_PERIPH ) )
    {
      return Controller::DMA_1;
    }
#if defined( STM32_DMA1_STREAM0_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ) )
    {
      return Controller::DMA_1;
    }
#endif

#if defined( STM32_DMA1_STREAM1_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ) )
    {
      return Controller::DMA_1;
    }
#endif

#if defined( STM32_DMA1_STREAM2_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ) )
    {
      return Controller::DMA_1;
    }
#endif

#if defined( STM32_DMA1_STREAM3_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ) )
    {
      return Controller::DMA_1;
    }
#endif

#if defined( STM32_DMA1_STREAM4_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ) )
    {
      return Controller::DMA_1;
    }
#endif

#if defined( STM32_DMA1_STREAM5_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ) )
    {
      return Controller::DMA_1;
    }
#endif

#if defined( STM32_DMA1_STREAM6_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM6 ) )
    {
      return Controller::DMA_1;
    }
#endif

#if defined( STM32_DMA1_STREAM7_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ) )
    {
      return Controller::DMA_1;
    }
#endif
#endif /* STM32_DMA1_PERIPH_AVAILABLE */


#if defined( STM32_DMA2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( DMA2_PERIPH ) )
    {
      return Controller::DMA_2;
    }
#if defined( STM32_DMA2_STREAM0_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ) )
    {
      return Controller::DMA_2;
    }
#endif

#if defined( STM32_DMA2_STREAM1_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ) )
    {
      return Controller::DMA_2;
    }
#endif

#if defined( STM32_DMA2_STREAM2_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ) )
    {
      return Controller::DMA_2;
    }
#endif

#if defined( STM32_DMA2_STREAM3_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ) )
    {
      return Controller::DMA_2;
    }
#endif

#if defined( STM32_DMA2_STREAM4_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ) )
    {
      return Controller::DMA_2;
    }
#endif

#if defined( STM32_DMA2_STREAM5_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ) )
    {
      return Controller::DMA_2;
    }
#endif

#if defined( STM32_DMA2_STREAM6_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM6 ) )
    {
      return Controller::DMA_2;
    }
#endif

#if defined( STM32_DMA2_STREAM7_AVAILABLE )
    else if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ) )
    {
      return Controller::DMA_2;
    }
#endif
#endif  /* STM32_DMA2_PERIPH_AVAILABLE */

    return Controller::NONE;
  }


  Streamer getStream( const std::uintptr_t address )
  {
#if defined( STM32_DMA1_PERIPH_AVAILABLE )
#if defined( STM32_DMA1_STREAM0_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM0 ) )
    {
      return Streamer::STREAM_0;
    }
#endif

#if defined( STM32_DMA1_STREAM1_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM1 ) )
    {
      return Streamer::STREAM_1;
    }
#endif

#if defined( STM32_DMA1_STREAM2_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM2 ) )
    {
      return Streamer::STREAM_2;
    }
#endif

#if defined( STM32_DMA1_STREAM3_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM3 ) )
    {
      return Streamer::STREAM_3;
    }
#endif

#if defined( STM32_DMA1_STREAM4_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM4 ) )
    {
      return Streamer::STREAM_4;
    }
#endif

#if defined( STM32_DMA1_STREAM5_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM5 ) )
    {
      return Streamer::STREAM_5;
    }
#endif

#if defined( STM32_DMA1_STREAM6_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM6 ) )
    {
      return Streamer::STREAM_6;
    }
#endif

#if defined( STM32_DMA1_STREAM7_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA1_STREAM7 ) )
    {
      return Streamer::STREAM_7;
    }
#endif
#endif /* STM32_DMA1_PERIPH_AVAILABLE */


#if defined( STM32_DMA2_PERIPH_AVAILABLE )
#if defined( STM32_DMA2_STREAM0_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM0 ) )
    {
      return Streamer::STREAM_0;
    }
#endif

#if defined( STM32_DMA2_STREAM1_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM1 ) )
    {
      return Streamer::STREAM_1;
    }
#endif

#if defined( STM32_DMA2_STREAM2_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM2 ) )
    {
      return Streamer::STREAM_2;
    }
#endif

#if defined( STM32_DMA2_STREAM3_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM3 ) )
    {
      return Streamer::STREAM_3;
    }
#endif

#if defined( STM32_DMA2_STREAM4_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM4 ) )
    {
      return Streamer::STREAM_4;
    }
#endif

#if defined( STM32_DMA2_STREAM5_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM5 ) )
    {
      return Streamer::STREAM_5;
    }
#endif

#if defined( STM32_DMA2_STREAM6_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM6 ) )
    {
      return Streamer::STREAM_6;
    }
#endif

#if defined( STM32_DMA2_STREAM7_AVAILABLE )
    if( address == reinterpret_cast<std::uintptr_t>( DMA2_STREAM7 ) )
    {
      return Streamer::STREAM_7;
    }
#endif
#endif  /* STM32_DMA2_PERIPH_AVAILABLE */

    return Streamer::NONE;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_DMA_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_DMA1_PERIPH_AVAILABLE )
    result |= driverList[ DMA1_RESOURCE_INDEX ].attach( DMA1_PERIPH );
#endif
#if defined( STM32_DMA2_PERIPH_AVAILABLE )
    result |= driverList[ DMA2_RESOURCE_INDEX ].attach( DMA2_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }

}  // namespace Thor::LLD::DMA
#endif  /* THOR_LLD_DMA */
