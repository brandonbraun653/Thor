/******************************************************************************
 *  File Name:
 *    can_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/utility>
#include <Thor/cfg>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/can/can_detail.hpp>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_prv_data.hpp>
#include <Thor/lld/interface/can/can_types.hpp>
#include <etl/algorithm.h>

#if defined( THOR_CAN )
namespace Thor::LLD::CAN
{
  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static uint8_t filterSize( const MessageFilter *const filter );

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  bool isSupported( const Chimera::CAN::Channel channel )
  {
    switch ( channel )
    {
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
      case Chimera::CAN::Channel::CAN0:
        return true;
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
      case Chimera::CAN::Channel::CAN1:
        return true;
#endif

      default:
        return false;
    };
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( CAN1_PERIPH ) )
    {
      return CAN1_RESOURCE_INDEX;
    }
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( CAN2_PERIPH ) )
    {
      return CAN2_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  RIndex_t getResourceIndex( const Chimera::CAN::Channel channel )
  {
    switch ( channel )
    {
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
      case Chimera::CAN::Channel::CAN0:
        return CAN1_RESOURCE_INDEX;
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
      case Chimera::CAN::Channel::CAN1:
        return CAN2_RESOURCE_INDEX;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
    };
  }


  Chimera::CAN::Channel getChannel( const std::uintptr_t address )
  {
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( CAN1_PERIPH ) )
    {
      return Chimera::CAN::Channel::CAN0;
    }
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( CAN2_PERIPH ) )
    {
      return Chimera::CAN::Channel::CAN1;
    }
#endif

    return Chimera::CAN::Channel::UNKNOWN;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------------------------------
    Reject bad inputs
    -------------------------------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_CAN_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Attach the drivers
    -------------------------------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    driverList[ CAN1_RESOURCE_INDEX ].attach( CAN1_PERIPH );
#endif
#if defined( STM32_CAN2_PERIPH_AVAILABLE )
    driverList[ CAN2_RESOURCE_INDEX ].attach( CAN2_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }


  bool sortFiltersBySize( const MessageFilter *const filterList, const uint8_t listSize, uint8_t *const indexList )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------------------------------
    Input Protection
    -------------------------------------------------------------------------*/
    if ( !filterList || !listSize || !indexList )
    {
      return false;
    }

    /*-------------------------------------------------------------------------
    Initialize the algorithm
    -------------------------------------------------------------------------*/
    for ( uint8_t idx = 0; idx < listSize; idx++ )
    {
      indexList[ idx ] = idx;
    }

    /*-------------------------------------------------------------------------
    Sort away!
    -------------------------------------------------------------------------*/
    etl::shell_sort( indexList, indexList + listSize, [ filterList ]( uint8_t a, uint8_t b ) {
      return filterSize( &filterList[ a ] ) > filterSize( &filterList[ b ] );
    } );

    return true;
  }


  /*---------------------------------------------------------------------------
  Static Functions
  ---------------------------------------------------------------------------*/
  static uint8_t filterSize( const MessageFilter *const filter )
  {
    using namespace Chimera::CAN;

    /*-------------------------------------------------------------------------
    Reject bad inputs
    -------------------------------------------------------------------------*/
    if ( !filter || !filter->valid )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Assign a size metric to each filter. Essentially
    there are four possible choices based on the scale
    and mode registers (CAN_FSCx & CAN_FMBx).

    | Scale Bit | Mode Bit | Bytes-Per-Filter |             Bank Config Type             |
    |:---------:|:--------:|:----------------:|:----------------------------------------:|
    |     0     |     0    |         4        | Two 16-bit filters per bank (Mask Mode)  |
    |     0     |     1    |         2        | Four 16-bit filters per bank (List Mode) |
    |     1     |     0    |         8        | One 32-bit filter per bank (Mask Mode)   |
    |     1     |     1    |         4        | Two 32-bit filters per bank (List Mode)  |

    Taken from RM0394 (Rev 4) Fig. 484
    -------------------------------------------------------------------------*/
    if ( filter->filterType == Thor::CAN::FilterType::MODE_32BIT_MASK )
    {
      return 8;
    }
    else if ( filter->filterType == Thor::CAN::FilterType::MODE_32BIT_LIST )
    {
      // 32-bit filters have priority over 16-bit, so make its "size" just a little larger
      return 5;
    }
    else if ( filter->filterType == Thor::CAN::FilterType::MODE_16BIT_MASK )
    {
      return 4;
    }
    else if ( filter->filterType == Thor::CAN::FilterType::MODE_16BIT_LIST )
    {
      return 2;
    }
    else
    {
      // Error condition. This filter wasn't configured correctly.
      return 0;
    }
  }
}    // namespace Thor::LLD::CAN

#endif /* THOR_LLD_CAN */
