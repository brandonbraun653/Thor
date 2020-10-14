/********************************************************************************
 *  File Name:
 *    can_intf.cpp
 *
 *  Description:
 *    LLD interface functions that are processor independent
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/utility>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/can/can_detail.hpp>
#include <Thor/lld/interface/can/can_prv_data.hpp>
#include <Thor/lld/interface/can/can_types.hpp>
#include <Thor/lld/interface/can/can_intf.hpp>

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported( const Chimera::CAN::Channel channel )
  {
    switch ( channel )
    {
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
      case Chimera::CAN::Channel::CAN0:
        return true;
        break;
#endif

      default:
        return false;
        break;
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

    return INVALID_RESOURCE_INDEX;
  }


  RIndex_t getResourceIndex( const Chimera::CAN::Channel channel )
  {
    switch ( channel )
    {
#if defined( STM32_CAN1_PERIPH_AVAILABLE )
      case Chimera::CAN::Channel::CAN0:
        return CAN1_RESOURCE_INDEX;
        break;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
        break;
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

    return Chimera::CAN::Channel::UNKNOWN;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_CAN_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_CAN1_PERIPH_AVAILABLE )
    driverList[ CAN1_RESOURCE_INDEX ].attach( CAN1_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }

}    // namespace Thor::LLD::CAN
