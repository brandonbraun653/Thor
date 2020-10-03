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
    return false;
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
    return 0;
  }


  RIndex_t getResourceIndex( const Chimera::CAN::Channel channel )
  {
    return 0;
  }


  bool attachDriverInstances( Driver *const driverList, const size_t numDrivers )
  {
    return false;
  }


  Chimera::CAN::Channel getChannel( const std::uintptr_t address )
  {
    return Chimera::CAN::Channel::UNKNOWN;
  }

}    // namespace Thor::LLD::CAN
