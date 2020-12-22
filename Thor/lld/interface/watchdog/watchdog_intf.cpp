/********************************************************************************
 *  File Name:
 *    watchdog_intf.cpp
 *
 *  Description:
 *    Common watchdog interface function implementation
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/watchdog/watchdog_detail.hpp>
#include <Thor/lld/interface/watchdog/watchdog_prv_data.hpp>
#include <Thor/lld/interface/watchdog/watchdog_types.hpp>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>

namespace Thor::LLD::Watchdog
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  bool isSupported( const Chimera::Watchdog::IChannel channel )
  {
    switch ( channel )
    {
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
      case Chimera::Watchdog::IChannel::WATCHDOG0:
        return true;
        break;
#endif

      default:
        return false;
        break;
    };
  }


  bool isSupported( const Chimera::Watchdog::WChannel channel )
  {
    switch ( channel )
    {
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
      case Chimera::Watchdog::WChannel::WATCHDOG0:
        return true;
        break;
#endif

      default:
        return false;
        break;
    };
  }


  RIndex_t getResourceIndex( const Chimera::Watchdog::IChannel channel )
  {
    switch ( channel )
    {
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
      case Chimera::Watchdog::IChannel::WATCHDOG0:
        return IWDG1_RESOURCE_INDEX;
        break;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


  RIndex_t getResourceIndex( const Chimera::Watchdog::WChannel channel )
  {
    switch ( channel )
    {
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
      case Chimera::Watchdog::WChannel::WATCHDOG0:
        return WWDG1_RESOURCE_INDEX;
        break;
#endif

      default:
        return INVALID_RESOURCE_INDEX;
        break;
    };
  }


  RIndex_t getResourceIndex( const std::uintptr_t address )
  {
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( IWDG1_PERIPH ) )
    {
      return IWDG1_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( WWDG1_PERIPH ) )
    {
      return WWDG1_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  Chimera::Watchdog::IChannel getIChannel( const std::uintptr_t address )
  {
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( IWDG1_PERIPH ) )
    {
      return Chimera::Watchdog::IChannel::IWDG1;
    }
#endif

    return Chimera::Watchdog::IChannel::UNKNOWN;
  }


  Chimera::Watchdog::WChannel getWChannel( const std::uintptr_t address )
  {
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( WWDG1_PERIPH ) )
    {
      return Chimera::Watchdog::WChannel::WWDG1;
    }
#endif

    return Chimera::Watchdog::WChannel::UNKNOWN;
  }


  bool attachDriverInstances( IndependentDriver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_IWDG_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
    result |= driverList[ IWDG1_RESOURCE_INDEX ].attach( IWDG1_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }


  bool attachDriverInstances( WindowDriver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != NUM_WWDG_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    result |= driverList[ WWDG1_RESOURCE_INDEX ].attach( WWDG1_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }

}  // namespace Thor::LLD::Watchdog
