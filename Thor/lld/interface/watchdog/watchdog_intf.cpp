/********************************************************************************
 *  File Name:
 *    watchdog_intf.cpp
 *
 *  Description:
 *    Common watchdog interface function implementation
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */
#include <cmath>

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
        return IWDG::IWDG1_RESOURCE_INDEX;
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
        return WWDG::WWDG1_RESOURCE_INDEX;
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
      return IWDG::IWDG1_RESOURCE_INDEX;
    }
#endif

#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( WWDG1_PERIPH ) )
    {
      return WWDG::WWDG1_RESOURCE_INDEX;
    }
#endif

    return INVALID_RESOURCE_INDEX;
  }


  Chimera::Watchdog::IChannel getIChannel( const std::uintptr_t address )
  {
#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( IWDG1_PERIPH ) )
    {
      return Chimera::Watchdog::IChannel::WATCHDOG0;
    }
#endif

    return Chimera::Watchdog::IChannel::UNKNOWN;
  }


  Chimera::Watchdog::WChannel getWChannel( const std::uintptr_t address )
  {
#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    if ( address == reinterpret_cast<std::uintptr_t>( WWDG1_PERIPH ) )
    {
      return Chimera::Watchdog::WChannel::WATCHDOG0;
    }
#endif

    return Chimera::Watchdog::WChannel::UNKNOWN;
  }


  bool attachDriverInstances( IndependentDriver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != IWDG::NUM_IWDG_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_IWDG1_PERIPH_AVAILABLE )
    result |= driverList[ IWDG::IWDG1_RESOURCE_INDEX ].attach( IWDG1_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }


  bool attachDriverInstances( WindowDriver *const driverList, const size_t numDrivers )
  {
    /*-------------------------------------------------
    Reject bad inputs
    -------------------------------------------------*/
    if ( !driverList || !numDrivers || ( numDrivers != WWDG::NUM_WWDG_PERIPHS ) )
    {
      return false;
    }

    /*-------------------------------------------------
    Attach the drivers. The architecture of the LLD
    ensures the ordering and number is correct.
    -------------------------------------------------*/
    Chimera::Status_t result = Chimera::Status::OK;

#if defined( STM32_WWDG1_PERIPH_AVAILABLE )
    result |= driverList[ WWDG::WWDG1_RESOURCE_INDEX ].attach( WWDG1_PERIPH );
#endif

    return result == Chimera::Status::OK;
  }


  uint8_t calculatePrescaler( const size_t ms, const size_t clock, const size_t maxCount, const uint8_t *const actVal,
                              const Reg32_t *const regVal, const size_t len )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if( !ms || !clock || !maxCount || !actVal || !regVal || !len )
    {
      return 0;
    }

    /*------------------------------------------------
    Initialize algorithm variables
    ------------------------------------------------*/
    size_t clockPeriod_mS = 0;
    size_t maxTimeout_mS  = 0;
    size_t minTimeout_mS  = 0;
    uint8_t bestIdx       = 0;

    /*------------------------------------------------
    The desired prescaler is found when the max watchdog
    expiration period drops below the desired timeout.
    ------------------------------------------------*/
    for ( uint8_t i = 0; i < len; i++ )
    {
      clockPeriod_mS = static_cast<size_t>( ( 1000.0f / static_cast<float>( clock ) ) * static_cast<float>( actVal[ i ] ) );
      maxTimeout_mS  = clockPeriod_mS * maxCount;
      minTimeout_mS  = ( clockPeriod_mS * maxCount ) + 1;

      if ( ( minTimeout_mS < ms ) || ( ms < maxTimeout_mS ) )
      {
        bestIdx = i;
        break;
      }
    }

    return bestIdx;
  }


  Reg32_t calculateReload( const size_t ms, const size_t clock, const size_t minCount, const size_t maxCount,
                           const size_t prescaler )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    if( !ms || !clock || !prescaler || ( minCount >= maxCount ) )
    {
      return 0;
    }

    /*------------------------------------------------
    Initialize algorithm variables
    ------------------------------------------------*/
    float lowestError    = std::numeric_limits<float>::max();
    float calcTimeout_mS = std::numeric_limits<float>::max();
    float absError       = std::numeric_limits<float>::max();
    float clockPeriod_mS = ( 1000.0f / static_cast<float>( clock ) ) * static_cast<float>( prescaler );
    uint32_t reloadVal   = 0;

    /*------------------------------------------------
    Iterate through all counter values to figure out
    which one produces the closest timeout.
    ------------------------------------------------*/
    size_t countRange = maxCount - minCount;

    for ( size_t testVal = 0; testVal <= countRange; testVal++ )
    {
      calcTimeout_mS = clockPeriod_mS * static_cast<float>( testVal ) + 1.0f;
      absError       = fabs( static_cast<float>( ms ) - calcTimeout_mS );

      if ( absError < lowestError )
      {
        lowestError = absError;
        reloadVal   = testVal;
      }
    }

    return reloadVal + minCount;
  }

}    // namespace Thor::LLD::Watchdog
