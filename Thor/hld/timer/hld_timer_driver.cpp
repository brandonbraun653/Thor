/********************************************************************************
 *  File Name:
 *    hld_timer_driver.cpp
 *
 *  Description:
 *    Driver implementation
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* STL Includes */

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/thread>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/timer>
#include <Thor/hld/common/types.hpp>
#include <Thor/hld/timer/hld_timer_prv_driver.hpp>
#include <Thor/lld/interface/timer/timer_intf.hpp>
#include <Thor/lld/interface/timer/timer_detail.hpp>


#if defined( THOR_HLD_TIMER )

namespace Thor::TIMER
{
  // Tracks if the module data has been initialized correctly
  static size_t s_driver_initialized;

  /*-------------------------------------------------------------------------------
  Chimera Free Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initializeModule()
  {
    /*------------------------------------------------
    Prevent re-initialization from occurring
    ------------------------------------------------*/
    auto result = Chimera::Status::OK;
    if ( s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return result;
    }

    /*------------------------------------------------
    Initialize HLD module
    ------------------------------------------------*/
    result |= initAdvancedDriverModule();
    result |= initBasicDriverModule();
    result |= initGeneralDriverModule();
    result |= initLowPowerDriverModule();

    /*------------------------------------------------
    Initialize the LLD module
    ------------------------------------------------*/
    result |= Thor::LLD::TIMER::initializeModule();

    s_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return result;
  }

  Chimera::Status_t resetModule()
  {
    return Chimera::Status::OK;
  }

  void incrementSystemTick()
  {
    Thor::LLD::TIMER::incrementSystemTick();
  }

  size_t millis()
  {
    return Thor::LLD::TIMER::millis();
  }

  size_t micros()
  {
    return millis() * 1000;
  }

  void delayMilliseconds( const size_t ms )
  {
    Thor::LLD::TIMER::delayMilliseconds( ms );
  }

  void delayMicroseconds( const size_t us )
  {
    Thor::LLD::TIMER::delayMicroseconds( us );
  }

  /*-------------------------------------------------------------------------------
  Driver Free Functions
  -------------------------------------------------------------------------------*/
  bool isInitialized()
  {
    return s_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY;
  }

  static bool getHLDResourceData( const Chimera::Timer::Peripheral peripheral, Thor::HLD::RIndex &index, Thor::LLD::TIMER::Type &type )
  {
    using namespace Thor::LLD::TIMER;

    /*-------------------------------------------------
    Get the LLD resource index for the driver, if it's supported.
    This will allow us to grab other pieces of needed data.
    -------------------------------------------------*/
    auto hld_resource_map = PeripheralToHLDResourceIndex.at( peripheral );
    auto lld_resource_map = PeripheralToLLDResourceIndex.at( peripheral );

    /*-------------------------------------------------
    Using the resource index, grab to the device descriptor
    -------------------------------------------------*/
    auto hld_resource_index = hld_resource_map.second;
    auto lld_resource_index = lld_resource_map.second;
    const DeviceDescription * pDeviceDescriptor = getPeripheralDescriptor( lld_resource_index );


    type = pDeviceDescriptor->timerType;
    index = hld_resource_index;

    return true;
  }

  Chimera::Timer::ITimer_rPtr lookUpRawPointer( const Chimera::Timer::Peripheral peripheral )
  {
    /*-------------------------------------------------
    Due to Thor implementing a persistent driver model, if the
    shared_ptr exists, it will never be deleted. Giving out the
    raw pointer is no big deal.
    -------------------------------------------------*/
    if( auto shared_view = lookUpSharedPointer( peripheral ); shared_view )
    {
      return shared_view.get();
    }

    return nullptr;
  }

  Chimera::Timer::ITimer_sPtr lookUpSharedPointer( const Chimera::Timer::Peripheral peripheral )
  {
    using namespace Thor::LLD::TIMER;

    /*-------------------------------------------------
    Grab the lookup data
    -------------------------------------------------*/
    HLD::RIndex hld_index = HLD::RIndex( 0 );
    Type hld_type = Type::INVALID;

    if( !getHLDResourceData( peripheral, hld_index, hld_type ))
    {
      return nullptr;
    }

    /*-------------------------------------------------
    Grab the driver, implicitly converting to the base type.
    If one does not exist yet, it will be created.
    -------------------------------------------------*/
    switch( hld_type )
    {
      case Type::ADVANCED_TIMER:
        initAdvancedDriverObject( hld_index );
        return hld_advanced_drivers[ hld_index.value() ];
        break;

      case Type::BASIC_TIMER:
        initBasicDriverObject( hld_index );
        return hld_basic_drivers[ hld_index.value() ];
        break;

      case Type::GENERAL_PURPOSE_TIMER:
        initGeneralDriverObject( hld_index );
        return getGeneralDriverObject( hld_index );
        break;

      case Type::LOW_POWER_TIMER:
        initLowPowerDriverObject( hld_index );
        return hld_low_power_drivers[ hld_index.value() ];
        break;

      default:
        return nullptr;
        break;
    }
  }
}

#endif /* THOR_HLD_TIMER */
