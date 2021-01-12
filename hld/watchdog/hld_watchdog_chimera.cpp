/********************************************************************************
 *  File Name:
 *    hld_gpio_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera Watchdog driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/watchdog>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/watchdog>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>
#include <Thor/lld/interface/watchdog/watchdog_detail.hpp>
#include <Thor/lld/interface/watchdog/watchdog_types.hpp>


/*-------------------------------------------------------------------------------
Aliases
-------------------------------------------------------------------------------*/
namespace HLD = Thor::Watchdog;
namespace ILLD = Thor::LLD::IWDG;
namespace WLLD = Thor::LLD::WWDG;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_IDRIVERS = ILLD::NUM_IWDG_PERIPHS;
static constexpr size_t NUM_WDRIVERS = WLLD::NUM_WWDG_PERIPHS;

/*-------------------------------------------------------------------------------
Variables
-------------------------------------------------------------------------------*/
#if defined( THOR_HLD_IWDG )
static Chimera::Watchdog::IndependentDriver s_raw_Idriver[ NUM_IDRIVERS ];
#endif

#if defined( THOR_HLD_WWDG )
static Chimera::Watchdog::WindowDriver s_raw_Wdriver[ NUM_WDRIVERS ];
#endif


namespace Chimera::Watchdog::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    auto result = Chimera::Status::OK;
    return result;
  }


  Chimera::Status_t reset()
  {
    return Thor::Watchdog::reset();
  }


  Chimera::Watchdog::Independent_sPtr getDriver( const IChannel channel )
  {
#if defined( THOR_HLD_IWDG )
    auto idx = ::Thor::LLD::Watchdog::getResourceIndex( channel );

    if( idx < NUM_IDRIVERS )
    {
      return Chimera::Watchdog::Independent_sPtr( &s_raw_Idriver[ idx ] );
    }
    else
    {
      return nullptr;
    }
#else
    return nullptr;
#endif
  }


  Chimera::Watchdog::Window_sPtr getDriver( const WChannel channel )
  {
#if defined( THOR_HLD_WWDG )
    auto idx = ::Thor::LLD::Watchdog::getResourceIndex( channel );

    if ( idx < NUM_WDRIVERS )
    {
      return Chimera::Watchdog::Window_sPtr( &s_raw_Wdriver[ idx ] );
    }
    else
    {
      return nullptr;
    }
#else
    return nullptr;
#endif
  }


  Chimera::Status_t registerDriver( Chimera::Watchdog::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_WWDG ) || defined( THOR_HLD_IWDG )
    registry.isSupported = true;
    registry.initialize  = initialize;
    registry.reset       = reset;

#if defined( THOR_HLD_IWDG )
    registry.getIndependentDriver = getDriver;
#else
    registry.getIndependentDriver = nullptr;
#endif

#if defined( THOR_HLD_WWDG )
    registry.getWindowDriver = getDriver;
#else
    registry.getWindowDriver      = nullptr;
#endif
    return Chimera::Status::OK;
#else
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_HLD_WWDG || THOR_HLD_IWDG */
  }
}    // namespace Chimera::Watchdog::Backend


namespace Chimera::Watchdog
{
  /*-------------------------------------------------------------------------------
  Independent Driver Implementation
  -------------------------------------------------------------------------------*/
  IndependentDriver::IndependentDriver() : mDriver( nullptr )
  {
  }


  IndependentDriver::~IndependentDriver()
  {
  }


  /*-------------------------------------------------
  Interface: Hardware
  -------------------------------------------------*/
  Chimera::Status_t IndependentDriver::initialize( const IChannel ch, const uint32_t timeout_mS )
  {
    mDriver = reinterpret_cast<void *>( ::HLD::getDriver( ch ) );

    if ( mDriver )
    {
      return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->initialize( ch, timeout_mS );
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
  }


  Status_t IndependentDriver::start()
  {
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->start();
  }


  Status_t IndependentDriver::stop()
  {
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->stop();
  }


  Status_t IndependentDriver::kick()
  {
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->kick();
  }


  Status_t IndependentDriver::pauseOnDebugHalt( const bool enable )
  {
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->pauseOnDebugHalt( enable );
  }

  size_t IndependentDriver::getTimeout()
  {
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->getTimeout();
  }


  size_t IndependentDriver::maxTimeout()
  {
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->maxTimeout();
  }


  size_t IndependentDriver::minTimeout()
  {
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->minTimeout();
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void IndependentDriver::lock()
  {
    static_cast<HLD::IndependentDriver_rPtr>( mDriver )->lock();
  }


  void IndependentDriver::lockFromISR()
  {
    static_cast<HLD::IndependentDriver_rPtr>( mDriver )->lockFromISR();
  }


  bool IndependentDriver::try_lock_for( const size_t timeout )
  {
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->try_lock_for( timeout );
  }


  void IndependentDriver::unlock()
  {
    static_cast<HLD::IndependentDriver_rPtr>( mDriver )->unlock();
  }


  void IndependentDriver::unlockFromISR()
  {
    static_cast<HLD::IndependentDriver_rPtr>( mDriver )->unlockFromISR();
  }


  /*-------------------------------------------------------------------------------
  Window Driver Implementation
  -------------------------------------------------------------------------------*/
  WindowDriver::WindowDriver() : mDriver( nullptr )
  {
  }


  WindowDriver::~WindowDriver()
  {
  }


  /*-------------------------------------------------
  Interface: Hardware
  -------------------------------------------------*/
  Chimera::Status_t WindowDriver::initialize( const WChannel ch, const uint32_t timeout_mS, const uint8_t windowPercent )
  {
    mDriver = reinterpret_cast<void *>( HLD::getDriver( ch ) );

    if ( mDriver )
    {
      return static_cast<HLD::WindowDriver_rPtr>( mDriver )->initialize( ch, timeout_mS, windowPercent );
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
  }


  Status_t WindowDriver::start()
  {
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->start();
  }


  Status_t WindowDriver::stop()
  {
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->stop();
  }


  Status_t WindowDriver::kick()
  {
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->kick();
  }


  Status_t WindowDriver::pauseOnDebugHalt( const bool enable )
  {
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->pauseOnDebugHalt( enable );
  }

  size_t WindowDriver::getTimeout()
  {
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->getTimeout();
  }


  size_t WindowDriver::maxTimeout()
  {
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->maxTimeout();
  }


  size_t WindowDriver::minTimeout()
  {
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->minTimeout();
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void WindowDriver::lock()
  {
    static_cast<HLD::WindowDriver_rPtr>( mDriver )->lock();
  }


  void WindowDriver::lockFromISR()
  {
    static_cast<HLD::WindowDriver_rPtr>( mDriver )->lockFromISR();
  }


  bool WindowDriver::try_lock_for( const size_t timeout )
  {
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->try_lock_for( timeout );
  }


  void WindowDriver::unlock()
  {
    static_cast<HLD::WindowDriver_rPtr>( mDriver )->unlock();
  }


  void WindowDriver::unlockFromISR()
  {
    static_cast<HLD::WindowDriver_rPtr>( mDriver )->unlockFromISR();
  }


}    // namespace Chimera::Watchdog