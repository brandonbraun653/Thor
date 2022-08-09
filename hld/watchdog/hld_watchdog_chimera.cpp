/********************************************************************************
 *  File Name:
 *    hld_gpio_chimera.cpp
 *
 *  Description:
 *    Implementation of Chimera Watchdog driver hooks
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/assert>
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
namespace HLD  = Thor::Watchdog;
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

#if defined( THOR_HLD_IWDG )
    result |= Thor::Watchdog::initializeIWDG();
#endif
#if defined( THOR_HLD_IWDG )
    result |= Thor::Watchdog::initializeWWDG();
#endif

    return result;
  }


  Chimera::Status_t reset()
  {
#if defined( THOR_HLD_IWDG )
    return Thor::Watchdog::reset();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Chimera::Watchdog::Independent_sPtr getDriver( const IChannel channel )
  {
#if defined( THOR_HLD_IWDG )
    auto idx = ::Thor::LLD::Watchdog::getResourceIndex( channel );

    if ( idx < NUM_IDRIVERS )
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
#if defined( THOR_HLD_IWDG )
    mDriver = reinterpret_cast<void *>( ::HLD::getDriver( ch ) );
    RT_DBG_ASSERT( mDriver );

    if ( mDriver )
    {
      return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->initialize( ch, timeout_mS );
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
#else
      return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Status_t IndependentDriver::start()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->start();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Status_t IndependentDriver::stop()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->stop();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Status_t IndependentDriver::kick()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->kick();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Status_t IndependentDriver::pauseOnDebugHalt( const bool enable )
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->pauseOnDebugHalt( enable );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }

  size_t IndependentDriver::getTimeout()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->getTimeout();
#else
    return 0;
#endif
  }


  size_t IndependentDriver::maxTimeout()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->maxTimeout();
#else
    return 0;
#endif
  }


  size_t IndependentDriver::minTimeout()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->minTimeout();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void IndependentDriver::lock()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    static_cast<HLD::IndependentDriver_rPtr>( mDriver )->lock();
#endif
  }


  void IndependentDriver::lockFromISR()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    static_cast<HLD::IndependentDriver_rPtr>( mDriver )->lockFromISR();
#endif
  }


  bool IndependentDriver::try_lock_for( const size_t timeout )
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::IndependentDriver_rPtr>( mDriver )->try_lock_for( timeout );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  void IndependentDriver::unlock()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    static_cast<HLD::IndependentDriver_rPtr>( mDriver )->unlock();
#endif
  }


  void IndependentDriver::unlockFromISR()
  {
#if defined( THOR_HLD_IWDG )
    RT_DBG_ASSERT( mDriver );
    static_cast<HLD::IndependentDriver_rPtr>( mDriver )->unlockFromISR();
#endif
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
#if defined( THOR_HLD_WWDG )
    mDriver = reinterpret_cast<void *>( HLD::getDriver( ch ) );
    RT_DBG_ASSERT( mDriver );

    if ( mDriver )
    {
      return static_cast<HLD::WindowDriver_rPtr>( mDriver )->initialize( ch, timeout_mS, windowPercent );
    }
    else
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Status_t WindowDriver::start()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->start();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Status_t WindowDriver::stop()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->stop();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Status_t WindowDriver::kick()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->kick();
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }


  Status_t WindowDriver::pauseOnDebugHalt( const bool enable )
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->pauseOnDebugHalt( enable );
#else
    return Chimera::Status::NOT_SUPPORTED;
#endif
  }

  size_t WindowDriver::getTimeout()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->getTimeout();
#else
    return 0;
#endif
  }


  size_t WindowDriver::maxTimeout()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->maxTimeout();
#else
    return 0;
#endif
  }


  size_t WindowDriver::minTimeout()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->minTimeout();
#else
    return 0;
#endif
  }


  /*-------------------------------------------------
  Interface: Lockable
  -------------------------------------------------*/
  void WindowDriver::lock()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    static_cast<HLD::WindowDriver_rPtr>( mDriver )->lock();
#endif
  }


  void WindowDriver::lockFromISR()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    static_cast<HLD::WindowDriver_rPtr>( mDriver )->lockFromISR();
#endif
  }


  bool WindowDriver::try_lock_for( const size_t timeout )
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    return static_cast<HLD::WindowDriver_rPtr>( mDriver )->try_lock_for( timeout );
#else
    return false;
#endif
  }


  void WindowDriver::unlock()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    static_cast<HLD::WindowDriver_rPtr>( mDriver )->unlock();
#endif
  }


  void WindowDriver::unlockFromISR()
  {
#if defined( THOR_HLD_WWDG )
    RT_DBG_ASSERT( mDriver );
    static_cast<HLD::WindowDriver_rPtr>( mDriver )->unlockFromISR();
#endif
  }


}    // namespace Chimera::Watchdog