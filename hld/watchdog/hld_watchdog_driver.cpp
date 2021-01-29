/********************************************************************************
 *  File Name:
 *    thor_watchdog.cpp
 *
 *  Description:
 *    Implementation of the hardware watchdog interface
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C/C++ Includes */
#include <array>
#include <cmath>
#include <cassert>
#include <limits>
#include <memory>

/* Aurora Includes */
#include <Aurora/constants>

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/watchdog>
#include <Thor/lld/interface/watchdog/watchdog_detail.hpp>
#include <Thor/lld/interface/watchdog/watchdog_intf.hpp>


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


namespace Thor::Watchdog
{
#if defined( THOR_HLD_IWDG ) || defined( THOR_HLD_WWDG )
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t reset()
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

#endif /* THOR_HLD_IWDG || THOR_HLD_WWDG */


#if defined( THOR_HLD_WWDG )
  static size_t s_wwdg_driver_initialized;                     /**< Tracks the module level initialization state */
  static ::HLD::WindowDriver hld_wdriver[ NUM_WDRIVERS ];      /**< Driver objects */
  static ::HLD::WindowDriver_rPtr hld_wshared[ NUM_WDRIVERS ]; /**< Shared references to driver objects */

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initializeWWDG()
  {
    /*-------------------------------------------------
    Prevent duplicate initialization
    -------------------------------------------------*/
    if ( s_wwdg_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*------------------------------------------------
    Initialize local memory
    ------------------------------------------------*/
    s_wwdg_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    for ( size_t x = 0; x < NUM_WDRIVERS; x++ )
    {
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_WWDG )
      hld_wshared[ x ] = ::HLD::WindowDriver_rPtr( new HLD::Window() );
#else
      hld_wshared[ x ] = ::HLD::WindowDriver_rPtr( &hld_wdriver[ x ] );
#endif
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::Watchdog::initializeWWDG();
    s_wwdg_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  WindowDriver_rPtr getDriver( const Chimera::Watchdog::WChannel channel )
  {
    if ( auto idx = LLD::Watchdog::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return &hld_wdriver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }

  /*-------------------------------------------------------------------------------
  Window Driver Implemenataion
  -------------------------------------------------------------------------------*/
  WindowDriver::WindowDriver() : mChannel( Chimera::Watchdog::WChannel::UNKNOWN )
  {
  }


  WindowDriver::~WindowDriver()
  {
  }


  Chimera::Status_t WindowDriver::initialize( const Chimera::Watchdog::WChannel ch, const uint32_t timeout_mS,
                                              const uint8_t windowPercent )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    auto hwDriver = Thor::LLD::Watchdog::getDriver( ch );
    if ( !hwDriver )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
    else
    {
      mChannel = ch;
    }

    /*-------------------------------------------------
    Grab the peripheral clock driving the WWDG
    -------------------------------------------------*/
    size_t wwdgClockFreq = Thor::LLD::Watchdog::getWWDGClockFrequency();

    /*-------------------------------------------------
    Calculate the operating parameters
    -------------------------------------------------*/
    /* clang-format off */
    auto idx = Thor::LLD::Watchdog::calculatePrescaler(
      timeout_mS,
      wwdgClockFreq,
      WLLD::COUNTER_MIN,
      WLLD::COUNTER_MAX,
      WLLD::DecimalPrescalers,
      WLLD::RegisterPrescalers,
      WLLD::NumPrescalers
    );

    uint32_t prescalerRegVal = Thor::LLD::WWDG::RegisterPrescalers[ idx ];
    uint32_t reloadRegVal = Thor::LLD::Watchdog::calculateReload(
      timeout_mS,
      wwdgClockFreq,
      WLLD::COUNTER_MIN,
      WLLD::COUNTER_MAX,
      WLLD::DecimalPrescalers[ idx ]
    );
    /* clang-format on */

    /*-------------------------------------------------
    Ensure the requested timeout can be achieved
    -------------------------------------------------*/
    if ( !( timeout_mS >= hwDriver->getMinTimeout( prescalerRegVal ) ) ||
         !( timeout_mS <= hwDriver->getMaxTimeout( prescalerRegVal ) ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Configure the watchdog
    -------------------------------------------------*/
    hwDriver->enableClock();

    if ( ( hwDriver->setPrescaler( prescalerRegVal ) != Chimera::Status::OK ) ||
         ( hwDriver->setReload( reloadRegVal ) != Chimera::Status::OK ) ||
         ( hwDriver->setWindow( windowPercent ) != Chimera::Status::OK ) )
    {
      return Chimera::Status::FAIL;
    }

    return Chimera::Status::OK;
  }

  Chimera::Status_t WindowDriver::start()
  {
    Thor::LLD::Watchdog::getDriver( mChannel )->start();
    return Chimera::Status::OK;
  }

  Chimera::Status_t WindowDriver::stop()
  {
    /*------------------------------------------------
    Once enabled, the watchdog cannot be stopped except by a system reset
    ------------------------------------------------*/
    return Chimera::Status::LOCKED;
  }

  Chimera::Status_t WindowDriver::kick()
  {
    Thor::LLD::Watchdog::getDriver( mChannel )->reload();
    return Chimera::Status::OK;
  }

  size_t WindowDriver::getTimeout()
  {
    return Thor::LLD::Watchdog::getDriver( mChannel )->getTimeout();
  }

  size_t WindowDriver::maxTimeout()
  {
    return Thor::LLD::Watchdog::getDriver( mChannel )->getMaxTimeout( WLLD::MAX_PRESCALE );
  }

  size_t WindowDriver::minTimeout()
  {
    return Thor::LLD::Watchdog::getDriver( mChannel )->getMinTimeout( WLLD::MIN_PRESCALE );
  }

  Chimera::Status_t WindowDriver::pauseOnDebugHalt( const bool enable )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

#endif /* THOR_HLD_WWDG */


#if defined( THOR_HLD_IWDG )
  static size_t s_iwdg_driver_initialized;                          /**< Tracks the module level initialization state */
  static ::HLD::IndependentDriver hld_idriver[ NUM_IDRIVERS ];      /**< Driver objects */
  static ::HLD::IndependentDriver_rPtr hld_ishared[ NUM_IDRIVERS ]; /**< Shared references to driver objects */

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initializeIWDG()
  {
    /*-------------------------------------------------
    Prevent duplicate initialization
    -------------------------------------------------*/
    if ( s_iwdg_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*------------------------------------------------
    Initialize local memory
    ------------------------------------------------*/
    s_iwdg_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    for ( size_t x = 0; x < NUM_IDRIVERS; x++ )
    {
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_IWDG )
      hld_ishared[ x ] = ::HLD::IndependentDriver_rPtr( new HLD::Independent() );
#else
      hld_ishared[ x ] = ::HLD::IndependentDriver_rPtr( &hld_idriver[ x ] );
#endif
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::Watchdog::initializeIWDG();
    s_iwdg_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  IndependentDriver_rPtr getDriver( const Chimera::Watchdog::IChannel channel )
  {
    if ( auto idx = LLD::Watchdog::getResourceIndex( channel ); idx != ::Thor::LLD::INVALID_RESOURCE_INDEX )
    {
      return &hld_idriver[ idx ];
    }
    else
    {
      return nullptr;
    }
  }


  /*-------------------------------------------------------------------------------
  Independent Watchdog Implementation
  -------------------------------------------------------------------------------*/
  IndependentDriver::IndependentDriver() : mChannel( Chimera::Watchdog::IChannel::UNKNOWN )
  {
  }


  IndependentDriver::~IndependentDriver()
  {
  }


  Chimera::Status_t IndependentDriver::initialize( Chimera::Watchdog::IChannel ch, const uint32_t timeout_mS )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    auto hwDriver = Thor::LLD::Watchdog::getDriver( ch );
    if ( !hwDriver )
    {
      return Chimera::Status::NOT_SUPPORTED;
    }
    else
    {
      mChannel = ch;
    }

    /*-------------------------------------------------
    Calculate the operating parameters
    -------------------------------------------------*/
    /* clang-format off */
    uint32_t idx = Thor::LLD::Watchdog::calculatePrescaler(
      timeout_mS,
      Thor::LLD::IWDG::PERIPH_CLOCK_FREQ_HZ,
      Thor::LLD::IWDG::COUNTER_MIN,
      Thor::LLD::IWDG::COUNTER_MAX,
      Thor::LLD::IWDG::DecimalPrescalers,
      Thor::LLD::IWDG::RegisterPrescalers,
      Thor::LLD::IWDG::NumPrescalers
    );

    uint32_t prescalerRegVal = Thor::LLD::IWDG::RegisterPrescalers[ idx ];

    uint32_t reloadRegVal = Thor::LLD::Watchdog::calculateReload(
      timeout_mS,
      Thor::LLD::IWDG::PERIPH_CLOCK_FREQ_HZ,
      Thor::LLD::IWDG::COUNTER_MIN,
      Thor::LLD::IWDG::COUNTER_MAX,
      Thor::LLD::IWDG::DecimalPrescalers[ idx ]
    );
    /* clang-format on */

    /*-------------------------------------------------
    Ensure the requested timeout can be achieved
    -------------------------------------------------*/
    if ( !( ( 1000 * timeout_mS ) >= hwDriver->getMinTimeout( Thor::LLD::IWDG::DecimalPrescalers[ idx ] ) ) ||
         !( ( 1000 * timeout_mS ) <= hwDriver->getMaxTimeout( Thor::LLD::IWDG::DecimalPrescalers[ idx ] ) ) )
    {
      return Chimera::Status::INVAL_FUNC_PARAM;
    }

    /*-------------------------------------------------
    Configure the watchdog
    -------------------------------------------------*/
    hwDriver->enableClock();

    if ( ( hwDriver->setPrescaler( prescalerRegVal ) != Chimera::Status::OK ) ||
         ( hwDriver->setReload( reloadRegVal ) != Chimera::Status::OK ) )
    {
      return Chimera::Status::FAIL;
    }

    return Chimera::Status::OK;
  }


  Chimera::Status_t IndependentDriver::start()
  {
    Thor::LLD::Watchdog::getDriver( mChannel )->start();
    return Chimera::Status::OK;
  }


  Chimera::Status_t IndependentDriver::stop()
  {
    /*------------------------------------------------
    Once enabled, the watchdog cannot be stopped except by a system reset
    ------------------------------------------------*/
    return Chimera::Status::LOCKED;
  }


  Chimera::Status_t IndependentDriver::kick()
  {
    Thor::LLD::Watchdog::getDriver( mChannel )->reload();
    return Chimera::Status::OK;
  }


  size_t IndependentDriver::getTimeout()
  {
    return Thor::LLD::Watchdog::getDriver( mChannel )->getTimeout();
  }


  size_t IndependentDriver::maxTimeout()
  {
    return Thor::LLD::Watchdog::getDriver( mChannel )->getMaxTimeout( ILLD::PR_MAX_PRESCALE );
  }


  size_t IndependentDriver::minTimeout()
  {
    return Thor::LLD::Watchdog::getDriver( mChannel )->getMinTimeout( ILLD::PR_MIN_PRESCALE );
  }


  Chimera::Status_t IndependentDriver::pauseOnDebugHalt( const bool enable )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

#endif /* THOR_HLD_IWDG */

}    // namespace Thor::Watchdog
