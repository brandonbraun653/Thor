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
namespace HLD = Thor::Watchdog;
namespace LLD = Thor::LLD::Watchdog;

/*-------------------------------------------------------------------------------
Constants
-------------------------------------------------------------------------------*/
static constexpr size_t NUM_IDRIVERS = LLD::NUM_IWDG_PERIPHS;
static constexpr size_t NUM_WDRIVERS = LLD::NUM_WWDG_PERIPHS;


namespace Thor::Watchdog
{
  #if defined( THOR_HLD_WWDG )
  static size_t s_wwdg_driver_initialized;                   /**< Tracks the module level initialization state */
  static HLD::Window hld_wdriver[ NUM_WDRIVERS ];            /**< Driver objects */
  static HLD::WindowDriver_sPtr hld_wshared[ NUM_WDRIVERS ]; /**< Shared references to driver objects */

  Chimera::Status_t initializeWWDG()
  {
    /*-------------------------------------------------
    Prevent duplicate initialization
    -------------------------------------------------*/
    if( s_wwdg_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*------------------------------------------------
    Initialize local memory
    ------------------------------------------------*/
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    for ( size_t x = 0; x < NUM_WDRIVERS; x++ )
    {
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_WWDG )
      hld_wshared[ x ] = HLD::WindowDriver_sPtr( new HLD::Window() );
#else
      hld_wshared[ x ] = HLD::WindowDriver_sPtr( &hld_wdriver[ x ] );
#endif
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::WWDG::initialize();
    s_wwdg_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  Window::Window() : mChannel( Chimera::Watchdog::WChannel::UNKNOWN )
  {
  }


  Window::~Window()
  {
  }


  Chimera::Status_t Window::initialize( const Chimera::Watchdog::WChannel ch, const uint32_t timeout_mS, const uint8_t windowPercent )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    auto hwDriver = LLD::getDriver( ch );
    if( !hwDriver )
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
    uint32_t prescalerRegVal = hwDriver->calculatePrescaler( timeout_mS );
    uint32_t reloadRegVal    = hwDriver->calculateReload( timeout_mS, prescalerRegVal );
    uint32_t windowRegVal    = hwDriver->calculateWindow( timeout_mS, windowPercent, prescalerRegVal );

    /*-------------------------------------------------
    Configure the watchdog
    -------------------------------------------------*/
    hwDriver->enableClock();

    if ( ( hwDriver->setPrescaler( prescalerRegVal ) != Chimera::Status::OK ) ||
         ( hwDriver->setReload( reloadRegVal ) != Chimera::Status::OK ) ||
         ( hwDriver->setWindow( windowRegVal ) != Chimera::Status::OK ) )
    {
      return Chimera::Status::FAIL;
    }

    return Chimera::Status::OK;
  }

  Chimera::Status_t Window::start()
  {
    LLD::getDriver( ch )->start();
    return Chimera::Status::OK;
  }

  Chimera::Status_t Window::stop()
  {
    /*------------------------------------------------
    Once enabled, the watchdog cannot be stopped except by a system reset
    ------------------------------------------------*/
    return Chimera::Status::LOCKED;
  }

  Chimera::Status_t Window::kick()
  {
    LLD::getDriver( ch )->reload();
    return Chimera::Status::OK;
  }

  size_t Window::getTimeout()
  {
    return LLD::getDriver( ch )->getTimeout();
  }

  size_t Window::maxTimeout()
  {
    return LLD::getDriver( ch )->getMaxTimeout( Thor::LLD::WWDG::CFR::CLK_DIV_MIN );
  }

  size_t Window::minTimeout()
  {
    return LLD::getDriver( ch )->getMinTimeout( Thor::LLD::WWDG::CFR::CLK_DIV_MAX );
  }

  Chimera::Status_t Window::pauseOnDebugHalt( const bool enable )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  #endif /* THOR_HLD_WWDG */


  #if defined( THOR_HLD_IWDG )
  static size_t s_iwdg_driver_initialized;                   /**< Tracks the module level initialization state */
  static HLD::Independent hld_idriver[ NUM_IDRIVERS ];            /**< Driver objects */
  static HLD::IndependentDriver_sPtr hld_ishared[ NUM_IDRIVERS ]; /**< Shared references to driver objects */


  Chimera::Status_t initializeIWDG()
  {
    /*-------------------------------------------------
    Prevent duplicate initialization
    -------------------------------------------------*/
    if( s_iwdg_driver_initialized == Chimera::DRIVER_INITIALIZED_KEY )
    {
      return Chimera::Status::OK;
    }

    /*------------------------------------------------
    Initialize local memory
    ------------------------------------------------*/
    s_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;
    for ( size_t x = 0; x < NUM_IDRIVERS; x++ )
    {
#if defined( THOR_HLD_TEST ) || defined( THOR_HLD_TEST_IWDG )
      hld_ishared[ x ] = HLD::IndependentDriver_sPtr( new HLD::Independent() );
#else
      hld_ishared[ x ] = HLD::IndependentDriver_sPtr( &hld_idriver[ x ] );
#endif
    }

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::Watchdog::initialize();
    s_iwdg_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }


  /*-------------------------------------------------------------------------------
  Independent Watchdog Implementation
  -------------------------------------------------------------------------------*/
  Independent::Independent() : mChannel( Chimera::Watchdog::IChannel::UNKNOWN )
  {
  }


  Independent::~Independent()
  {
  }


  Chimera::Status_t Independent::initialize( Chimera::Watchdog::IChannel ch, const uint32_t timeout_mS )
  {
    /*-------------------------------------------------
    Input Protection
    -------------------------------------------------*/
    auto hwDriver = LLD::getDriver( ch );
    if( !hwDriver )
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
    uint32_t prescalerRegVal = hwDriver->calculatePrescaler( timeout_mS );
    uint32_t reloadRegVal    = hwDriver->calculateReload( timeout_mS, prescalerRegVal );

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


  Chimera::Status_t Independent::start()
  {
    LLD::getDriver( ch )->start();
    return Chimera::Status::OK;
  }


  Chimera::Status_t Independent::stop()
  {
    /*------------------------------------------------
    Once enabled, the watchdog cannot be stopped except by a system reset
    ------------------------------------------------*/
    return Chimera::Status::LOCKED;
  }


  Chimera::Status_t Independent::kick()
  {
    LLD::getDriver( ch )->reload();
    return Chimera::Status::OK;
  }


  size_t Independent::getTimeout()
  {
    return LLD::getDriver( ch )->getTimeout();
  }


  size_t Independent::maxTimeout()
  {
    return LLD::getDriver( ch )->getMaxTimeout( Thor::LLD::IWDG::PR::PRESCALE_MAX );
  }


  size_t Independent::minTimeout()
  {
    return LLD::getDriver( ch )->getMinTimeout( Thor::LLD::IWDG::PR::PRESCALE_MIN );
  }


  Chimera::Status_t Independent::pauseOnDebugHalt( const bool enable )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  #endif /* THOR_HLD_IWDG */

}    // namespace Thor::Watchdog
