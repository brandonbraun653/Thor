/********************************************************************************
 *   File Name:
 *       thor_watchdog.cpp
 *
 *   Description:
 *       Implementation of the hardware watchdog interface
 *
 *   2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C/C++ Includes */
#include <array>
#include <cmath>
#include <cassert>
#include <limits>
#include <memory>

/* Aurora Includes */
#include <Aurora/constants/common.hpp>

/* Thor Includes */
#include <Thor/watchdog.hpp>
#include <Thor/drivers/watchdog.hpp>

namespace Chimera::Watchdog
{
  Chimera::Status_t initialize()
  {
    Chimera::Status_t resultWWDG = Chimera::CommonStatusCodes::OK;
    Chimera::Status_t resultIWDG = Chimera::CommonStatusCodes::OK;

    #if defined( THOR_DRIVER_WWDG ) && ( THOR_DRIVER_WWDG == 1 )
    resultWWDG = Thor::Watchdog::initializeWWDG();
    #endif 

    #if defined( THOR_DRIVER_IWDG ) && ( THOR_DRIVER_IWDG == 1 )
    resultIWDG = Thor::Watchdog::initializeIWDG();
    #endif 

    if ( ( resultIWDG == Chimera::CommonStatusCodes::OK ) && ( resultWWDG == Chimera::CommonStatusCodes::OK ) )
    {
      return Chimera::CommonStatusCodes::OK;
    }
    else
    {
      return Chimera::CommonStatusCodes::FAIL;
    }
  }
}


namespace Thor::Watchdog
{
  /*------------------------------------------------
  Window Watchdog Driver
  ------------------------------------------------*/
#if defined( THOR_DRIVER_WWDG ) && ( THOR_DRIVER_WWDG == 1 )

  static size_t s_wwdg_driver_initialized;

  Chimera::Status_t initializeWWDG()
  {
    s_wwdg_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::Driver::WWDG::initialize();


    s_wwdg_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }

  Window::Window() : currentPrescaler( 0u )
  {
    hwDriver = std::make_unique<Thor::Driver::WWDG::Driver>( Thor::Driver::WWDG::WWDG1_PERIPH );
  }

  Window::~Window()
  {
  }

  Chimera::Status_t Window::initialize( const uint32_t timeout_mS, const uint8_t windowPercent )
  {
    uint32_t prescalerRegVal = hwDriver->calculatePrescaler( timeout_mS );
    uint32_t reloadRegVal    = hwDriver->calculateReload( timeout_mS, prescalerRegVal );
    uint32_t windowRegVal    = hwDriver->calculateWindow( timeout_mS, windowPercent, prescalerRegVal );

    hwDriver->enableClock();

    if ( ( hwDriver->setPrescaler( prescalerRegVal ) != Chimera::CommonStatusCodes::OK ) ||
         ( hwDriver->setReload( reloadRegVal ) != Chimera::CommonStatusCodes::OK ) ||
         ( hwDriver->setWindow( windowRegVal ) != Chimera::CommonStatusCodes::OK ) )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Window::start()
  {
    hwDriver->start();
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Window::stop()
  {
    /*------------------------------------------------
    Once enabled, the watchdog cannot be stopped except by a system reset
    ------------------------------------------------*/
    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t Window::kick()
  {
    hwDriver->reload();
    return Chimera::CommonStatusCodes::OK;
  }

  size_t Window::getTimeout()
  {
    return hwDriver->getTimeout();
  }

  size_t Window::maxTimeout()
  {
    return hwDriver->getMaxTimeout( Thor::Driver::WWDG::CFR::CLK_DIV_MIN );
  }

  size_t Window::minTimeout()
  {
    return hwDriver->getMinTimeout( Thor::Driver::WWDG::CFR::CLK_DIV_MAX );
  }

  Chimera::Status_t Window::pauseOnDebugHalt( const bool enable )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

#endif /* THOR_DRIVER_WWDG */

  /*------------------------------------------------
  Independent Watchdog Driver
  ------------------------------------------------*/
#if defined( THOR_DRIVER_IWDG ) && ( THOR_DRIVER_IWDG == 1 )

  static size_t s_iwdg_driver_initialized;

  Chimera::Status_t initializeIWDG()
  {
    s_iwdg_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::Driver::IWDG::initialize();


    s_iwdg_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::CommonStatusCodes::OK;
  }

  Independent::Independent() : currentPrescaler( 0u )
  {
    hwDriver = std::make_unique<Thor::Driver::IWDG::Driver>( Thor::Driver::IWDG::IWDG1_PERIPH );
  }

  Independent::~Independent()
  {
  }

  Chimera::Status_t Independent::initialize( const uint32_t timeout_mS, const uint8_t windowPercent )
  {
    uint32_t prescalerRegVal = hwDriver->calculatePrescaler( timeout_mS );
    uint32_t reloadRegVal    = hwDriver->calculateReload( timeout_mS, prescalerRegVal );

    hwDriver->start();

    if ( ( hwDriver->setPrescaler( prescalerRegVal ) != Chimera::CommonStatusCodes::OK ) ||
         ( hwDriver->setReload( reloadRegVal ) != Chimera::CommonStatusCodes::OK ) )
    {
      return Chimera::CommonStatusCodes::FAIL;
    }

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Independent::start()
  {
    hwDriver->start();
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Independent::stop()
  {
    /*------------------------------------------------
    Once enabled, the watchdog cannot be stopped except by a system reset
    ------------------------------------------------*/
    return Chimera::CommonStatusCodes::LOCKED;
  }

  Chimera::Status_t Independent::kick()
  {
    hwDriver->reload();
    return Chimera::CommonStatusCodes::OK;
  }

  size_t Independent::getTimeout()
  {
    return hwDriver->getTimeout();
  }

  size_t Independent::maxTimeout()
  {
    return hwDriver->getMaxTimeout( Thor::Driver::IWDG::PR::PRESCALE_MAX );
  }

  size_t Independent::minTimeout()
  {
    return hwDriver->getMinTimeout( Thor::Driver::IWDG::PR::PRESCALE_MIN );
  }

  Chimera::Status_t Independent::pauseOnDebugHalt( const bool enable )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

#endif /* THOR_DRIVER_IWDG */

}    // namespace Thor::Watchdog
