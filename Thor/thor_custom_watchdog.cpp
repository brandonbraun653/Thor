/********************************************************************************
 *   File Name:
 *       thor_watchdog.cpp
 *
 *   Description:
 *       Implementation of the hardware watchdog interface
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C/C++ Includes */
#include <array>
#include <cmath>
#include <cassert>
#include <limits>
#include <memory>

/* Thor Includes */
#include <Thor/watchdog.hpp>

namespace Thor::Watchdog
{
  /*------------------------------------------------
  Window Watchdog Driver
  ------------------------------------------------*/
#if defined( THOR_DRIVER_WWDG ) && ( THOR_DRIVER_WWDG == 1 )

  Window::Window() : currentPrescaler( 0u )
  {
    hwDriver = std::make_unique<Thor::Driver::WWDG::Driver>( Thor::Driver::WWDG::WWDG_PERIPH );
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

#endif  /* THOR_DRIVER_WWDG */

  /*------------------------------------------------
  Independent Watchdog Driver 
  ------------------------------------------------*/
#if defined( THOR_DRIVER_IWDG ) && ( THOR_DRIVER_IWDG == 1 )

  Independent::Independent() : currentPrescaler( 0u )
  {
    hwDriver = std::make_unique<Thor::Driver::IWDG::Driver>( Thor::Driver::IWDG::IWDG_PERIPH );
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

#endif  /* THOR_DRIVER_IWDG */

}    // namespace Thor::Watchdog
