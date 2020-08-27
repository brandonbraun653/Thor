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
#include <Aurora/constants/common.hpp>

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/watchdog>
#include <Thor/lld/interface/watchdog/watchdog.hpp>

// namespace Chimera::Watchdog::Backend
// {
//   Chimera::Status_t prjInitialize()
//   {
//     Chimera::Status_t resultWWDG = Chimera::Status::OK;
//     Chimera::Status_t resultIWDG = Chimera::Status::OK;

//     resultWWDG = Thor::Watchdog::initializeWWDG();
//     resultIWDG = Thor::Watchdog::initializeIWDG();

//     if ( ( resultIWDG == Chimera::Status::OK ) && ( resultWWDG == Chimera::Status::OK ) )
//     {
//       return Chimera::Status::OK;
//     }
//     else
//     {
//       return Chimera::Status::FAIL;
//     }
//   }
// }



namespace Thor::Watchdog
{

  #if defined( THOR_HLD_WWDG )
  /*------------------------------------------------
  Window Watchdog Driver
  ------------------------------------------------*/
  static size_t s_wwdg_driver_initialized;

  Chimera::Status_t initializeWWDG()
  {
    s_wwdg_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::WWDG::initialize();


    s_wwdg_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }

  Window::Window() : currentPrescaler( 0u )
  {
    hwDriver = std::make_unique<Thor::LLD::WWDG::Driver>( Thor::LLD::WWDG::WWDG1_PERIPH );
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
    hwDriver->start();
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
    hwDriver->reload();
    return Chimera::Status::OK;
  }

  size_t Window::getTimeout()
  {
    return hwDriver->getTimeout();
  }

  size_t Window::maxTimeout()
  {
    return hwDriver->getMaxTimeout( Thor::LLD::WWDG::CFR::CLK_DIV_MIN );
  }

  size_t Window::minTimeout()
  {
    return hwDriver->getMinTimeout( Thor::LLD::WWDG::CFR::CLK_DIV_MAX );
  }

  Chimera::Status_t Window::pauseOnDebugHalt( const bool enable )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  #endif /* THOR_HLD_WWDG */

  #if defined( THOR_HLD_IWDG )
  /*------------------------------------------------
  Independent Watchdog Driver
  ------------------------------------------------*/
  static size_t s_iwdg_driver_initialized;

  Chimera::Status_t initializeIWDG()
  {
    s_iwdg_driver_initialized = ~Chimera::DRIVER_INITIALIZED_KEY;

    /*------------------------------------------------
    Initialize the low level driver
    ------------------------------------------------*/
    Thor::LLD::IWDG::initialize();


    s_iwdg_driver_initialized = Chimera::DRIVER_INITIALIZED_KEY;
    return Chimera::Status::OK;
  }

  Independent::Independent() : currentPrescaler( 0u )
  {
    hwDriver = std::make_unique<Thor::LLD::IWDG::Driver>( Thor::LLD::IWDG::IWDG1_PERIPH );
  }

  Independent::~Independent()
  {
  }

  Chimera::Status_t Independent::initialize( const uint32_t timeout_mS, const uint8_t windowPercent )
  {
    uint32_t prescalerRegVal = hwDriver->calculatePrescaler( timeout_mS );
    uint32_t reloadRegVal    = hwDriver->calculateReload( timeout_mS, prescalerRegVal );

    hwDriver->start();

    if ( ( hwDriver->setPrescaler( prescalerRegVal ) != Chimera::Status::OK ) ||
         ( hwDriver->setReload( reloadRegVal ) != Chimera::Status::OK ) )
    {
      return Chimera::Status::FAIL;
    }

    return Chimera::Status::OK;
  }

  Chimera::Status_t Independent::start()
  {
    hwDriver->start();
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
    hwDriver->reload();
    return Chimera::Status::OK;
  }

  size_t Independent::getTimeout()
  {
    return hwDriver->getTimeout();
  }

  size_t Independent::maxTimeout()
  {
    return hwDriver->getMaxTimeout( Thor::LLD::IWDG::PR::PRESCALE_MAX );
  }

  size_t Independent::minTimeout()
  {
    return hwDriver->getMinTimeout( Thor::LLD::IWDG::PR::PRESCALE_MIN );
  }

  Chimera::Status_t Independent::pauseOnDebugHalt( const bool enable )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  #endif /* THOR_HLD_IWDG */

}    // namespace Thor::Watchdog
