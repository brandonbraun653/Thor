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

namespace Chimera::Watchdog::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    auto result = Chimera::Status::OK;

    result |= Thor::Watchdog::initializeIWDG();
    result |= Thor::Watchdog::initializeWWDG();

    return result;
  }

  Chimera::Status_t reset()
  {
    return Thor::Watchdog::reset();
  }

  Chimera::Watchdog::Driver_sPtr getDriver( const Channel channel )
  {
    return Thor::Watchdog::getDriver( channel );
  }

  Chimera::Status_t registerDriver( Chimera::Watchdog::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_WWDG ) || defined( THOR_HLD_IWDG )
    registry.isSupported  = true;
    registry.createShared = getDriver;
    registry.initialize   = initialize;
    registry.reset        = reset;
    return Chimera::Status::OK;
#else
    registry.isSupported  = false;
    registry.getDriver    = nullptr;
    registry.initialize   = nullptr;
    registry.reset        = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_HLD_WWDG || THOR_HLD_IWDG */
  }
}    // namespace Chimera::Watchdog::Backend