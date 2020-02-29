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
  Chimera::Status_t initialize()
  {
    auto result = Chimera::CommonStatusCodes::OK;

    result |= Thor::Watchdog::initializeIWDG();
    result |= Thor::Watchdog::initializeWWDG();

    return result;
  }

  Chimera::Status_t reset()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Watchdog::Watchdog_sPtr create_shared_ptr()
  {
    return std::make_shared<Thor::Watchdog::Independent>();
  }

  Chimera::Watchdog::Watchdog_uPtr create_unique_ptr()
  {
    return std::make_unique<Thor::Watchdog::Independent>();
  }

  Chimera::Status_t registerDriver( Chimera::Watchdog::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_WWDG ) || defined( THOR_HLD_IWDG )
    registry.isSupported  = true;
    registry.createShared = create_shared_ptr;
    registry.createUnique = create_unique_ptr;
    registry.initialize   = initialize;
    registry.reset        = reset;
    return Chimera::CommonStatusCodes::OK;
#else
    registry.isSupported  = false;
    registry.createShared = nullptr;
    registry.createUnique = nullptr;
    registry.initialize   = nullptr;
    registry.reset        = nullptr;
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
#endif /* THOR_DRIVER_Watchdog == 1*/
  }
}    // namespace Chimera::Watchdog::Backend