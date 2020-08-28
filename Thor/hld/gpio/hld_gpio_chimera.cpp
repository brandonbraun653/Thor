/********************************************************************************
 *  File Name:
 *    hld_gpio_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera GPIO driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/gpio>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/gpio>
#include <Thor/hld/gpio/hld_gpio_chimera.hpp>

namespace Chimera::GPIO::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::GPIO::initialize();
  }

  Chimera::Status_t reset()
  {
    return Thor::GPIO::reset();
  }

  Chimera::GPIO::IGPIO_sPtr getDriver( const Port port )
  {
    return Thor::GPIO::getDriver( port );
  }

  Chimera::Status_t registerDriver( Chimera::GPIO::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_GPIO )
    registry.isSupported = true;
    registry.getDriver   = getDriver;
    registry.initialize  = initialize;
    registry.reset       = reset;
    return Chimera::Status::OK;
#else
    registry.isSupported = false;
    registry.getDriver   = nullptr;
    registry.initialize  = nullptr;
    registry.reset       = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
#endif    // THOR_HLD_GPIO
  }
}    // namespace Chimera::GPIO::Backend