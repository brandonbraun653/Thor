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

namespace Chimera::GPIO::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::GPIO::initialize();
  }

  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }

  Chimera::GPIO::GPIO_sPtr create_shared_ptr()
  {
    return std::make_shared<Thor::GPIO::Driver>();
  }

  Chimera::GPIO::GPIO_uPtr create_unique_ptr()
  {
    return std::make_unique<Thor::GPIO::Driver>();
  }

  Chimera::Status_t registerDriver( Chimera::GPIO::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_GPIO )
    registry.isSupported  = true;
    registry.createShared = create_shared_ptr;
    registry.createUnique = create_unique_ptr;
    registry.initialize   = initialize;
    registry.reset        = reset;
    return Chimera::Status::OK;
#else
    registry.isSupported  = false;
    registry.createShared = nullptr;
    registry.createUnique = nullptr;
    registry.initialize   = nullptr;
    registry.reset        = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_DRIVER_GPIO == 1*/
  }
}    // namespace Chimera::GPIO::Backend