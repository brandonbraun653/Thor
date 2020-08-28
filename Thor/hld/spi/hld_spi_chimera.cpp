/********************************************************************************
 *  File Name:
 *    hld_spi_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera SPI driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/spi>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/spi>

namespace Chimera::SPI::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::SPI::initialize();
  }

  Chimera::Status_t reset()
  {
    return Thor::SPI::reset();
  }

  ISPI_sPtr getDriver( const Channel channel )
  {
    return Thor::SPI::getDriver( channel );
  }

  Chimera::Status_t registerDriver( Chimera::SPI::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_SPI )
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
#endif /* THOR_HLD_SPI */
  }
}    // namespace Chimera::SPI::Backend
