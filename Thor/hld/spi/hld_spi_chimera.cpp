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
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::SPI::ISPI_sPtr create_shared_ptr()
  {
    return std::make_shared<Thor::SPI::Driver>();
  }

  Chimera::SPI::ISPI_uPtr create_unique_ptr()
  {
    return std::make_unique<Thor::SPI::Driver>();
  }

  Chimera::Status_t registerDriver( Chimera::SPI::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_SPI )
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
#endif /* THOR_DRIVER_SPI == 1*/
  }
}    // namespace Chimera::SPI::Backend