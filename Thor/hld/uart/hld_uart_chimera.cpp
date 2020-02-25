/********************************************************************************
 *  File Name:
 *    hld_uart_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera UART driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/uart>

/* Thor Includes */
#include <Thor/uart>

namespace Chimera::UART::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::UART::initialize();
  }

  Chimera::Status_t reset()
  {
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::UART::UART_sPtr create_shared_ptr()
  {
    return std::make_shared<Thor::UART::Driver>();
  }

  Chimera::UART::UART_uPtr create_unique_ptr()
  {
    return std::make_unique<Thor::UART::Driver>();
  }

  Chimera::Status_t registerDriver( Chimera::UART::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_UART )
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
#endif /* THOR_DRIVER_UART == 1*/
  }
}    // namespace Chimera::UART::Backend