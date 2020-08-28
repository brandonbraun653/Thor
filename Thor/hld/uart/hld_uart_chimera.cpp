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
#include <Thor/cfg>
#include <Thor/uart>
#include <Thor/lld/interface/uart/uart_intf.hpp>

namespace Chimera::UART::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::UART::initialize();
  }

  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }

  bool isChannelUART( const Chimera::Serial::Channel channel )
  {
    return Thor::LLD::UART::isChannelSupported( channel );
  }

  IUART_sPtr getDriver( const Chimera::Serial::Channel channel )
  {
    return std::make_shared<Thor::UART::Driver>();
  }

  Chimera::Status_t registerDriver( Chimera::UART::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_UART )
    registry.isSupported   = true;
    registry.getDriver     = getDriver;
    registry.initialize    = initialize;
    registry.reset         = reset;
    registry.isChannelUART = isChannelUART;
    return Chimera::Status::OK;
#else
    registry.isSupported   = false;
    registry.getDriver     = nullptr;
    registry.initialize    = nullptr;
    registry.reset         = nullptr;
    registry.isChannelUART = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_DRIVER_UART == 1*/
  }
}    // namespace Chimera::UART::Backend
