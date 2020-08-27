/********************************************************************************
 *  File Name:
 *    hld_usart_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera USART driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/usart>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/usart>
#include <Thor/lld/interface/usart/usart_intf.hpp>

namespace Chimera::USART::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::USART::initialize();
  }

  Chimera::Status_t reset()
  {
    return Chimera::Status::OK;
  }

  bool isChannelUSART( const Chimera::Serial::Channel channel )
  {
    return Thor::LLD::USART::isChannelSupported( channel );
  }

  Chimera::USART::USART_sPtr create_shared_ptr()
  {
    return std::make_shared<Thor::USART::Driver>();
  }

  Chimera::USART::USART_uPtr create_unique_ptr()
  {
    return std::make_unique<Thor::USART::Driver>();
  }

  Chimera::Status_t registerDriver( Chimera::USART::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_USART )
    registry.isSupported    = true;
    registry.createShared   = create_shared_ptr;
    registry.createUnique   = create_unique_ptr;
    registry.initialize     = initialize;
    registry.reset          = reset;
    registry.isChannelUSART = isChannelUSART;
    return Chimera::Status::OK;
#else
    registry.isSupported    = false;
    registry.createShared   = nullptr;
    registry.createUnique   = nullptr;
    registry.initialize     = nullptr;
    registry.reset          = nullptr;
    registry.isChannelUSART = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_DRIVER_USART == 1*/
  }
}    // namespace Chimera::USART::Backend