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
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t initialize()
  {
    return Thor::USART::initialize();
  }

  Chimera::Status_t reset()
  {
    return Thor::USART::reset();
  }

  bool isChannelUSART( const Chimera::Serial::Channel channel )
  {
    return Thor::LLD::USART::isChannelSupported( channel );
  }

  IUSART_sPtr getDriver( const Chimera::Serial::Channel channel )
  {
    return Thor::USART::getDriver( channel );
  }

  Chimera::Status_t registerDriver( Chimera::USART::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_USART )
    registry.isSupported    = true;
    registry.getDriver      = getDriver;
    registry.initialize     = initialize;
    registry.reset          = reset;
    registry.isChannelUSART = isChannelUSART;
    return Chimera::Status::OK;
#else
    registry.isSupported    = false;
    registry.getDriver      = nullptr;
    registry.initialize     = nullptr;
    registry.reset          = nullptr;
    registry.isChannelUSART = nullptr;
    return Chimera::Status::NOT_SUPPORTED;
#endif /* THOR_HLD_USART */
  }
}    // namespace Chimera::USART::Backend
