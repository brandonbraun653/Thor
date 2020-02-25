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

namespace Chimera::USART::Backend
{
  Chimera::Status_t initialize()
  {
    return Thor::USART::initialize();
  }

  Chimera::Status_t reset()
  {
    return Chimera::CommonStatusCodes::OK;
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
#endif /* THOR_DRIVER_USART == 1*/
  }
}    // namespace Chimera::USART::Backend