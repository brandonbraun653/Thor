/********************************************************************************
 *  File Name:
 *    hld_exti_chimera.cpp
 *
 *	 Description:
 *    Implementation of Chimera EXTI driver hooks
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/exti>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/exti>


namespace Chimera::EXTI::Backend
{
  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t open()
  {
    return Thor::EXTI::open();
  }


  Chimera::Status_t close()
  {
    return Thor::EXTI::close();
  }


  Chimera::Status_t attach( const EventLine_t listener, const EdgeTrigger edge, Chimera::Function::vGeneric callback )
  {
    return Thor::EXTI::attach( listener, edge, callback );
  }


  Chimera::Status_t detach( const EventLine_t listener )
  {
    return Thor::EXTI::detach( listener );
  }


  Chimera::Status_t trigger( const EventLine_t listener )
  {
    return Thor::EXTI::trigger( listener );
  }


  Chimera::Status_t disable( const EventLine_t listener )
  {
    return Thor::EXTI::disable( listener );
  }


  Chimera::Status_t enable( const EventLine_t listener )
  {
    return Thor::EXTI::enable( listener );
  }


  EventLine_t numInterruptLines()
  {
    return Thor::EXTI::numInterruptLines();
  }


  Chimera::Status_t registerDriver( Chimera::EXTI::Backend::DriverConfig &registry )
  {
#if defined( THOR_EXTI )
    registry.isSupported       = true;
    registry.attach            = attach;
    registry.close             = close;
    registry.disable           = disable;
    registry.detach            = detach;
    registry.enable            = enable;
    registry.numInterruptLines = numInterruptLines;
    registry.open              = open;
    registry.trigger           = trigger;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::EXTI::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif    // THOR_HLD_EXTI
  }
}    // namespace Chimera::EXTI::Backend
