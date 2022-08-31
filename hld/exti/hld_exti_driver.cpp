/********************************************************************************
 *  File Name:
 *    hld_exti_driver.cpp
 *
 *  Description:
 *    Implements the EXTI driver for STM32
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/exti>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/exti>

#if defined( THOR_EXTI )
namespace Chimera::EXTI::Backend
{
  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  namespace LLD = ::Thor::LLD::EXTI;

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::EXTI::Backend::DriverConfig &registry )
  {
    registry.isSupported       = true;
    registry.attach            = LLD::attach;
    registry.close             = LLD::close;
    registry.disable           = LLD::disable;
    registry.detach            = LLD::detach;
    registry.enable            = LLD::enable;
    registry.numInterruptLines = LLD::numInterruptLines;
    registry.open              = LLD::open;
    registry.trigger           = LLD::trigger;
    return Chimera::Status::OK;
  }

}    // namespace Chimera::EXTI::Backend

#endif /* THOR_EXTI */
