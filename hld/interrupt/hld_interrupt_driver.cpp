/********************************************************************************
 *  File Name:
 *    hld_interrupt_driver.cpp
 *
 *  Description:
 *    High level interrupt driver implementation
 *
 *  2021-2022 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/interrupt>
#include <Thor/cfg>
#include <Thor/lld/interface/inc/interrupt>

#if defined( THOR_INT )
namespace Chimera::Interrupt::Backend
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  Chimera::Status_t registerDriver( Chimera::Interrupt::Backend::DriverConfig &registry )
  {
    registry.isSupported        = true;
    registry.initialize         = Thor::LLD::INT::initialize;
    registry.reset              = Thor::LLD::INT::reset;
    registry.registerISRHandler = Thor::LLD::INT::registerISRHandler;
    return Chimera::Status::OK;
  }

}  // namespace Thor::Interrupt
#endif  /* THOR_HLD_INT */
