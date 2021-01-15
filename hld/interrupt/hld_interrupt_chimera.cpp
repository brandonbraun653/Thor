/********************************************************************************
 *  File Name:
 *    hld_interrupt_chimera.cpp
 *
 *  Description:
 *    Registration layer to Chimera
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/interrupt>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/interrupt>

namespace Chimera::Interrupt::Backend
{
/*-------------------------------------------------------------------------------
Public Functions
-------------------------------------------------------------------------------*/
#if defined( THOR_HLD_INT )
  Chimera::Status_t initialize()
  {
    return Thor::Interrupt::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::Interrupt::reset();
  }


  Chimera::Status_t registerISRHandler( const Chimera::Peripheral::Type type, const Chimera::Interrupt::Signal_t signal,
                                        const Chimera::Interrupt::SignalCallback &callback )
  {
    return Thor::Interrupt::registerISRHandler( type, signal, callback );
  }
#endif  /* THOR_HLD_INT */


  Chimera::Status_t registerDriver( Chimera::Interrupt::Backend::DriverConfig &registry )
  {
#if defined( THOR_HLD_INT )
    registry.isSupported        = true;
    registry.initialize         = initialize;
    registry.reset              = reset;
    registry.registerISRHandler = registerISRHandler;
    return Chimera::Status::OK;
#else
    memset( &registry, 0, sizeof( Chimera::Interrupt::Backend::DriverConfig ) );
    registry.isSupported = false;
    return Chimera::Status::NOT_SUPPORTED;
#endif    // THOR_HLD_Interrupt
  }
}    // namespace Chimera::Interrupt::Backend
