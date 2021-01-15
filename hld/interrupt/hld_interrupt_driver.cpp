/********************************************************************************
 *  File Name:
 *    hld_interrupt_driver.cpp
 *
 *  Description:
 *    High level interrupt driver implementation
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/interrupt>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/interrupt>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>

namespace Thor::Interrupt
{
/*-------------------------------------------------------------------------------
Public Functions
-------------------------------------------------------------------------------*/
#if defined( THOR_HLD_INT )
  Chimera::Status_t initialize()
  {
    return Thor::LLD::INT::initialize();
  }


  Chimera::Status_t reset()
  {
    return Thor::LLD::INT::reset();
  }


  Chimera::Status_t registerISRHandler( const Chimera::Peripheral::Type type, const Chimera::Interrupt::Signal_t signal,
                                        const Chimera::Interrupt::SignalCallback &callback )
  {
    return Thor::LLD::INT::registerISRHandler( type, signal, callback );
  }
#endif  /* THOR_HLD_INT */

}  // namespace Thor::Interrupt
