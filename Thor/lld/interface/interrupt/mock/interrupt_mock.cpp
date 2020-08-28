/********************************************************************************
 *  File Name:
 *    interrupt_mock.cpp
 *
 *  Description:
 *    Mock implementation for Interrupts
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Thor Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>

#if defined( THOR_LLD_IT_MOCK )

namespace Thor::LLD::IT
{
  static Chimera::System::InterruptMask mockITMask;

  Chimera::System::InterruptMask disableInterrupts()
  {
    mockITMask = {};
    return mockITMask;
  }


  void enableInterrupts( Chimera::System::InterruptMask &interruptMask )
  {
    // Do nothing
  }

}  // namespace Thor::LLD::IT

#endif /* THOR_LLD_IT_MOCK */
