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
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>

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
