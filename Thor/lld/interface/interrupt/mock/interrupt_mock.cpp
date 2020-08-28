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
#include <Thor/lld/interface/interrupt/mock/interrupt_mock.hpp>
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

  void setPriorityGrouping( const uint32_t priorityGroup )
  {
  }

  uint32_t getPriorityGrouping()
  {
    return 0;
  }

  void setPriority( const IRQn_Type IRQn, const uint32_t preemptPriority, const uint32_t subPriority )
  {
  }

  void getPriority( const IRQn_Type IRQn, const uint32_t priorityGroup, uint32_t *const preemptPriority,
                    uint32_t *const subPriority )
  {
  }

  void enableIRQ( const IRQn_Type IRQn )
  {
  }

  void disableIRQ( const IRQn_Type IRQn )
  {
  }

  void setPendingIRQ( const IRQn_Type IRQn )
  {
  }

  void clearPendingIRQ( const IRQn_Type IRQn )
  {
  }

  uint32_t getPendingIRQ( const IRQn_Type IRQn )
  {
    return 0;
  }

  uint32_t getActive( const IRQn_Type IRQn )
  {
    return 0;
  }

  void SystemReset()
  {
  }

}  // namespace Thor::LLD::IT

#endif /* THOR_LLD_IT_MOCK */
