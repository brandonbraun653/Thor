/********************************************************************************
 *  File Name:
 *    hw_interrupt_driver_STM32L4.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32L4 series INTERRUPT hardware.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/system>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/common/cortex-m4/utilities.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_driver.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_mapping.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_prj.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_types.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_IT )

namespace Thor::LLD::IT
{
  Chimera::System::InterruptMask disableInterrupts()
  {
    Chimera::System::InterruptMask it;
    it.mask = CortexM4::disableInterrupts();
    return it;
  }

  void enableInterrupts( Chimera::System::InterruptMask &interruptMask )
  {
    CortexM4::enableInterrupts( interruptMask.mask );
  }

}    // namespace Thor::LLD::IT

#endif /* TARGET_STM32L4 && THOR_DRIVER_INTERRUPT */
