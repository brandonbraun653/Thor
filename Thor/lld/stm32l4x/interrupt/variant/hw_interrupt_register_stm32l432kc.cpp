/********************************************************************************
 *  File Name:
 *    hw_interrupt_register_stm32l432kc.cpp
 *
 *  Description:
 *    Interrupt register definitions for the STM32L432KC series chips.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_driver.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_mapping.hpp>
#include <Thor/lld/stm32l4x/interrupt/variant/hw_interrupt_register_stm32l432kc.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_types.hpp>
#include <Thor/lld/stm32l4x/rcc/hw_rcc_mapping.hpp>

#if defined( STM32L432xx ) && defined( THOR_LLD_IT )

namespace Thor::LLD::IT
{
}    // namespace Thor::LLD::IT

#endif /* STM32L432xx && THOR_LLD_INTERRUPT */
