/********************************************************************************
 *  File Name:
 *    hw_interrupt_mapping.cpp
 *
 *  Description:
 *    Provides structures for conversion and mapping between data types for fast
 *    runtime performance of driver code.
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* STL Includes */
#include <array>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_mapping.hpp>

#if defined( TARGET_STM32L4 ) && defined( THOR_IT )

namespace Thor::LLD::INT
{
}    // namespace Thor::LLD::INT

#endif /* TARGET_STM32L4 && THOR_LLD_INTERRUPT */
