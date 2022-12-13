/******************************************************************************
 *  File Name:
 *    hw_interrupt_driver.cpp
 *
 *  Description:
 *    Implements the LLD interface to the STM32F4 series INTERRUPT hardware.
 *
 *  2021 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/system>

/* Driver Includes */
#include <Thor/cfg>

#if defined( TARGET_STM32F4 )

namespace Thor::LLD::INT
{
}    // namespace Thor::LLD::INT

#endif /* TARGET_STM32F4 && THOR_DRIVER_INTERRUPT */
