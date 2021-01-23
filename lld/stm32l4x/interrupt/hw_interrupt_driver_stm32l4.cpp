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

#if defined( TARGET_STM32L4 ) && defined( THOR_LLD_IT )

namespace Thor::LLD::INT
{

}    // namespace Thor::LLD::INT

#endif /* TARGET_STM32L4 && THOR_DRIVER_INTERRUPT */
