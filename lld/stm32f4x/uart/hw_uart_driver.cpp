/********************************************************************************
 *  File Name:
 *    hw_uart_driver.hpp
 *
 *  Description:
 *    STM32 Driver for the UART Peripheral
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/inc/uart>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_UART )

namespace Thor::LLD::UART
{
}    // namespace Thor::LLD::UART

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
