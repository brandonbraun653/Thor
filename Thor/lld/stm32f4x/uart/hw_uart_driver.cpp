/********************************************************************************
 *   File Name:
 *    hw_uart_driver.hpp
 *
 *   Description:
 *    STM32 Driver for the UART Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/cfg>
#include <Thor/lld/stm32f4x/uart/hw_uart_driver.hpp>
#include <Thor/lld/stm32f4x/uart/hw_uart_mapping.hpp>
#include <Thor/lld/stm32f4x/uart/hw_uart_prj.hpp>
#include <Thor/lld/stm32f4x/uart/hw_uart_types.hpp>

#if defined( TARGET_STM32F4 ) && defined( THOR_LLD_UART )

namespace Thor::LLD::UART
{
  void initialize()
  {
    initializeRegisters();
    initializeMapping();
  }

}    // namespace Thor::LLD::UART

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
