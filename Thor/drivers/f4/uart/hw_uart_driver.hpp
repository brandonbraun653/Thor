/********************************************************************************
 *   File Name:
 *    hw_uart_driver.hpp
 *
 *   Description:
 *    STM32 Driver for the UART Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_UART_DRIVER_HPP
#define THOR_HW_UART_DRIVER_HPP

/* C++ Includes */

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>

/* Driver Includes */
#include <Thor/drivers/f4/common/types.hpp>
#include <Thor/drivers/f4/uart/hw_uart_types.hpp>
#include <Thor/drivers/f4/uart/hw_uart_stubs.hpp>
#include <Thor/drivers/f4/usart/hw_usart_driver.hpp>

namespace Thor::Driver::UART
{
  /**
   *  On the STM32F4 chips, the UART and USART peripherals share the
   *  same registers and configuration settings. To eliminate duplicate
   *  code, this UART variant simply inherits the USART driver functionality.
   */
  class Driver : public Thor::Driver::USART::Driver
  {
  public:
    Driver( RegisterMap *const peripheral ) : Thor::Driver::USART::Driver( peripheral )
    {
    }

    ~Driver()
    {
    }
  };
}    // namespace Thor::Driver::UART

extern void UART4_IRQHandler();
extern void UART5_IRQHandler();

#endif /* !THOR_HW_UART_DRIVER_HPP */
