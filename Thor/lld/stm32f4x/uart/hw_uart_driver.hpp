/********************************************************************************
 *  File Name:
 *    hw_uart_driver.hpp
 *
 *  Description:
 *    STM32 Driver for the UART Peripheral
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_UART_DRIVER_HPP
#define THOR_HW_UART_DRIVER_HPP

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/common/interrupts/usart_interrupt_vectors.hpp>
#include <Thor/lld/stm32f4x/common/types.hpp>
#include <Thor/lld/stm32f4x/uart/hw_uart_types.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_driver.hpp>

namespace Thor::LLD::UART
{
  /**
   *  Initializes the low level driver
   *
   *  @return void
   */
  void initialize();

  /**
   *  On the STM32F4 chips, the UART and USART peripherals share the
   *  same registers and configuration settings. To eliminate duplicate
   *  code, this UART variant simply inherits the USART driver functionality.
   */
  class Driver : public Thor::LLD::USART::Driver
  {
  public:
    Driver( RegisterMap *const peripheral ) :
        Thor::LLD::USART::Driver( peripheral )
    {
    }

    ~Driver()
    {
    }
  };
}    // namespace Thor::LLD::UART

#endif /* !THOR_HW_UART_DRIVER_HPP */
