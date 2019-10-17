/********************************************************************************
 *   File Name:
 *    hw_uart_types.hpp
 *
 *   Description:
 *    STM32 Types for the UART Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_UART_TYPES_HPP
#define THOR_HW_UART_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/uart/hw_uart_prj.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_UART == 1 )

namespace Thor::Driver::UART
{
  /*------------------------------------------------
  For the F4, the UART and USART peripherals are synonymous
  ------------------------------------------------*/
  using RegisterMap = Thor::Driver::USART::RegisterMap;

  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  bool isUART( const std::uintptr_t address );

}    // namespace Thor::Driver::UART

#endif /* TARGET_STM32F4 && THOR_DRIVER_UART */
#endif /* !THOR_HW_UART_TYPES_HPP */