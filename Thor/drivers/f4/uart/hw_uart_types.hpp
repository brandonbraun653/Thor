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
#include <Thor/drivers/f4/uart/hw_uart_prj.hpp>
#include <Thor/drivers/f4/usart/hw_usart_types.hpp>

namespace Thor::Driver::UART
{
  /*------------------------------------------------
  For the F4, the UART and USART peripherals are synonymous
  ------------------------------------------------*/
  using RegisterMap = Thor::Driver::USART::RegisterMap;

  static RegisterMap *const UART4_PERIPH = reinterpret_cast<RegisterMap *const>( UART4_BASE_ADDR );
  static RegisterMap *const UART5_PERIPH = reinterpret_cast<RegisterMap *const>( UART5_BASE_ADDR );

}    // namespace Thor::Driver::UART

#endif /* !THOR_HW_UART_TYPES_HPP */