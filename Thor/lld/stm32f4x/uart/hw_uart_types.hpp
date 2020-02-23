/********************************************************************************
 *  File Name:
 *    hw_uart_types.hpp
 *
 *  Description:
 *    STM32 Types for the UART Peripheral
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_UART_TYPES_HPP
#define THOR_HW_UART_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>

/* Driver Includes */
#include <Thor/lld/stm32f4x/uart/hw_uart_prj.hpp>
#include <Thor/lld/stm32f4x/usart/hw_usart_types.hpp>

namespace Thor::LLD::UART
{
  /*------------------------------------------------
  For the F4, the UART and USART peripherals are synonymous
  ------------------------------------------------*/
  using RegisterMap = Thor::Driver::USART::RegisterMap;

  class Driver;

  using DriverInstanceList = std::array<Driver *, NUM_UART_PERIPHS>;
  using PeriphRegisterList = std::array<RegisterMap *, NUM_UART_PERIPHS>;
  using DMASignalList      = std::array<Reg32_t, NUM_UART_PERIPHS>;


}    // namespace Thor::LLD::UART

#endif /* !THOR_HW_UART_TYPES_HPP */