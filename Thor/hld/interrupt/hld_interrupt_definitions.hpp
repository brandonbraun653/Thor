/********************************************************************************
 *  File Name:
 *    interrupt_definitions.hpp
 *
 *  Description:
 *    Thor Interrupt Definitions
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_IT_DEFS_HPP
#define THOR_IT_DEFS_HPP

/* C++ Includes */
#include <cstdint>

namespace Thor::Interrupt
{
  /*------------------------------------------------
  DO NOT CHANGE: Sets the priority grouping to use all 4 preempt bits with no subpriority bits.
  ------------------------------------------------*/
  static constexpr uint32_t SYSTEM_NVIC_PRIORITY_GROUPING = 0x00000003u;    // Priority group 4

  /*------------------------------------------------
  These values can utilize the full range of priority grouping bits
  EXCEPT for peripherals that us the FreeRTOS ISR API calls. Their
  priority cannot be higher than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
  ------------------------------------------------*/
  static constexpr uint32_t EXTI0_MAX_IRQn_PRIORITY     = 5;
  static constexpr uint32_t DMA_STREAM_PREEMPT_PRIORITY = 5;
  static constexpr uint32_t ADC_IT_PREEMPT_PRIORITY     = 5;
  static constexpr uint32_t CAN_IT_PREEMPT_PRIORITY     = 5;
  static constexpr uint32_t SPI_IT_PREEMPT_PRIORITY     = 5;
  static constexpr uint32_t UART_IT_PREEMPT_PRIORITY    = 5;
  static constexpr uint32_t UART_DMA_PREEMPT_PRIORITY   = 5;
  static constexpr uint32_t USART_IT_PREEMPT_PRIORITY   = UART_IT_PREEMPT_PRIORITY;
  static constexpr uint32_t USART_DMA_PREEMPT_PRIORITY  = UART_DMA_PREEMPT_PRIORITY;

}    // namespace Thor::Interrupt

#endif /* !THOR_IT_DEFS_HPP */