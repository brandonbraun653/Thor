/********************************************************************************
 *   File Name:
 *    interrupt_definitions.hpp
 *
 *   Description:
 *    Thor Interrupt Definitions
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_IT_DEFS_HPP
#define THOR_IT_DEFS_HPP

/* C++ Includes */
#include <cstdint>

/* Thor Includes */
#include <Thor/headers.hpp>

namespace Thor::Interrupt
{
  /*------------------------------------------------
  DO NOT CHANGE: Sets the priority grouping to use all 4 preempt bits with no subpriority bits.
  ------------------------------------------------*/
#if defined( THOR_STM32HAL_DRIVERS ) && ( THOR_STM32HAL_DRIVERS == 1 )
  static constexpr uint32_t SYSTEM_NVIC_PRIORITY_GROUPING = NVIC_PRIORITYGROUP_4;
#endif

#if defined( USING_FREERTOS )
  static constexpr uint32_t MAX_PENDING_TASK_TRIGGERS = 10;

  /*------------------------------------------------
  These values can utilize the full range of priority grouping bits
  EXCEPT for peripherals that us the FreeRTOS ISR API calls. Their
  priority cannot be higher than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
  ------------------------------------------------*/
  static constexpr uint32_t EXTI0_MAX_IRQn_PRIORITY = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
  static_assert( EXTI0_MAX_IRQn_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, "Invalid interrupt priority" );

  static constexpr uint32_t UART_IT_PREEMPT_PRIORITY = 5;
  static_assert( UART_IT_PREEMPT_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, "Invalid interrupt priority" );

  static constexpr uint32_t UART_DMA_PREEMPT_PRIORITY = 5;
  static_assert( UART_DMA_PREEMPT_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, "Invalid interrupt priority" );

  static constexpr uint32_t USART_IT_PREEMPT_PRIORITY = UART_IT_PREEMPT_PRIORITY;
  static_assert( USART_IT_PREEMPT_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, "Invalid interrupt priority" );

  static constexpr uint32_t USART_DMA_PREEMPT_PRIORITY = UART_DMA_PREEMPT_PRIORITY;
  static_assert( USART_DMA_PREEMPT_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, "Invalid interrupt priority" );

  /** The various types of triggers that can be used to unlock a FreeRTOS thread */
  enum Trigger : uint8_t
  {
    RX_COMPLETE,
    TX_COMPLETE,
    TXRX_COMPLETE,
    TRANSFER_ERROR,
    BUFFERED_TX_COMPLETE,
    BUFFERED_TXRX_COMPLETE,
    MAX_SOURCES
  };
#else
    /* These values can safely take on the full range of the priority grouping bits (0-15) with 0 as the highest priority. */
    static constexpr uint32_t UART_IT_PREEMPT_PRIORITY   = 2;
    static constexpr uint32_t UART_DMA_PREEMPT_PRIORITY  = 2;
    static constexpr uint32_t USART_IT_PREEMPT_PRIORITY  = UART_IT_PREEMPT_PRIORITY;
    static constexpr uint32_t USART_DMA_PREEMPT_PRIORITY = UART_DMA_PREEMPT_PRIORITY;
#endif
}

#endif /* !THOR_IT_DEFS_HPP */