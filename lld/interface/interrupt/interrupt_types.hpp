/********************************************************************************
 *  File Name:
 *    interrupt_types.hpp
 *
 *  Description:
 *    STM32 Interrupt Types
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_TYPES_INTERRUPT_HPP
#define THOR_DRIVER_TYPES_INTERRUPT_HPP

/* STL Includes */
#include <cstdint>

namespace Thor::LLD::INT
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  /*------------------------------------------------
  DO NOT CHANGE: Sets the priority grouping to use
  all 4 preempt bits with no subpriority bits.
  ------------------------------------------------*/
  static constexpr uint32_t SYSTEM_NVIC_PRIORITY_GROUPING = 0x00000003u;    // Priority group 4

  /*------------------------------------------------
  These values can utilize the full range of priority grouping bits
  EXCEPT for peripherals that us the FreeRTOS ISR API calls. Their
  priority cannot be higher than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
  ------------------------------------------------*/
  static constexpr uint32_t ADC_IT_PREEMPT_PRIORITY     = 5;
  static constexpr uint32_t CAN_IT_PREEMPT_PRIORITY     = 5;
  static constexpr uint32_t DMA_STREAM_PREEMPT_PRIORITY = 5;
  static constexpr uint32_t EXTI0_MAX_IRQn_PRIORITY     = 5;
  static constexpr uint32_t SPI_IT_PREEMPT_PRIORITY     = 5;
  static constexpr uint32_t UART_DMA_PREEMPT_PRIORITY   = 5;
  static constexpr uint32_t UART_IT_PREEMPT_PRIORITY    = 5;
  static constexpr uint32_t USART_DMA_PREEMPT_PRIORITY  = 5;
  static constexpr uint32_t USART_IT_PREEMPT_PRIORITY   = 5;
  static constexpr uint32_t USB_IT_PREEMPT_PRIORITY     = 5;
}    // namespace Thor::LLD::INT

#endif /* !THOR_DRIVER_TYPES_INTERRUPT_HPP */
