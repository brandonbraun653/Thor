/******************************************************************************
 *  File Name:
 *    interrupt_types.hpp
 *
 *  Description:
 *    STM32 Interrupt Types
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_DRIVER_TYPES_INTERRUPT_HPP
#define THOR_DRIVER_TYPES_INTERRUPT_HPP

/* STL Includes */
#include <cstdint>
#include <Thor/hld/common/preprocessor.hpp>


/*-----------------------------------------------------------------------------
Constants
-----------------------------------------------------------------------------*/
#if defined( USING_FREERTOS ) || defined( USING_FREERTOS_THREADS )
#include "FreeRTOSConfig.h"
#define _THOR_LLD_INT_MAX_ISR_PRI ( configMAX_SYSCALL_INTERRUPT_PRIORITY )
#else
#include <limits>
#define _THOR_LLD_INT_MAX_ISR_PRI ( std::numeric_limits<uint32_t>::max() )
#endif


namespace Thor::LLD::INT
{
/*---------------------------------------------------------------------------
Set the NVIC controller priority group style
---------------------------------------------------------------------------*/
#if defined( CORTEX_M4 )
  /*---------------------------------------------------------------------------
  Use all 4 preempt bits with no subpriority bits.
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t SYSTEM_NVIC_PRIORITY_GROUPING = 0x00000003u;    // Priority group 4
#else
  static_assert( false );
#endif

  /*---------------------------------------------------------------------------
  These next values assign a preemption priority to hardware ISR handlers,but
  they cannot exceed a specific value. This takes a bit of explaining.

  Thor's architecture defers the bulk of ISR work to a non-ISR context. This is
  to allow user-space callbacks that can take advantage of the system threading
  library without worrying about an ISR context. As a result, an ISR is usually
  paired with a dedicated high priority thread. In order to wake this thread, a
  signal must be issued using the host system's thread API. Some implementations
  restrict how high the priority can be to use this API properly.

  The checks below validate each configuration is correct.
  ---------------------------------------------------------------------------*/
  static constexpr uint32_t ADC_IT_PREEMPT_PRIORITY = 5;
  static_assert( ADC_IT_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t CAN_IT_PREEMPT_PRIORITY = 5;
  static_assert( CAN_IT_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t DMA_STREAM_PREEMPT_PRIORITY = 5;
  static_assert( DMA_STREAM_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t EXTI0_MAX_IRQn_PRIORITY = 5;
  static_assert( EXTI0_MAX_IRQn_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t I2C_IT_PREEMPT_PRIORITY = 5;
  static_assert( I2C_IT_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t SPI_IT_PREEMPT_PRIORITY = 5;
  static_assert( SPI_IT_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t TIMER_IT_PREEMPT_PRIORITY = 5;
  static_assert( TIMER_IT_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t UART_DMA_PREEMPT_PRIORITY = 5;
  static_assert( UART_DMA_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t UART_IT_PREEMPT_PRIORITY = 5;
  static_assert( UART_IT_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t USART_DMA_PREEMPT_PRIORITY = 5;
  static_assert( USART_DMA_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t USART_IT_PREEMPT_PRIORITY = 5;
  static_assert( USART_IT_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

  static constexpr uint32_t USB_IT_PREEMPT_PRIORITY = 5;
  static_assert( USB_IT_PREEMPT_PRIORITY <= _THOR_LLD_INT_MAX_ISR_PRI );

}    // namespace Thor::LLD::INT

#endif /* !THOR_DRIVER_TYPES_INTERRUPT_HPP */
