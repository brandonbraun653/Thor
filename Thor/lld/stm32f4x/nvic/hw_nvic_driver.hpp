/********************************************************************************
 *  File Name:
 *    hw_nvic_driver.hpp
 *
 *  Description:
 *    Implements the STM32F4 NVIC driver interface. 
 *
 *  Notes:
 *    These functions are simply wrappers around the CMSIS NVIC driver as they 
 *    provide all the low level Cortex-M4 specific implementation details.
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_DRIVER_NVIC_HPP
#define THOR_HW_DRIVER_NVIC_HPP

/* C++ Includes */
#include <cstdint>

/* Driver Includes */
#include <Thor/lld/stm32f4x/nvic/hw_nvic_prj.hpp>
#include <Thor/lld/stm32f4x/nvic/hw_nvic_types.hpp>
#include <Thor/lld/stm32f4x/interrupt/hw_it_prj.hpp>

namespace Thor::LLD::IT
{
  void initialize();

  /**
   *  Sets the priority grouping field (preemption priority and subpriority)
   *
   *  @note When the Priority Group 0 is selected, IRQ preemption is no longer possible.
   *        The pending IRQ priority will be managed only by the subpriority.
   *
   *  @param[in]  priorityGroup   The desired priority grouping to be set
   *  @retval void
   */
  void setPriorityGrouping( const PriorityGrouping_t priorityGroup );

  /**
   *  Gets the current priority grouping
   *
   *  @return PriorityGrouping_t
   */
  PriorityGrouping_t getPriorityGrouping();

  /**
   *  Sets the priority of an interrupt request
   *
   *  @param[in]  IRQn            External interrupt number
   *  @param[in]  preemptPriority A lower value indicates a higher priority
   *  @param[in]  subPriority     The subpriority level for the IRQ channel. A lower value indicates a higher priority.
   *  @retval void
   */
  void setPriority( const IRQn_Type IRQn, const uint32_t preemptPriority, const uint32_t subPriority );

  /**
   *  Gets the priority of an interrupt request
   *
   *  @param[in]  IRQn            External interrupt number 
   *  @param[in]  priorityGroup   The priority grouping bits length
   *  @param[out] preemptPriority Pointer to the preemptive priority value
   *  @param[out] subPriority     Pointer to the subPriority value
   *  @retval void
   */
  void getPriority( const IRQn_Type IRQn, const uint32_t priorityGroup, uint32_t *const preemptPriority, uint32_t *const subPriority );

  /**
   *  Enables a device specific interrupt in the NVIC interrupt controller
   *  
   *  @param[in]  IRQn            External interrupt number
   *  @retval void
   */
  void enableIRQ( const IRQn_Type IRQn );

  /**
   *  Disables a device specific interrupt in the NVIC interrupt controller
   *
   *  @param[in]  IRQn            External interrupt number
   *  @retval void
   */
  void disableIRQ( const IRQn_Type IRQn );

  /**
   *  Enables a device specific interrupt in the NVIC interrupt controller
   *
   *  @param[in]  IRQn            External interrupt number
   *  @retval void
   */
  void setPendingIRQ( const IRQn_Type IRQn );

  /**
   *  Clears a device specific pending interrupt in the NVIC interrupt controller
   *
   *  @param[in]  IRQn            External interrupt number
   *  @retval void
   */
  void clearPendingIRQ( const IRQn_Type IRQn );

  /**
   *  Gets a device specific pending interrupt in the NVIC interrupt controller
   *
   *  @param[in]  IRQn            External interrupt number
   *  @retval void
   */
  uint32_t getPendingIRQ( const IRQn_Type IRQn );

  /**
   *  Gets active interrupt (reads the active register in NVIC and returns the active bit)
   *
   *  @param[in]  IRQn            External interrupt number
   *  @return uint32_t
   */
  uint32_t getActive( const IRQn_Type IRQn );

  /**
   *  Initiates a system reset request to the MCU
   *
   *  @return void
   */
  void SystemReset();
}

#endif /* !THOR_HW_DRIVER_NVIC_HPP */