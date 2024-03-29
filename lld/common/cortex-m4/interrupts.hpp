/******************************************************************************
 *  File Name:
 *    interrupts.hpp
 *
 *  Description:
 *    Common interrupt functions for Cortex-M4 processors
 *
 *  2020-2023 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_CORTEX_M4_INTERRUPTS
#define THOR_LLD_CORTEX_M4_INTERRUPTS

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/common/cmsis/configuration.hpp>
#include <cstdint>

#if defined( CORTEX_M4 ) && defined( EMBEDDED )
#include <Thor/lld/common/cmsis/core/include/core_cm4.h>
#endif

namespace Thor::LLD::INT
{
  static constexpr uint32_t NVIC_PRIORITYGROUP_0 = 0x00000007u; /**< 0 bits for pre-emption priority, 4 bits for subpriority */
  static constexpr uint32_t NVIC_PRIORITYGROUP_1 = 0x00000006u; /**< 1 bits for pre-emption priority, 3 bits for subpriority */
  static constexpr uint32_t NVIC_PRIORITYGROUP_2 = 0x00000005u; /**< 2 bits for pre-emption priority, 2 bits for subpriority */
  static constexpr uint32_t NVIC_PRIORITYGROUP_3 = 0x00000004u; /**< 3 bits for pre-emption priority, 1 bits for subpriority */
  static constexpr uint32_t NVIC_PRIORITYGROUP_4 = 0x00000003u; /**< 4 bits for pre-emption priority, 0 bits for subpriority */

  /**
   *  Sets the priority grouping field (preemption priority and subpriority)
   *
   *  @note When the Priority Group 0 is selected, IRQ preemption is no longer possible.
   *        The pending IRQ priority will be managed only by the subpriority.
   *
   *  @param[in]  priorityGroup   The desired priority grouping to be set
   *  @retval void
   */
  void setPriorityGrouping( const uint32_t priorityGroup );

  /**
   *  Gets the current priority grouping
   *
   *  @return uint32_t
   */
  uint32_t getPriorityGrouping();

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
  void getPriority( const IRQn_Type IRQn, const uint32_t priorityGroup, uint32_t *const preemptPriority,
                    uint32_t *const subPriority );

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
   * @brief Checks if the interrupt is enabled
   *
   * @param IRQn      The interrupt to check
   * @return bool     True if enabled, false otherwise
   */
  bool isEnabledIRQ( const IRQn_Type IRQn );

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
}    // namespace Thor::LLD::INT

#endif /* !THOR_LLD_CORTEX_M4_INTERRUPTS */
