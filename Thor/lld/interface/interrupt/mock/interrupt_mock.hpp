/********************************************************************************
 *  File Name:
 *    interrupt_mock.hpp
 *
 *  Description:
 *    Mock interface for Interrupt
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_INTERRUPT_MOCK_HPP
#define THOR_LLD_INTERRUPT_MOCK_HPP

/* LLD Includes */
#include <Thor/cfg>
#include <Thor/lld/interface/interrupt/interrupt_intf.hpp>
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>

#if defined( THOR_LLD_IT_MOCK )

/* Mock Includes */
#include "gmock/gmock.h"

using IRQn_Type = uint16_t;

namespace Thor::LLD::IT
{
  void setPriorityGrouping( const uint32_t priorityGroup );
  uint32_t getPriorityGrouping();
  void setPriority( const IRQn_Type IRQn, const uint32_t preemptPriority, const uint32_t subPriority );
  void getPriority( const IRQn_Type IRQn, const uint32_t priorityGroup, uint32_t *const preemptPriority,
                    uint32_t *const subPriority );
  void enableIRQ( const IRQn_Type IRQn );
  void disableIRQ( const IRQn_Type IRQn );
  void setPendingIRQ( const IRQn_Type IRQn );
  void clearPendingIRQ( const IRQn_Type IRQn );
  uint32_t getPendingIRQ( const IRQn_Type IRQn );
  uint32_t getActive( const IRQn_Type IRQn );
  void SystemReset();
}  // namespace Thor::LLD::IT

#endif  /* THOR_LLD_IT_MOCK */
#endif  /* !THOR_LLD_INTERRUPT_MOCK_HPP */
