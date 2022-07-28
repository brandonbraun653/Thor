/******************************************************************************
 *  File Name:
 *    lld_timer_isr.hpp
 *
 *  Description:
 *    Timer Interrupt Interface
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_ISR_HPP
#define THOR_LLD_TIMER_ISR_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/function>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

namespace Thor::LLD::TIMER
{
  /**
   * @brief Attaches an interrupt handler to a timer interrupt
   *
   * @param irq   Which interrupt to attach to
   * @param func  The function to call when the interrupt occurs
   * @return Chimera::Status_t
   */
  Chimera::Status_t attachISR( const IRQn_Type irq, Chimera::Function::Opaque func );

  /**
   * @brief Removes callback function associated with the given IRQ
   *
   * @param irq   Which ISR to remove the callback from
   */
  void detachISR( const IRQn_Type irq );

}  // namespace Thor::LLD::TIMER

#endif  /* !THOR_LLD_TIMER_ISR_HPP */
