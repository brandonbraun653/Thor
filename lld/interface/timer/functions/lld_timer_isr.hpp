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
  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
  enum class ISRExtended : uint8_t
  {
    TIM1_BRK_TIM9,
    TIM1_UP_TIM10,
    TIM1_TRG_COM_TIM11,

    NUM_OPTIONS,
    NONE
  };

  enum class ISRSource : uint32_t
  {
    UPDATE = ( 1u << DIER_UIE_Pos ),   /**< Update event */
    CC1    = ( 1u << DIER_CC1IE_Pos ), /**< Capture/Compare 1 event */
    CC2    = ( 1u << DIER_CC2IE_Pos ), /**< Capture/Compare 2 event */
    CC3    = ( 1u << DIER_CC3IE_Pos ), /**< Capture/Compare 3 event */
    CC4    = ( 1u << DIER_CC4IE_Pos ), /**< Capture/Compare 4 event */
    COM    = ( 1u << DIER_COMIE_Pos ), /**< Commutation event */
    TRIG   = ( 1u << DIER_TIE_Pos ),   /**< Trigger event */
    BREAK  = ( 1u << DIER_BIE_Pos ),   /**< Break event */

    NUM_OPTIONS
  };

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/
  /**
   * @brief Gets the hardware interrupt vector index for the given timer instance
   *
   * @param inst  Which timer instance to lookup
   * @param ext   Optional extended ISR to adjust the index for
   * @return IRQn_Type
   */
  IRQn_Type getHWISRIndex( const Chimera::Timer::Instance inst, const ISRExtended ext );

  /**
   * @brief Attaches an interrupt handler to a timer interrupt
   *
   * @param irq   Which interrupt to attach to
   * @param func  The function to call when the interrupt occurs
   * @param ext   Optional extended ISR to attach to if the timer supports more than one interrupt
   * @return Chimera::Status_t
   */
  Chimera::Status_t attachISR( const Chimera::Timer::Instance inst, Chimera::Function::Opaque func, const ISRExtended ext );

  /**
   * @brief Removes callback function associated with the given IRQ
   *
   * @param irq   Which ISR to remove the callback from
   * @param ext   Optional extended ISR to remove the callback from if the timer supports more than one interrupt
   */
  void detachISR( const Chimera::Timer::Instance inst, const ISRExtended ext );

  /**
   * @brief Enables the ISR for the given timer
   *
   * @param timer   Which timer to act on
   * @param source  Which source to enable
   */
  void enableISR( Handle_rPtr timer, const ISRSource source );

  /**
   * @brief Disables the ISR for the given timer
   *
   * @param timer   Which timer to act on
   * @param source  Which source to disable
   */
  void disableISR( Handle_rPtr timer, const ISRSource source );

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_ISR_HPP */
