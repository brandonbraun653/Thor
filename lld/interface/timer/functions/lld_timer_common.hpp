/******************************************************************************
 *  File Name:
 *    lld_timer_common.hpp
 *
 *  Description:
 *    Common driver for Advanced, Basic, General timers
 *
 *  2022 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_TIMER_COMMON_DRIVER_HPP
#define THOR_LLD_TIMER_COMMON_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/timer/timer_types.hpp>
#include <Thor/lld/interface/rcc/rcc_intf.hpp>

namespace Thor::LLD::TIMER
{
  /*---------------------------------------------------------------------------
  Public Methods
  ---------------------------------------------------------------------------*/
  /**
   * @brief Attaches a peripheral instance to the driver
   *
   * @param timer         Timer instance to act on
   * @param peripheral    Memory mapped struct of the desired peripheral
   * @return void
   */
  void attach( Handle_rPtr timer, RegisterMap *const peripheral );

  /**
   * @brief Resets the hardware registers back to boot-up values
   *
   * @param timer         Timer instance to act on
   * @return Chimera::Status_t
   */
  void reset( Handle_rPtr timer );

  /**
   * @brief Enables the peripheral clock
   *
   * @param timer         Timer instance to act on
   * @return void
   */
  void clockEnable( Handle_rPtr timer );

  /**
   * @brief Disables the peripheral clock
   *
   * @param timer         Timer instance to act on
   * @return void
   */
  void clockDisable( Handle_rPtr timer );

  /**
   * @brief Powers up the entire peripheral
   *
   * @param timer         Timer instance to act on
   * @return void
   */
  void open( Handle_rPtr timer );

  /**
   * @brief Tear down the peripheral
   *
   * @param timer         Timer instance to act on
   * @return void
   */
  void close( Handle_rPtr timer );

}    // namespace Thor::LLD::TIMER

#endif /* !THOR_LLD_TIMER_COMMON_DRIVER_HPP */
