/********************************************************************************
 *  File Name:
 *    hw_can_prv_driver.hpp
 *
 *  Description:
 *    Private driver implementation details
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_PRIVATE_CAN_DRIVER_HPP
#define THOR_LLD_PRIVATE_CAN_DRIVER_HPP

/* include description */
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_types.hpp>

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Private Functions
  -------------------------------------------------------------------------------*/
  /**
   *  Resets the peripheral to power on state
   *
   *  @param[in]  periph      Memory mapped struct to the peripheral
   *  @return void
   */
  void prv_reset( RegisterMap *const periph );

  /**
   *  Asks the hardware to enter initialization mode
   *
   *  @param[in]  periph      Memory mapped struct to the peripheral
   *  @return void
   */
  void prv_enter_initialization_mode( RegisterMap *const periph );

  /**
   *  Asks the hardware to leave initialization mode
   *
   *  @param[in]  periph      Memory mapped struct to the peripheral
   *  @return void
   */
  void prv_exit_initialization_mode( RegisterMap *const periph );

  /**
   *  Asks the hardware to enter normal mode
   *
   *  @param[in]  periph      Memory mapped struct to the peripheral
   *  @return void
   */
  void prv_enter_normal_mode( RegisterMap *const periph );

  /**
   *  Asks the hardware to enter sleep mode
   *
   *  @param[in]  periph      Memory mapped struct to the peripheral
   *  @return void
   */
  void prv_enter_sleep_mode( RegisterMap *const periph );

  /**
   *  Asks the hardware to exit sleep mode
   *
   *  @param[in]  periph      Memory mapped struct to the peripheral
   *  @return void
   */
  void prv_exit_sleep_mode( RegisterMap *const periph );

  /**
   *  Configures the peripheral baud rate appropriately
   *
   *  @param[in]  periph      Memory mapped struct to the peripheral
   *  @param[in]  cfg         Configuration to be used
   *  @return size_t          Baud rate achieved (Hz)
   */
  size_t prv_set_baud_rate( RegisterMap *const periph, const Chimera::CAN::DriverConfig &cfg );

  /**
   *  Uses the given values of the Peripheral Clock and configuration register values
   *  to estimate the actual baud rate achieved by the CAN peripheral.
   *
   *  @param[in]  periph      Memory mapped struct to the peripheral
   *  @return size_t          Baud rate achieved (Hz)
   */
  size_t prv_get_baud_rate( RegisterMap *const periph );

}  // namespace Thor::LLD::CAN

#endif  /* !THOR_LLD_PRIVATE_CAN_DRIVER_HPP */
