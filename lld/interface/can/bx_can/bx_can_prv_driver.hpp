/******************************************************************************
 *  File Name:
 *    bx_can_prv_driver.hpp
 *
 *  Description:
 *    bxCAN driver implementation details
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_LLD_PRIVATE_CAN_DRIVER_HPP
#define THOR_LLD_PRIVATE_CAN_DRIVER_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/interface/can/bx_can/bx_can_types.hpp>
#include <Thor/lld/interface/can/can_intf.hpp>
#include <Thor/lld/interface/can/can_types.hpp>

namespace Thor::LLD::CAN
{
  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/
  /**
   * Resets the peripheral to power on state
   *
   * @param periph      Memory mapped struct to the peripheral
   * @return void
   */
  void prv_reset( RegisterMap *const periph );

  /**
   * Asks the hardware to enter initialization mode
   *
   * @param periph      Memory mapped struct to the peripheral
   * @return void
   */
  void prv_enter_initialization_mode( RegisterMap *const periph );

  /**
   * Asks the hardware to enter normal mode
   *
   * @param periph      Memory mapped struct to the peripheral
   * @return void
   */
  void prv_enter_normal_mode( RegisterMap *const periph );

  /**
   * Asks the hardware to enter sleep mode
   *
   * @param periph      Memory mapped struct to the peripheral
   * @return void
   */
  void prv_enter_sleep_mode( RegisterMap *const periph );

  /**
   * Configures the peripheral baud rate appropriately
   *
   * @param periph      Memory mapped struct to the peripheral
   * @param cfg         Configuration to be used
   * @return size_t          Baud rate achieved (Hz)
   */
  size_t prv_set_baud_rate( RegisterMap *const periph, const Chimera::CAN::DriverConfig &cfg );

  /**
   * Uses the given values of the Peripheral Clock and configuration register values
   * to estimate the actual baud rate achieved by the CAN peripheral.
   *
   * @param periph      Memory mapped struct to the peripheral
   * @return size_t          Baud rate achieved (Hz)
   */
  size_t prv_get_baud_rate( RegisterMap *const periph );

  /**
   * Checks that a given frame is valid for the driver
   *
   * @param frame       Frame to be checked
   * @return bool
   */
  bool prv_validate_frame( const Chimera::CAN::BasicFrame &frame );

  /**
   * Checks if the hardware is in normal mode
   *
   * @param periph      Memory mapped struct to the peripheral
   * @return bool
   */
  bool prv_in_normal_mode( RegisterMap *const periph );

  /**
   * Looks at the given filter bank registers and determines if the
   * filter configuration will fit. Assuming it does, returns which
   * slot it fits into.
   *
   * @param filter      The user's filter that is trying to be applied
   * @param bank        Hardware filter bank to be analyzed
   * @param slot        Slot the filter fits into
   * @return bool
   */
  bool prv_does_filter_fit( const MessageFilter *const filter, volatile FilterReg *const bank, FilterSlot &slot );

  /**
   * Assigns the filter to the given slot, overwriting anything
   * that currently exists.
   *
   * @param filter      The user's filter that is trying to be applied
   * @param bank        Hardware filter bank to be analyzed
   * @param slot        Slot the filter fits into
   * @return bool
   */
  bool prv_assign_filter( const MessageFilter *const filter, volatile FilterReg *const bank, const FilterSlot slot );

  /**
   * Gets the FIFO identifier that has been assigned to a given filter bank
   *
   * @param periph      Memory mapped struct to the peripheral
   * @param bank_idx    Which filter bank to look at
   * @return Mailbox
   */
  Mailbox prv_get_filter_bank_fifo( RegisterMap *const periph, const size_t bank_idx );

  /**
   * Gets the filtering mode that has been assigned to a given filter bank
   *
   * @param periph      Memory mapped struct to the peripheral
   * @param bank_idx    Which filter bank to look at
   * @return Thor::CAN::FilterType
   */
  Thor::CAN::FilterType prv_get_filter_bank_mode( RegisterMap *const periph, const size_t bank_idx );

  /**
   * Writes packed frame data into the appropriate TX mailbox registes
   *
   * @param box         Hardware mailbox to write into
   * @param frame       The packed frame data
   * @return void
   */
  void prv_assign_frame_to_mailbox( volatile TxMailbox *const box, const Chimera::CAN::BasicFrame &frame );

  /**
   * Gets the next available TX mailbox that can hold a message
   *
   * @param periph      Memory mapped struct to the peripheral
   * @return volatile TxMailbox *
   */
  volatile TxMailbox *prv_get_free_tx_mailbox( RegisterMap *const periph );

}    // namespace Thor::LLD::CAN

#endif /* !THOR_LLD_PRIVATE_CAN_DRIVER_HPP */
