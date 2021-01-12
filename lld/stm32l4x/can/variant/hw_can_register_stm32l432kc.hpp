/********************************************************************************
 *  File Name:
 *    hw_can_register_stm32l432kc.hpp
 *
 *  Description:
 *    CAN definitions for the STM32L432KC series chips
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_CAN_REGISTER_STM32L432KC_HPP
#define THOR_LLD_CAN_REGISTER_STM32L432KC_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_CAN1_PERIPH_AVAILABLE

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_CAN_PERIPHS      = 1;
  static constexpr size_t NUM_CAN_IRQ_HANDLERS = 4; /**< Number of unique interrupts on each peripheral */
  static constexpr size_t NUM_CAN_TX_MAILBOX   = 3;
  static constexpr size_t NUM_CAN_RX_MAILBOX   = 2;
  static constexpr size_t NUM_CAN_FILTER_BANKS = 14; /**< Technically 28, but other 14 is for CAN2, which isn't on L4 */

  static constexpr size_t NUM_CAN_MAX_32BIT_MASK_FILTERS = NUM_CAN_FILTER_BANKS;
  static constexpr size_t NUM_CAN_MAX_32BIT_LIST_FILTERS = NUM_CAN_FILTER_BANKS * 2;
  static constexpr size_t NUM_CAN_MAX_16BIT_MASK_FILTERS = NUM_CAN_FILTER_BANKS * 2;
  static constexpr size_t NUM_CAN_MAX_16BIT_LIST_FILTERS = NUM_CAN_FILTER_BANKS * 4;
  static constexpr size_t NUM_CAN_MAX_FILTERS            = NUM_CAN_MAX_16BIT_LIST_FILTERS;
  static constexpr size_t CAN_RX_FIFO_DEPTH              = 3;

  static constexpr RIndex_t CAN1_RESOURCE_INDEX = 0u;

  static constexpr RIndex_t CAN_TX_ISR_SIGNAL_INDEX  = 0u;
  static constexpr RIndex_t CAN_RX_ISR_SIGNAL_INDEX  = 1u;
  static constexpr RIndex_t CAN_STS_ISR_SIGNAL_INDEX = 2u;
  static constexpr RIndex_t CAN_ERR_ISR_SIGNAL_INDEX = 3u;

}    // namespace Thor::LLD::CAN

#endif /* !THOR_LLD_CAN_REGISTER_STM32L432KC_HPP */
