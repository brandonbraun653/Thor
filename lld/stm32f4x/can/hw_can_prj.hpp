/********************************************************************************
 *  File Name:
 *    hw_can_prj.hpp
 *
 *  Description:
 *    Target specific configuration of the bxCAN peripheral
 *
 *  2020-2022 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_CAN_PROJECT_HPP
#define THOR_HW_CAN_PROJECT_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Thor/lld/common/types.hpp>
#include <cstddef>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_CAN1_PERIPH_AVAILABLE
#define STM32_CAN2_PERIPH_AVAILABLE

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_CAN_PERIPHS      = 2;
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
  static constexpr RIndex_t CAN2_RESOURCE_INDEX = 1u;

  static constexpr RIndex_t CAN_TX_ISR_SIGNAL_INDEX  = 0u;
  static constexpr RIndex_t CAN_RX_ISR_SIGNAL_INDEX  = 1u;
  static constexpr RIndex_t CAN_STS_ISR_SIGNAL_INDEX = 2u;
  static constexpr RIndex_t CAN_ERR_ISR_SIGNAL_INDEX = 3u;

}    // namespace Thor::LLD::CAN

#endif /* !THOR_HW_CAN_PROJECT_HPP */
