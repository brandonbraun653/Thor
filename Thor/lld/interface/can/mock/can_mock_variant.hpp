/********************************************************************************
 *  File Name:
 *    can_mock_variant.hpp
 *
 *  Description:
 *    Mock variant of the CAN hardware
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_CAN_MOCK_VARIANT_HPP
#define THOR_LLD_CAN_MOCK_VARIANT_HPP

/* STL Includes */
#include <cstddef>

/* Thor Includes */
#include <Thor/lld/common/types.hpp>

#ifndef STM32_CAN1_PERIPH_AVAILABLE
#define STM32_CAN1_PERIPH_AVAILABLE
#endif

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Literals
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_CAN_PERIPHS      = 1;
  static constexpr size_t NUM_CAN_IRQ_HANDLERS = 4; /**< Number of unique interrupts on each peripheral */
  static constexpr size_t NUM_CAN_TX_MAILBOX   = 3;
  static constexpr size_t NUM_CAN_RX_MAILBOX   = 2;
  static constexpr size_t NUM_CAN_FILTER_BANKS = 14; /**< Technically 28, but other 14 is for CAN2, which isn't on L4 */
  static constexpr size_t NUM_CAN_MAX_FILTERS  = NUM_CAN_FILTER_BANKS * 4;    // Each bank can hold a max of four 16-bit filters
  static constexpr size_t CAN_RX_FIFO_DEPTH    = 3;

  static constexpr RIndex_t CAN1_RESOURCE_INDEX = 0u;

  static constexpr RIndex_t CAN_TX_ISR_SIGNAL_INDEX  = 0u;
  static constexpr RIndex_t CAN_RX_ISR_SIGNAL_INDEX  = 1u;
  static constexpr RIndex_t CAN_STS_ISR_SIGNAL_INDEX = 2u;
  static constexpr RIndex_t CAN_ERR_ISR_SIGNAL_INDEX = 3u;


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    uint32_t mockRegister;
  };

  struct ISREventContext
  {
    uint32_t mock;
  };

  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeMapping();
  void initializeRegisters();

}  // namespace Thor::LLD::CAN

#endif  /* !THOR_LLD_CAN_MOCK_VARIANT_HPP */
