/********************************************************************************
 *  File Name:
 *    can_prv_data.hpp
 *
 *  Description:
 *    Declaration of data that must be defined by the LLD implementation or is
 *    shared among all possible drivers.
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_CAN_DATA_HPP
#define THOR_LLD_CAN_DATA_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/can>

/* Thor Includes */
#include <Thor/lld/common/macros.hpp>
#include <Thor/lld/common/types.hpp>
#include <Thor/lld/interface/can/can_detail.hpp>
#include <Thor/lld/interface/can/can_types.hpp>
#include <Thor/lld/interface/interrupt/interrupt_detail.hpp>

#if defined( THOR_LLD_CAN )
namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr RIndex_t RIDX_TX_MAILBOX_1 = 0;
  static constexpr RIndex_t RIDX_TX_MAILBOX_2 = 1;
  static constexpr RIndex_t RIDX_TX_MAILBOX_3 = 2;

  static constexpr RIndex_t RIDX_RX_MAILBOX_1 = 0;
  static constexpr RIndex_t RIDX_RX_MAILBOX_2 = 1;

  /*-------------------------------------------------------------------------------
  Project Defined Constants
  -------------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------------
  Peripheral Instances:
    Memory mapped structures that allow direct access to peripheral registers
  -------------------------------------------------------------------------------*/
  #if defined( STM32_CAN1_PERIPH_AVAILABLE )
    extern RegisterMap *CAN1_PERIPH;
  #endif

  /*-------------------------------------------------------------------------------
  Configuration Maps:
    These convert high level configuration options into low level register config
    options. The idea is to allow the user to specify some general options, then
    convert that over to what the peripheral understands during config/init steps.
  -------------------------------------------------------------------------------*/
  namespace ConfigMap
  {
    extern LLD_CONST Reg32_t DebugMode[ static_cast<size_t>( Chimera::CAN::DebugMode::NUM_OPTIONS ) ];
    extern LLD_CONST Reg32_t IdentifierMode[ static_cast<size_t>( Chimera::CAN::IdType::NUM_OPTIONS ) ];
    extern LLD_CONST Reg32_t FrameType[ static_cast<size_t>( Chimera::CAN::FrameType::NUM_OPTIONS ) ];
  }


  /*-------------------------------------------------------------------------------
  Peripheral Resources
  -------------------------------------------------------------------------------*/
  namespace Resource
  {
    /**
     *  Indexes into the second dimension of the IRQSignals array to
     *  identify the interrupt request number in a more friendly manner.
     */
    enum IRQHandlerIndex : uint8_t
    {
      TRANSMIT = 0,
      RECEIVE_0,
      RECEIVE_1,
      ERROR_STATUS_CHANGE,

      NUM_OPTIONS,
      UNKNOWN,
    };

    extern LLD_CONST IRQn_Type IRQSignals[ NUM_CAN_PERIPHS ][ NUM_CAN_IRQ_HANDLERS ];
  }    // namespace ResourceMap
}    // namespace Thor::LLD::CAN

#endif /* THOR_LLD_CAN */
#endif /* !THOR_LLD_CAN_DATA_HPP */
