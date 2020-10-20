/********************************************************************************
 *  File Name:
 *    can_types.hpp
 *
 *  Description:
 *    Common LLD CAN Types
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_LLD_CAN_DRIVER_TYPES_HPP
#define THOR_LLD_CAN_DRIVER_TYPES_HPP

/* STL Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/can>

/* Thor Includes */
#include <Thor/lld/interface/can/can_detail.hpp>

namespace Thor::LLD::CAN
{
  /*-------------------------------------------------------------------------------
  Foward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;

  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  /*-------------------------------------------------
  Magic numbers that are used to indicate a filter
  has been reset. Generated from random.org.
  -------------------------------------------------*/
  static constexpr Reg32_t FLTR_RST_1 = 0x2083d26f;
  static constexpr Reg32_t FLTR_RST_2 = 0xbde27d78;

  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum class Mailbox : uint8_t
  {
    TX_MAILBOX_1,
    TX_MAILBOX_2,
    TX_MAILBOX_3,

    RX_MAILBOX_1,
    RX_MAILBOX_2,

    NUM_OPTIONS,
    UNKNOWN
  };

  enum class TXPriority : uint8_t
  {
    IDENTIFIER_DRIVEN, /**< CAN ID determines priority for TX */
    REQUEST_DRIVEN,    /**< First come first serve TX priority */

    NUM_OPTIONS,
    UNKNOWN
  };

  enum class EventType : uint8_t
  {

    NUM_OPTIONS,
    UNKNOWN
  };

  /**
   *  Error code key for the CAN_ESR register in the LEC field.
   *  These values match up with the LEC field after is has been
   *  shifted to right-aligned, so do __NOT__ modify the order.
   */
  enum class ErrorCode : uint8_t
  {
    NO_ERROR,
    STUFF_ERROR,
    FORM_ERROR,
    ACK_ERROR,
    BIT_RECESSIVE_ERROR,
    BIT_DOMINANT_ERROR,
    CRC_ERROR,
    SW_ERROR,

    NUM_OPTIONS,
    UNKNOWN
  };

  enum class FilterSlot : uint8_t
  {
    SLOT_0,
    SLOT_1,
    SLOT_2,
    SLOT_3,

    NUM_OPTIONS,
    UNKNOWN
  };

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  /**
   *  Describes information about what happened in a CAN ISR. There are
   *  multiple kinds of events and all of them are captured here.
   */
  struct ISREventContext
  {
    uint16_t isrPending; /**< Bit-field indicating ISR events needing handled */

    /**
     *  Union holding possible event contexts, interpreted by
     *  the isrPending variable above. These describe what actually
     *  happened in the last ISR event.
     */
    union _EventData
    {
      /*-------------------------------------------------
      Transmit Interrupt
      -------------------------------------------------*/
      struct _Transmit
      {
        bool txError; /**< A transmit error occurred */
        bool arbLost; /**< Arbitration was lost */
        bool txOk;    /**< Transmit was ok */
      } tx[ NUM_CAN_TX_MAILBOX ];

      /*-------------------------------------------------
      Receive Interrupt
      -------------------------------------------------*/
      struct _Receive
      {
      } rx[ NUM_CAN_RX_MAILBOX ];

      /*-------------------------------------------------
      Status Change Interrupt
      -------------------------------------------------*/
      struct _StatusChange
      {
        bool sleepAck; /**< Device has gone to sleep */
        bool wakeup;   /**< Device has woken up */
      } sts;

      /*-------------------------------------------------
      Data associated with Error Interrupts
      -------------------------------------------------*/
      struct _Error
      {
        uint8_t rxErrorCount;    /**< Last receive error count */
        uint8_t txErrorCount;    /**< Last transmit error count */
        ErrorCode lastErrorCode; /**< Last general error code */
        bool busOff;             /**< Bus has been disabled */
        bool passive;            /**< Bus has gone to passive state */
        bool warning;            /**< Bus has hit warning threshold for transceiver */
      } err;

    } event;
  };


  /**
   *  Filter description on a message ID that can be *almost*
   *  directly applied to hardware.
   */
  struct MessageFilter
  {
    bool valid;                    /**< Should this filter configuration even be trusted as valid? */
    bool active;                   /**< Should this filter be active? */
    uint32_t identifier;           /**< Determines dominant/recessive bit level for the matching identifier */
    uint32_t mask;                 /**< Optional: If mask mode, determines bits used for id comparison */
    Mailbox fifoBank;              /**< Which filter bank this message should be placed in */
    Chimera::CAN::FilterMode mode; /**< Hardware filtering mode */
    uint8_t assignedFMI;           /**< Read only. Contains the filter's match index once assigned to a hw filter bank */
  };


}    // namespace Thor::LLD::CAN

#endif /* !THOR_LLD_CAN_DRIVER_TYPES_HPP */
