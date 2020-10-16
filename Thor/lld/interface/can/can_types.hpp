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
    IDENTIFIER_DRIVEN,  /**< CAN ID determines priority for TX */
    REQUEST_DRIVEN,     /**< First come first serve TX priority */

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
   *  These values match up with the field after is has been
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

  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct MailboxError
  {
    bool txError;
    bool arbLost;
  };

  struct ISREventContext
  {
    uint16_t isrPending; /**< Bit-field (BFPendingISR) indicating ISR events needing handled */

    /**
     *  Union holding possible event contexts, interpreted by
     *  the isrEvent variable above. These describe what actually
     *  happened in the last ISR event.
     */
    union _EventData
    {
      /*-------------------------------------------------
      Transmit Interrupt
      -------------------------------------------------*/
      struct _Transmit
      {
        struct _ErrorCodes
        {
          bool txError;
          bool arbLost;
          bool txOk;
        };

        _ErrorCodes mailbox0;
        _ErrorCodes mailbox1;
        _ErrorCodes mailbox2;
      } txEvent;

      /*-------------------------------------------------
      Receive Interrupt
      -------------------------------------------------*/
      struct _Receive
      {

      } rxEvent;

      /*-------------------------------------------------
      Status Change Interrupt
      -------------------------------------------------*/
      struct _StatusChange
      {
        bool sleepAck;
        bool wakeup;
      } stsEvent;

      /*-------------------------------------------------
      Data associated with Error Interrupts
      -------------------------------------------------*/
      struct _Error
      {
        uint8_t rxErrorCount;
        uint8_t txErrorCount;
        ErrorCode lastErrorCode;
        bool busOff;
        bool passive;
        bool warning;
      } errEvent;

    } details;
  };
}    // namespace Thor::LLD::CAN

#endif /* !THOR_LLD_CAN_DRIVER_TYPES_HPP */
