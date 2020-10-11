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


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct ISREventContext
  {
    /**
     *  Describes which event is being handled by this structure
     */
    Chimera::CAN::InterruptType isrEvent;

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

      } stsEvent;

      /*-------------------------------------------------
      Data associated with Error Interrupts
      -------------------------------------------------*/
      struct _Error
      {

      } errEvent;

    } details;
  };
}    // namespace Thor::LLD::CAN

#endif /* !THOR_LLD_CAN_DRIVER_TYPES_HPP */
