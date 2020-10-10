/********************************************************************************
 *  File Name:
 *    can_types.hpp
 *
 *  Description:
 *    Common LLD CAN Types
 *
 *  2019-2020 | Brandon Braun | brandonbraun653@gmail.com
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
  struct EventData
  {
    EventType whatHappened;

    union _EventData
    {
      // fill with struct defs
      struct _ErrorEvent
      {

      } errorEvent;

    } details;
  };
}    // namespace Thor::LLD::CAN

#endif /* !THOR_LLD_CAN_DRIVER_TYPES_HPP */
