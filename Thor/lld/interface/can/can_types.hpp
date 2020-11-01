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
#include <type_traits>

/* Chimera Includes */
#include <Chimera/can>

/* Thor Includes */
#include <Thor/hld/can/hld_can_types.hpp>


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
   *  Filter description on a message ID that can be *almost*
   *  directly applied to hardware.
   */
  struct MessageFilter
  {
    bool valid;                        /**< Should this filter configuration even be trusted as valid? */
    bool active;                       /**< Should this filter be active? */
    uint32_t identifier;               /**< Determines dominant/recessive bit level for the matching identifier */
    uint32_t mask;                     /**< Optional: If mask mode, determines bits used for id comparison */
    Mailbox fifoBank;                  /**< Which filter bank this message should be placed in */
    Thor::CAN::FilterType filterType;  /**< Hardware filtering mode */
    Chimera::CAN::FrameType frameType; /**< What kind of framing should the filter use? */
    Chimera::CAN::IdType idType;       /**< Standard or extended filter ID? */
    uint8_t hwFMI;                     /**< Read only. Contains the filter's match index once assigned to a hw filter bank */

    void clear()
    {
      valid      = false;
      active     = false;
      identifier = std::numeric_limits<decltype( MessageFilter::identifier )>::max();
      mask       = std::numeric_limits<decltype( MessageFilter::mask )>::max();
      fifoBank   = Mailbox::UNKNOWN;
      filterType = Thor::CAN::FilterType::UNKNOWN;
      frameType  = Chimera::CAN::FrameType::UNKNOWN;
      idType     = Chimera::CAN::IdType::UNKNOWN;
      hwFMI      = std::numeric_limits<decltype( MessageFilter::hwFMI )>::max();
    }
  };


}    // namespace Thor::LLD::CAN

#endif /* !THOR_LLD_CAN_DRIVER_TYPES_HPP */
