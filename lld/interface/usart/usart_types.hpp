/********************************************************************************
 *  File Name:
 *    usart_types.hpp
 *
 *  Description:
 *    Common USART types used in Thor Drivers
 *
 *  2020-2021 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_DRIVER_USART_COMMON_TYPES_HPP
#define THOR_DRIVER_USART_COMMON_TYPES_HPP

/* STL Includes */
#include <cstdint>
#include <memory>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/serial>

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Forward Declarations
  -------------------------------------------------------------------------------*/
  class Driver;
  struct RegisterMap;


  /*-------------------------------------------------------------------------------
  Aliases
  -------------------------------------------------------------------------------*/
  using Driver_rPtr = Driver*;


  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
  enum class ISRSignal : uint8_t
  {
    TRANSMIT_DATA_REG_EMPTY,
    CLEAR_TO_SEND,
    TRANSMIT_COMPLETE,
    RECEIVED_DATA_READY,
    OVERRUN_ERROR,
    LINE_IDLE,
    PARITY_ERROR,
    BREAK_FLAG,
    NOISE_FLAG,

    NUM_OPTIONS,
    INVALID
  };

  namespace StateMachine
  {
    enum TX : Chimera::Status_t
    {
      TX_READY    = Chimera::Serial::Status::TX_READY,
      TX_ONGOING  = Chimera::Serial::Status::TX_IN_PROGRESS,
      TX_ABORTED  = Chimera::Serial::Status::TX_ABORTED,
      TX_COMPLETE = Chimera::Serial::Status::TX_COMPLETE
    };

    enum RX : Chimera::Status_t
    {
      RX_READY    = Chimera::Serial::Status::RX_READY,
      RX_ONGOING  = Chimera::Serial::Status::RX_IN_PROGRESS,
      RX_COMPLETE = Chimera::Serial::Status::RX_COMPLETE,
      RX_ABORTED  = Chimera::Serial::Status::RX_ABORTED
    };
  }    // namespace StateMachine

}    // namespace Thor::LLD::USART

#endif /* !THOR_DRIVER_USART_COMMON_TYPES_HPP */
