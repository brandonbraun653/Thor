/********************************************************************************
 *  File Name:
 *    serial_types.hpp
 *
 *  Description:
 *    STM32 Serial Driver Types
 *
 *  2019-2021 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_DRIVER_TYPES_SERIAL_HPP
#define THOR_DRIVER_TYPES_SERIAL_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/common>
#include <Chimera/serial>

/* Driver Includes */
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>

namespace Thor::LLD::Serial
{
  /*-------------------------------------------------------------------------------
  Enumerations
  -------------------------------------------------------------------------------*/
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


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  /**
   * @brief Abstraction interface for UART/USART hardware differences
   */
  struct HwInterface
  {

  };

  /**
   *  Transfer control block that handles data which should
   *  never be modified during a transfer.
   *
   *  (C)onstant (D)ata (T)ransfer (C)ontrol (B)lock
   */
  struct CDTCB
  {
    const uint8_t *buffer;                  /**< Data buffer to transfer out of */
    size_t remaining;                       /**< How many bytes are left to transfer */
    size_t expected;                        /**< How many bytes were expected to receive */
    Chimera::Status_t state;                /**< Current state of the transfer */
    Chimera::Hardware::PeripheralMode mode; /**< Transfer mode being used */

    inline void reset()
    {
      buffer    = nullptr;
      remaining = 0;
      expected  = 0;
      state     = StateMachine::TX::TX_READY;
      mode      = Chimera::Hardware::PeripheralMode::UNKNOWN_MODE;
    }
  };

  /**
   *  Transfer control block that handles data which might
   *  be modified during a transfer.
   *
   *  (M)odifiable (D)ata (T)ransfer (C)ontrol (B)lock
   */
  struct MDTCB
  {
    uint8_t *buffer;                        /**< Data buffer to transfer into */
    size_t remaining;                       /**< How many bytes are left to transfer */
    size_t expected;                        /**< How many bytes were expected to receive */
    Chimera::Status_t state;                /**< Current state of the transfer */
    Chimera::Hardware::PeripheralMode mode; /**< Transfer mode being used */

    inline void reset()
    {
      buffer    = nullptr;
      remaining = 0;
      expected  = 0;
      state     = StateMachine::RX::RX_READY;
      mode      = Chimera::Hardware::PeripheralMode::UNKNOWN_MODE;
    }
  };

  /**
   *  Configuration structure for the Serial peripherals (UART/USART).
   *  Each member is expected to be equal to the exact value needed to
   *  configure the appropriate control register. The calculation of these
   *  values is left up to the hardware driver as this might vary from
   *  chip to chip. Expect that these values will be writen directly to
   *  a register without much translation or protection.
   *
   *  The only exception is the BaudRate parameter, which can be set to
   *  a numerical value and not a register configuration value.
   */
  struct Config
  {
    /*------------------------------------------------
    This member configures the communication baud rate
    ------------------------------------------------*/
    uint32_t BaudRate;

    /*------------------------------------------------
    Specifies the number of data bits transmitted or received in a frame
    ------------------------------------------------*/
    uint32_t WordLength;

    /*------------------------------------------------
    Specifies the number of stop bits transmitted
    ------------------------------------------------*/
    uint32_t StopBits;

    /*------------------------------------------------
    Specifies the parity mode
    ------------------------------------------------*/
    uint32_t Parity;

    /*------------------------------------------------
    Specifies whether the Receive or Transmit mode is enabled or disabled
    ------------------------------------------------*/
    uint32_t Mode;

    /*------------------------------------------------
    Specifies whether the hardware flow control mode is enabled or disabled
    (UART only)
    ------------------------------------------------*/
    uint32_t HwFlowCtl;

    /*------------------------------------------------
    Specifies whether the Over sampling 8 is enabled or
    disabled, to achieve higher speed (up to fPCLK/8)
    ------------------------------------------------*/
    uint32_t OverSampling;

    /*------------------------------------------------
    Specifies the steady state of the serial clock
    (USART only synchronous mode)
    ------------------------------------------------*/
    uint32_t CLKPolarity;

    /*------------------------------------------------
    Specifies the clock transition on which the bit capture is made
    (USART only synchronous mode)
    ------------------------------------------------*/
    uint32_t CLKPhase;

    /*------------------------------------------------
    Specifies whether the clock pulse corresponding to the last transmitted
    data bit (MSB) has to be output on the SCLK pin.
    (USART only synchronous mode)
    ------------------------------------------------*/
    uint32_t CLKLastBit;
  };

}    // namespace Thor::LLD::Serial

#endif /* !THOR_DRIVER_TYPES_SERIAL_HPP */
