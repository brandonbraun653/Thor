/******************************************************************************
 *  File Name:
 *    serial_types.hpp
 *
 *  Description:
 *    STM32 Serial Driver Types
 *
 *  2019-2022 | Brandon Braun | brandonbraun653@gmail.com
 *****************************************************************************/

#pragma once
#ifndef THOR_DRIVER_TYPES_SERIAL_HPP
#define THOR_DRIVER_TYPES_SERIAL_HPP

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <Chimera/common>
#include <Chimera/serial>
#include <Thor/lld/interface/interrupt/interrupt_types.hpp>
#include <Thor/lld/interface/uart/uart_detail.hpp>
#include <Thor/lld/interface/usart/usart_detail.hpp>
#include <cstdint>

namespace Thor::LLD::Serial
{
  /*---------------------------------------------------------------------------
  Forward Declarations
  ---------------------------------------------------------------------------*/
  class Driver;

  /*---------------------------------------------------------------------------
  Aliases
  ---------------------------------------------------------------------------*/
  using Driver_rPtr = Driver *;

  /*---------------------------------------------------------------------------
  Constants
  ---------------------------------------------------------------------------*/
  static constexpr size_t NUM_SERIAL_PERIPHS = Thor::LLD::UART::NUM_UART_PERIPHS + Thor::LLD::USART::NUM_USART_PERIPHS;

  /*---------------------------------------------------------------------------
  Enumerations
  ---------------------------------------------------------------------------*/
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


  using Flag_t = Reg32_t;
  namespace Flag
  {
    static constexpr Flag_t RX_PARITY_ERROR    = 1u << 0;
    static constexpr Flag_t RX_FRAMING_ERROR   = 1u << 1;
    static constexpr Flag_t RX_NOISE_ERROR     = 1u << 2;
    static constexpr Flag_t RX_OVERRUN         = 1u << 3;
    static constexpr Flag_t RX_IDLE_DETECTED   = 1u << 4;
    static constexpr Flag_t RX_BYTE_READY      = 1u << 5;
    static constexpr Flag_t TX_COMPLETE        = 1u << 6;
    static constexpr Flag_t TX_DR_EMPTY        = 1u << 7;
    static constexpr Flag_t RX_LINE_IN_BREAK   = 1u << 8;
    static constexpr Flag_t CTL_CLEAR_TO_SEND  = 1u << 9;
    static constexpr Flag_t RX_LINE_IDLE_ABORT = 1u << 10;
    static constexpr Flag_t RX_COMPLETE        = 1u << 11;
  }    // namespace Flag


  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/
  /**
   *  Transfer control block that handles data which should
   *  never be modified during a transfer.
   *
   *  (C)onstant (D)ata (T)ransfer (C)ontrol (B)lock
   */
  struct CDTCB
  {
    uint8_t                          *buffer;    /**< Data buffer to transfer out of */
    size_t                            offset;    /**< Current offset into the buffer */
    size_t                            remaining; /**< How many bytes are left to transfer */
    size_t                            expected;  /**< How many bytes were expected to receive */
    Chimera::Status_t                 state;     /**< Current state of the transfer */
    Chimera::Hardware::PeripheralMode mode;      /**< Transfer mode being used */

    inline void reset()
    {
      buffer    = nullptr;
      offset    = 0;
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
    uint8_t                          *buffer;    /**< Data buffer to transfer into */
    size_t                            remaining; /**< How many bytes are left to transfer */
    size_t                            expected;  /**< How many bytes were expected to receive */
    Chimera::Status_t                 state;     /**< Current state of the transfer */
    Chimera::Hardware::PeripheralMode mode;      /**< Transfer mode being used */

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
  struct RegConfig
  {
    /*-------------------------------------------------------------------------
    This member configures the communication baud rate
    -------------------------------------------------------------------------*/
    uint32_t BaudRate;

    /*-------------------------------------------------------------------------
    Specifies the number of data bits transmitted or received in a frame
    -------------------------------------------------------------------------*/
    uint32_t WordLength;

    /*-------------------------------------------------------------------------
    Specifies the number of stop bits transmitted
    -------------------------------------------------------------------------*/
    uint32_t StopBits;

    /*-------------------------------------------------------------------------
    Specifies the parity mode
    -------------------------------------------------------------------------*/
    uint32_t Parity;

    /*-------------------------------------------------------------------------
    Specifies whether the Receive or Transmit mode is enabled or disabled
    -------------------------------------------------------------------------*/
    uint32_t Mode;

    /*-------------------------------------------------------------------------
    Specifies whether the hardware flow control mode is enabled or disabled
    (UART only)
    -------------------------------------------------------------------------*/
    uint32_t HwFlowCtl;

    /*-------------------------------------------------------------------------
    Specifies whether the Over sampling 8 is enabled or
    disabled, to achieve higher speed (up to fPCLK/8)
    -------------------------------------------------------------------------*/
    uint32_t OverSampling;

    /*-------------------------------------------------------------------------
    Specifies the steady state of the serial clock
    (USART only synchronous mode)
    -------------------------------------------------------------------------*/
    uint32_t CLKPolarity;

    /*-------------------------------------------------------------------------
    Specifies the clock transition on which the bit capture is made
    (USART only synchronous mode)
    -------------------------------------------------------------------------*/
    uint32_t CLKPhase;

    /*-------------------------------------------------------------------------
    Specifies whether the clock pulse corresponding to the last transmitted
    data bit (MSB) has to be output on the SCLK pin.
    (USART only synchronous mode)
    -------------------------------------------------------------------------*/
    uint32_t CLKLastBit;
  };


  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/
  /**
   * @brief Abstraction interface for UART/USART hardware differences
   */
  class HwInterface
  {
  public:
    virtual Chimera::Peripheral::Type periphType()                                                                        = 0;
    virtual Chimera::Status_t         init( const RegConfig &cfg )                                                        = 0;
    virtual Chimera::Status_t         deinit()                                                                            = 0;
    virtual int               transmit( const Chimera::Serial::TxfrMode mode, const void *const data, const size_t size ) = 0;
    virtual int               receive( const Chimera::Serial::TxfrMode mode, void *const data, const size_t size )        = 0;
    virtual Chimera::Status_t txTransferStatus()                                                                          = 0;
    virtual Chimera::Status_t rxTransferStatus()                                                                          = 0;
    virtual uint32_t          getFlags()                                                                                  = 0;
    virtual void              clearFlags( const uint32_t flagBits )                                                       = 0;
    virtual Thor::LLD::Serial::CDTCB *getTCB_TX()                                                                         = 0;
    virtual Thor::LLD::Serial::MDTCB *getTCB_RX()                                                                         = 0;
    virtual void                      killTransfer( Chimera::Hardware::SubPeripheral periph )                             = 0;

  protected:
    ~HwInterface() = default;
  };

}    // namespace Thor::LLD::Serial

#endif /* !THOR_DRIVER_TYPES_SERIAL_HPP */
