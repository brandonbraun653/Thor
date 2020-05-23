/********************************************************************************
 *  File Name:
 *    hw_usart_types.hpp
 *
 *  Description:
 *    STM32L4 Types for the USART Peripheral
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_USART_TYPES_HPP
#define THOR_HW_USART_TYPES_HPP

/* C++ Includes */
#include <cstdint>
#include <array>

/* Driver Includes */
#include <Thor/lld/common/registers/field_accessor.hpp>
#include <Thor/lld/stm32l4x/interrupt/hw_interrupt_prj.hpp>
#include <Thor/lld/stm32l4x/usart/hw_usart_prj.hpp>

namespace Thor::LLD::USART
{
  struct RegisterMap
  {
    volatile uint32_t CR1;  /*!< USART Control register 1,                 Address offset: 0x00 */
    volatile uint32_t CR2;  /*!< USART Control register 2,                 Address offset: 0x04 */
    volatile uint32_t CR3;  /*!< USART Control register 3,                 Address offset: 0x08 */
    volatile uint32_t BRR;  /*!< USART Baud rate register,                 Address offset: 0x0C */
    volatile uint16_t GTPR; /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    uint16_t RESERVED2;     /*!< Reserved, 0x12                                                 */
    volatile uint32_t RTOR; /*!< USART Receiver Time Out register,         Address offset: 0x14 */
    volatile uint16_t RQR;  /*!< USART Request register,                   Address offset: 0x18 */
    uint16_t RESERVED3;     /*!< Reserved, 0x1A                                                 */
    volatile uint32_t ISR;  /*!< USART Interrupt and status register,      Address offset: 0x1C */
    volatile uint32_t ICR;  /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
    volatile uint16_t RDR;  /*!< USART Receive Data register,              Address offset: 0x24 */
    uint16_t RESERVED4;     /*!< Reserved, 0x26                                                 */
    volatile uint16_t TDR;  /*!< USART Transmit Data register,             Address offset: 0x28 */
    uint16_t RESERVED5;     /*!< Reserved, 0x2A                                                 */
  };

  class Driver;
  using DriverInstanceList = std::array<Driver *, NUM_USART_PERIPHS>;
  using PeriphRegisterList = std::array<RegisterMap *, NUM_USART_PERIPHS>;
  using DMASignalList      = std::array<Reg32_t, NUM_USART_PERIPHS>;
  using IRQSignalList      = std::array<IRQn_Type, NUM_USART_PERIPHS>;

  /*------------------------------------------------
  Configuration Options
  ------------------------------------------------*/
  namespace Configuration
  {
    namespace WordLength
    {
      static constexpr Reg32_t LEN_8BIT = 0u;
      static constexpr Reg32_t LEN_9BIT = CR1_M;
    }    // namespace WordLength

    namespace Stop
    {
      static constexpr Reg32_t BIT_1   = 0u;
      static constexpr Reg32_t BIT_0_5 = CR2_STOP_0;
      static constexpr Reg32_t BIT_2   = CR2_STOP_1;
      static constexpr Reg32_t BIT_1_5 = CR2_STOP_0 | CR2_STOP_1;
    }    // namespace Stop

    namespace Parity
    {
      static constexpr Reg32_t NONE = 0u;
      static constexpr Reg32_t EVEN = CR1_PCE;
      static constexpr Reg32_t ODD  = CR1_PCE | CR1_PS;
    }    // namespace Parity

    namespace Modes
    {
      static constexpr Reg32_t RX    = CR1_RE;
      static constexpr Reg32_t TX    = CR1_TE;
      static constexpr Reg32_t TX_RX = RX | TX;
    }    // namespace Modes

    namespace Clock
    {
      static constexpr Reg32_t CLOCK_DISABLE = 0u;
      static constexpr Reg32_t CLOCK_ENABLE  = CR2_CLKEN;
    }    // namespace Clock

    namespace Polarity
    {
      static constexpr Reg32_t POLARITY_LOW  = 0u;
      static constexpr Reg32_t POLARITY_HIGH = CR2_CPOL;
    }    // namespace Polarity

    namespace Phase
    {
      static constexpr Reg32_t PHASE_1EDGE = 0u;
      static constexpr Reg32_t PHASE_2EDGE = CR2_CPHA;
    }    // namespace Phase

    namespace LastBit
    {
      static constexpr Reg32_t LASTBIT_DISABLE = 0u;
      static constexpr Reg32_t LASTBIT_ENABLE  = CR2_LBCL;
    }    // namespace LastBit

    namespace Nack
    {
      static constexpr Reg32_t NACK_ENABLE  = CR3_NACK;
      static constexpr Reg32_t NACK_DISABLE = 0u;
    }    // namespace Nack

    namespace Flags
    {
      static constexpr Reg32_t FLAG_CTS  = ISR_CTS;
      static constexpr Reg32_t FLAG_LBD  = ISR_LBDF;
      static constexpr Reg32_t FLAG_TXE  = ISR_TXE;
      static constexpr Reg32_t FLAG_TC   = ISR_TC;
      static constexpr Reg32_t FLAG_RXNE = ISR_RXNE;
      static constexpr Reg32_t FLAG_IDLE = ISR_IDLE;
      static constexpr Reg32_t FLAG_ORE  = ISR_ORE;
      static constexpr Reg32_t FLAG_NF   = ISR_NE;
      static constexpr Reg32_t FLAG_FE   = ISR_FE;
      static constexpr Reg32_t FLAG_PE   = ISR_PE;
    }    // namespace Flags
  }

  /*------------------------------------------------
  Runtime
  ------------------------------------------------*/
  namespace Runtime
  {
    using Flag_t = Reg32_t;
    namespace Flag
    {
      /* Let the first 16 bits match the Status Register for consistency */
      static constexpr Flag_t RX_PARITY_ERROR    = Configuration::Flags::FLAG_PE;
      static constexpr Flag_t RX_FRAMING_ERROR   = Configuration::Flags::FLAG_FE;
      static constexpr Flag_t RX_NOISE_ERROR     = Configuration::Flags::FLAG_NF;
      static constexpr Flag_t RX_OVERRUN         = Configuration::Flags::FLAG_ORE;
      static constexpr Flag_t RX_IDLE_DETECTED   = Configuration::Flags::FLAG_IDLE;
      static constexpr Flag_t RX_BYTE_READY      = Configuration::Flags::FLAG_RXNE;
      static constexpr Flag_t TX_COMPLETE        = Configuration::Flags::FLAG_TC;
      static constexpr Flag_t TX_DR_EMPTY        = Configuration::Flags::FLAG_TXE;
      static constexpr Flag_t RX_LINE_IN_BREAK   = Configuration::Flags::FLAG_LBD;
      static constexpr Flag_t CTL_CLEAR_TO_SEND  = Configuration::Flags::FLAG_CTS;

      /* Use the remaining 16 bits for other signals */
      static constexpr Flag_t RX_LINE_IDLE_ABORT = ( 1u << 16 );
      static constexpr Flag_t RX_COMPLETE        = ( 1u << 17 );
    }
  }

  /*------------------------------------------------
  State Machine
  ------------------------------------------------*/
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

#endif /* !THOR_HW_USART_TYPES_HPP*/
