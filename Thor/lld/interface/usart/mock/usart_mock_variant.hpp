/********************************************************************************
 *  File Name:
 *    usart_mock_variant.hpp
 *
 *  Description:
 *    Mock variant of the USART hardware
 *
 *  2020 | Brandon Braun | brandonbraun653@gmail.com
 *******************************************************************************/

#pragma once
#ifndef THOR_LLD_USART_MOCK_VARIANT_HPP
#define THOR_LLD_USART_MOCK_VARIANT_HPP

/* STL Includes */
#include <cstddef>

/* Chimera Includes */
#include <Chimera/serial>

/* Thor Includes */
#include <Thor/lld/interface/usart/usart_intf.hpp>
#include <Thor/lld/interface/usart/usart_types.hpp>

/*-------------------------------------------------------------------------------
Macros
-------------------------------------------------------------------------------*/
#define STM32_USART1_PERIPH_AVAILABLE
#define STM32_USART2_PERIPH_AVAILABLE
#define STM32_USART3_PERIPH_AVAILABLE
#define STM32_USART4_PERIPH_AVAILABLE

namespace Thor::LLD::USART
{
  /*-------------------------------------------------------------------------------
  Constants
  -------------------------------------------------------------------------------*/
  static constexpr size_t NUM_USART_PERIPHS = 4;

  static constexpr uint32_t USART1_RESOURCE_INDEX = 0u;
  static constexpr uint32_t USART2_RESOURCE_INDEX = 1u;
  static constexpr uint32_t USART3_RESOURCE_INDEX = 2u;
  static constexpr uint32_t USART4_RESOURCE_INDEX = 3u;

  namespace Configuration
  {
    namespace WordLength
    {
      static constexpr Reg32_t LEN_8BIT = 0u;
      static constexpr Reg32_t LEN_9BIT = 1u;
    }    // namespace WordLength

    namespace Stop
    {
      static constexpr Reg32_t BIT_1   = 0u;
      static constexpr Reg32_t BIT_0_5 = 1u;
      static constexpr Reg32_t BIT_2   = 2u;
      static constexpr Reg32_t BIT_1_5 = 3u;
    }    // namespace Stop

    namespace Parity
    {
      static constexpr Reg32_t NONE = 0u;
      static constexpr Reg32_t EVEN = 1u;
      static constexpr Reg32_t ODD  = 2u;
    }    // namespace Parity

    namespace Modes
    {
      static constexpr Reg32_t RX    = 0u;
      static constexpr Reg32_t TX    = 1u;
      static constexpr Reg32_t TX_RX = 2u;
    }    // namespace Modes

    namespace Clock
    {
      static constexpr Reg32_t CLOCK_DISABLE = 0u;
      static constexpr Reg32_t CLOCK_ENABLE  = 1u;
    }    // namespace Clock

    namespace Polarity
    {
      static constexpr Reg32_t POLARITY_LOW  = 0u;
      static constexpr Reg32_t POLARITY_HIGH = 1u;
    }    // namespace Polarity

    namespace Phase
    {
      static constexpr Reg32_t PHASE_1EDGE = 0u;
      static constexpr Reg32_t PHASE_2EDGE = 1u;
    }    // namespace Phase

    namespace LastBit
    {
      static constexpr Reg32_t LASTBIT_DISABLE = 0u;
      static constexpr Reg32_t LASTBIT_ENABLE  = 1u;
    }    // namespace LastBit

    namespace Nack
    {
      static constexpr Reg32_t NACK_ENABLE  = 1u;
      static constexpr Reg32_t NACK_DISABLE = 0u;
    }    // namespace Nack

    namespace Flags
    {
      static constexpr Reg32_t FLAG_CTS  = ( 1u << 0 );
      static constexpr Reg32_t FLAG_LBD  = ( 1u << 1 );
      static constexpr Reg32_t FLAG_TXE  = ( 1u << 2 );
      static constexpr Reg32_t FLAG_TC   = ( 1u << 3 );
      static constexpr Reg32_t FLAG_RXNE = ( 1u << 4 );
      static constexpr Reg32_t FLAG_IDLE = ( 1u << 5 );
      static constexpr Reg32_t FLAG_ORE  = ( 1u << 6 );
      static constexpr Reg32_t FLAG_NF   = ( 1u << 7 );
      static constexpr Reg32_t FLAG_FE   = ( 1u << 8 );
      static constexpr Reg32_t FLAG_PE   = ( 1u << 9 );
    }    // namespace Flags
  }      // namespace Configuration

  namespace Runtime
  {
    using Flag_t = Reg32_t;
    namespace Flag
    {
      /* Let the first 25 bits match the Status Register for consistency */
      static constexpr Flag_t RX_PARITY_ERROR   = ( 1u << 0 );
      static constexpr Flag_t RX_FRAMING_ERROR  = ( 1u << 1 );
      static constexpr Flag_t RX_NOISE_ERROR    = ( 1u << 2 );
      static constexpr Flag_t RX_OVERRUN        = ( 1u << 3 );
      static constexpr Flag_t RX_IDLE_DETECTED  = ( 1u << 4 );
      static constexpr Flag_t RX_BYTE_READY     = ( 1u << 5 );
      static constexpr Flag_t TX_COMPLETE       = ( 1u << 6 );
      static constexpr Flag_t TX_DR_EMPTY       = ( 1u << 7 );
      static constexpr Flag_t RX_LINE_IN_BREAK  = ( 1u << 8 );
      static constexpr Flag_t CTL_CLEAR_TO_SEND = ( 1u << 9 );

      /* Use the remaining 6 bits for other signals */
      static constexpr Flag_t RX_LINE_IDLE_ABORT = ( 1u << 26 );
      static constexpr Flag_t RX_COMPLETE        = ( 1u << 27 );
    }    // namespace Flag
  }      // namespace Runtime


  /*-------------------------------------------------------------------------------
  Structures
  -------------------------------------------------------------------------------*/
  struct RegisterMap
  {
    uint32_t placeholderReg;
  };


  /*-------------------------------------------------------------------------------
  External Variables
  -------------------------------------------------------------------------------*/
  extern std::array<RegisterMap*, NUM_USART_PERIPHS> PeripheralRegisterMaps;
  extern Thor::LLD::RIndexMap InstanceToResourceIndex;
  extern Chimera::Container::LightFlatMap<Chimera::Serial::Channel, RegisterMap*> ChannelToInstance;


  extern std::array<uint32_t, static_cast<size_t>( Chimera::Serial::CharWid::NUM_OPTIONS )> CharWidToRegConfig;
  extern std::array<uint32_t, static_cast<size_t>( Chimera::Serial::Parity::NUM_OPTIONS )> ParityToRegConfig;
  extern std::array<uint32_t, static_cast<size_t>( Chimera::Serial::StopBits::NUM_OPTIONS )> StopBitsToRegConfig;


  /*-------------------------------------------------------------------------------
  Public Functions
  -------------------------------------------------------------------------------*/
  void initializeMapping();
  void initializeRegisters();

}  // namespace Thor::LLD::USART

#endif  /* !THOR_LLD_USART_MOCK_VARIANT_HPP */
