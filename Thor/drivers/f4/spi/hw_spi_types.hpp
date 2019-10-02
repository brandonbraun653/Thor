/********************************************************************************
 *   File Name:
 *    hw_spi_types.hpp
 *
 *   Description:
 *    STM32 Types for the SPI Peripheral
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_HW_SPI_TYPES_HPP
#define THOR_HW_SPI_TYPES_HPP

/* C++ Includes */
#include <cstdint>

/* Chimera Includes */
#include <Chimera/types/spi_types.hpp>

/* Driver Includes */
#include <Thor/headers.hpp>
#include <Thor/drivers/f4/spi/hw_spi_prj.hpp>

#if defined( TARGET_STM32F4 ) && ( THOR_DRIVER_SPI == 1 )

namespace Thor::Driver::SPI
{
  struct RegisterMap
  {
    volatile uint32_t CR1;      /**< Control Register 1,            Address Offset: 0x00 */
    volatile uint32_t CR2;      /**< Control Register 2,            Address Offset: 0x04 */
    volatile uint32_t SR;       /**< Status Register,               Address Offset: 0x08 */
    volatile uint32_t DR;       /**< Data Register,                 Address Offset: 0x0C */
    volatile uint32_t CRCPR;    /**< CRC Polynomial Register,       Address Offset: 0x10 */
    volatile uint32_t RXCRCR;   /**< RX CRC Register,               Address Offset: 0x14 */
    volatile uint32_t TXCRCR;   /**< TX CRC Register,               Address Offset: 0x18 */
    volatile uint32_t I2SCFGR;  /**< I2S Configuration Register,    Address Offset: 0x1C */
    volatile uint32_t I2SPR;    /**< I2S Prescale Register,         Address Offset: 0x20 */
  };

  static RegisterMap *const SPI1_PERIPH = reinterpret_cast<RegisterMap *const>( SPI1_BASE_ADDR );
  static RegisterMap *const SPI2_PERIPH = reinterpret_cast<RegisterMap *const>( SPI2_BASE_ADDR );
  static RegisterMap *const SPI3_PERIPH = reinterpret_cast<RegisterMap *const>( SPI3_BASE_ADDR );
  static RegisterMap *const SPI4_PERIPH = reinterpret_cast<RegisterMap *const>( SPI4_BASE_ADDR );


  /**
   *  Checks if the given address belongs to a peripheral instance
   *
   *  @return bool
   */
  bool isSPI( const std::uintptr_t address );

  /*------------------------------------------------
  Configuration
  ------------------------------------------------*/
  namespace Configuration
  {
    
  }

  /*------------------------------------------------
  Runtime
  ------------------------------------------------*/
  namespace Runtime
  {
    using Flag_t = uint32_t;
    namespace Flag
    {
//      /* Let the first 16 bits match the Status Register for consistency */
//      static constexpr Flag_t RX_PARITY_ERROR   = Configuration::Flags::FLAG_PE;
//      static constexpr Flag_t RX_FRAMING_ERROR  = Configuration::Flags::FLAG_FE;
//      static constexpr Flag_t RX_NOISE_ERROR    = Configuration::Flags::FLAG_NF;
//      static constexpr Flag_t RX_OVERRUN        = Configuration::Flags::FLAG_ORE;
//      static constexpr Flag_t RX_IDLE_DETECTED  = Configuration::Flags::FLAG_IDLE;
//      static constexpr Flag_t RX_BYTE_READY     = Configuration::Flags::FLAG_RXNE;
//      static constexpr Flag_t TX_COMPLETE       = Configuration::Flags::FLAG_TC;
//      static constexpr Flag_t TX_DR_EMPTY       = Configuration::Flags::FLAG_TXE;
//      static constexpr Flag_t RX_LINE_IN_BREAK  = Configuration::Flags::FLAG_LBD;
//      static constexpr Flag_t CTL_CLEAR_TO_SEND = Configuration::Flags::FLAG_CTS;
//
//      /* Use the remaining 16 bits for other signals */
//      static constexpr Flag_t RX_LINE_IDLE_ABORT = ( 1u << 16 );
//      static constexpr Flag_t RX_COMPLETE        = ( 1u << 17 );
    }    // namespace Flag
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

  /*------------------------------------------------
  Control Register 1
  ------------------------------------------------*/
  namespace CR1
  {

  }

  /*------------------------------------------------
  Control Register 2
  ------------------------------------------------*/
  namespace CR2
  {

  }

  /*------------------------------------------------
  Status Register
  ------------------------------------------------*/
  namespace SR
  {

  }

  /*------------------------------------------------
  Data Register
  ------------------------------------------------*/
  namespace DR
  {

  }

  /*------------------------------------------------
  CRC Polynomial Register
  ------------------------------------------------*/
  namespace CRCPR
  {

  }

  /*------------------------------------------------
  RX CRC Register
  ------------------------------------------------*/
  namespace RXCRCR
  {
  }

  /*------------------------------------------------
  TX CRC Register
  ------------------------------------------------*/
  namespace TXCRCR
  {
  }

  /*------------------------------------------------
  I2S Configuration Register
  ------------------------------------------------*/
  namespace I2SCFGR
  {
  }

  /*------------------------------------------------
  I2S Prescale Register
  ------------------------------------------------*/
  namespace I2SPR
  {
  }

}    // namespace Thor::Driver::SPI

#endif /* TARGET_STM32F4 && THOR_DRIVER_SPI */
#endif /* THOR_HW_SPI_REGISTER_HPP */