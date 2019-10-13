/********************************************************************************
 *  File Name:
 *    system_checks.hpp
 *
 *  Description:
 *    Provides assertions on various assumptions made about the Chimera library
 *    so that random bugs don't suddenly show up.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef THOR_SYSTEM_CHECKS_HPP
#define THOR_SYSTEM_CHECKS_HPP

/* Chimera Includes */
#include <Chimera/types/common_types.hpp>
#include <Chimera/types/serial_types.hpp>

namespace Thor
{
  /*------------------------------------------------
  Validate the SubPeripheral enum: Used as array accessors
  ------------------------------------------------*/
  static_assert( static_cast<uint8_t>( Chimera::Hardware::SubPeripheral::RX ) == 0, "RX SubPeripheral accessor invalid" );
  static_assert( Chimera::Hardware::SubPeripheral::RX != Chimera::Hardware::SubPeripheral::TXRX, "RX == TXRX" );
  static_assert( Chimera::Hardware::SubPeripheral::RX != Chimera::Hardware::SubPeripheral::TX, "RX == TX" );

  static_assert( static_cast<uint8_t>( Chimera::Hardware::SubPeripheral::TX ) == 1, "RX SubPeripheral accessor invalid" );
  static_assert( Chimera::Hardware::SubPeripheral::TX != Chimera::Hardware::SubPeripheral::TXRX, "TX == TXRX" );

  static_assert( static_cast<uint8_t>( Chimera::Hardware::SubPeripheral::TXRX ) == 2, "RX SubPeripheral accessor invalid" );

  /*------------------------------------------------
  Validate the SubPeripheralMode enum: Used as array accessors
  ------------------------------------------------*/
  static_assert( static_cast<uint8_t>( Chimera::Hardware::PeripheralMode::BLOCKING ) == 0, "Blocking mode accessor invalid" );
  static_assert( Chimera::Hardware::PeripheralMode::BLOCKING != Chimera::Hardware::PeripheralMode::INTERRUPT, "Blocking mode == interrupt mode" );
  static_assert( Chimera::Hardware::PeripheralMode::BLOCKING != Chimera::Hardware::PeripheralMode::DMA, "Blocking mode == DMA mode" );

  static_assert( static_cast<uint8_t>( Chimera::Hardware::PeripheralMode::INTERRUPT ) == 1, "Interrupt mode accessor invalid" );
  static_assert( Chimera::Hardware::PeripheralMode::INTERRUPT != Chimera::Hardware::PeripheralMode::DMA, "Interrupt mode == DMA mode" );

  static_assert( static_cast<uint8_t>( Chimera::Hardware::PeripheralMode::DMA ) == 2, "DMA mode accessor invalid" );
}

namespace Thor::Serial
{
  
}    // namespace Thor::Serial


#endif /* !THOR_SYSTEM_CHECKS_HPP */